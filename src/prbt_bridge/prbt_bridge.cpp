#include "prbt_bridge/prbt_bridge.h"

/****************************************************************************************************************************
 *                                                                                                                          *
 *                                                       NUOVA IDEA                                                         *
 *                                                                                                                          *
 ****************************************************************************************************************************
 *                                                                                                                          *
 *  1. Prima traiettoria ricevuta viene inviata interamente al manipolatore                                                 *
 *                                                                                                                          *
 *  2. Un contatore tiene traccia del punto della traiettoria raggiunto dal manipolatore                                    *
 *      - controllo sul tempo ?                                                                                             *
 *      - controllo sulla posizione ?                                                                                       *
 *      - esiste un topic in cui posso vedere il punto della traiettoria raggiunto (actual/desired ?)?                      *
 *                                                                                                                          *
 *  3. Quando ricevo una nuova traietoria (dynamic replanning) elimino tutti i punti che sono già stati eseguiti            *
 *      - creo un nuovo vector traiettoria che contiene solo i punti che vanno da actual_point a end                        *
 *                                                                                                                          *
 *  4. Invio la nuova traiettoria al manipolatore sovrascrivendo quella precedente                                          *
 *                                                                                                                          *
 *  3b. In caso la traiettoria finisca devo effettuare un controllo                                                         *
 *      - quando ricevo una nuova traiettoria controllo che combaci con quella precedente e pubblico dal primo punto        *
 *        diverso (posso farlo anche nel dynamic replanning)                                                                *
 *                                                                                                                          *
 ***************************************************************************************************************************/



//----------------------------------------------------- CONSTRUCTOR -----------------------------------------------------//


prbt_bridge::prbt_bridge () {

    nh.param("/prbt_Bridge_Node/simulation", simulation, false);

    current_position.joint_names = {"empty", "empty", "empty", "empty", "empty", "empty"};

// Initialize Publishers and Subscribers

    trajectory_subscriber = nh.subscribe("/Robot_Bridge/prbt_Planned_Trajectory", 1000, &prbt_bridge::Planned_Trajectory_Callback, this);
    dynamic_trajectory_subscriber = nh.subscribe("/Robot_Bridge/prbt_Dynamic_Trajectory", 1000, &prbt_bridge::Dynamic_Trajectory_Callback, this);
    single_point_trajectory_subscriber = nh.subscribe("/Robot_Bridge/prbt_Single_Point_Trajectory", 1000, &prbt_bridge::Single_Point_Trajectory_Callback, this);
    current_position_subscriber = nh.subscribe("/prbt/manipulator_joint_trajectory_controller/state", 1000, &prbt_bridge::Current_Position_Callback, this);

    trajectory_counter_publisher = nh.advertise<std_msgs::Int32>("/Robot_Bridge/prbt_Trajectory_Counter", 1);
    current_state_position_publisher = nh.advertise<control_msgs::JointTrajectoryControllerState>("/Robot_Bridge/prbt_Current_State_Position", 1000);
    prbt_position_reached_publisher = nh.advertise<std_msgs::Bool>("/Robot_Bridge/prbt_Position_Reached", 1);
    manipulator_joint_trajectory_controller_command_publisher = nh.advertise<trajectory_msgs::JointTrajectory>("/prbt/manipulator_joint_trajectory_controller/command", 1);


// Trajectory Control Actions

    trajectory_client = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("/prbt/manipulator_joint_trajectory_controller/follow_joint_trajectory", true);

// Operation Mode

    prbt_unhold_client = nh.serviceClient<std_srvs::Trigger>("/prbt/manipulator_joint_trajectory_controller/unhold");
    prbt_hold_client = nh.serviceClient<std_srvs::Trigger>("/prbt/manipulator_joint_trajectory_controller/hold");
    prbt_monitor_cartesian_speed_client = nh.serviceClient<std_srvs::SetBool>("/prbt/manipulator_joint_trajectory_controller/monitor_cartesian_speed");
    get_speed_override_client = nh.serviceClient<pilz_msgs::GetSpeedOverride>("/prbt/get_speed_override");
    operation_mode_publisher = nh.advertise<prbt_hardware_support::OperationModes>("/prbt/operation_mode", 1);

    position_reached.data = false;
    new_static_trajectory_received = false;
    new_dynamic_trajectory_received = false;
    first_single_planning = true;

    last_message = ros::Time::now();

}


//------------------------------------------------------ CALLBACK ------------------------------------------------------//


void prbt_bridge::Planned_Trajectory_Callback (const trajectory_msgs::JointTrajectory::ConstPtr &msg) {
    
    new_static_trajectory_received = true;
    static_planning = true;
    planned_trajectory = *msg;

    last_message = ros::Time::now();

}

void prbt_bridge::Single_Point_Trajectory_Callback (const trajectory_msgs::JointTrajectory::ConstPtr &msg) {
    
    new_single_point_trajectory_received = true;
    single_planning = true;
    planned_trajectory = *msg;

    last_message = ros::Time::now();

}

void prbt_bridge::Dynamic_Trajectory_Callback (const trajectory_msgs::JointTrajectory::ConstPtr &msg) {
   
    new_dynamic_trajectory_received = true;
    dynamic_planning = true;
    planned_trajectory = *msg;

    last_message = ros::Time::now();

}

void prbt_bridge::Current_Position_Callback (const control_msgs::JointTrajectoryControllerState::ConstPtr &msg) {

    current_position = *msg;
    current_state_position_publisher.publish(current_position);

}


//----------------------------------------------------- PUBLISHER ------------------------------------------------------//


void prbt_bridge::Publish_Next_Goal (trajectory_msgs::JointTrajectory goal) {

    int end = goal.points.size() - 1;
    ROS_INFO("PRBT GOTO (%.2f;%.2f;%.2f;%.2f;%.2f;%.2f)", goal.points[end].positions[0], goal.points[end].positions[1], goal.points[end].positions[2], goal.points[end].positions[3], goal.points[end].positions[4], goal.points[end].positions[5]);
    ROS_INFO("Sampling Time: %.2f", sampling_time);

    if (simulation) {

        manipulator_joint_trajectory_controller_command_publisher.publish(goal);


    } else {

        trajectory_goal.trajectory = goal;
        trajectory_client -> waitForServer();
        trajectory_client -> sendGoal(trajectory_goal);

    }

    

}

void prbt_bridge::Publish_Trajectory_Counter (int counter) {

    std_msgs::Int32 count;
    count.data = counter;
    
    ROS_INFO("Trajectory Counter: %d", count.data);
    trajectory_counter_publisher.publish(count);

}


//----------------------------------------------------- FUNCTIONS -----------------------------------------------------//

void prbt_bridge::Check_Joint_Limits (trajectory_msgs::JointTrajectory *point) {

    float joint_limits[6] = {2.967, 2.095, 2.356, 2.967, 2.966, 3.123};

    for (int i = 0; i < 6; i++) {

        if (point->points[0].positions[i] > joint_limits[i]) {
            
            point->points[0].positions[i] = joint_limits[i];

            ROS_WARN("Joint_%d Limit Exceeded [MAX = %.3f]", i+1, joint_limits[i]);
            
        } else if (point->points[0].positions[i] < -joint_limits[i]) {

            point->points[0].positions[i] = -joint_limits[i];

            ROS_WARN("Joint_%d Limit Exceeded [MIN = %.3f]", i+1, -joint_limits[i]);

        }

    }

}

void prbt_bridge::Next_Goal (trajectory_msgs::JointTrajectory planned_trajectory, int counter/*, float sampling_time*/) {

    next_point.joint_names = planned_trajectory.joint_names;
    next_point.points.resize(1);
    next_point.points[0] = planned_trajectory.points[counter];
    next_point.points[0].time_from_start = ros::Duration (sampling_time);

    Check_Joint_Limits(&next_point);

}

void prbt_bridge::Compute_Tolerance (trajectory_msgs::JointTrajectory planned_trajectory) {

    if (planned_trajectory.points[0].time_from_start.toSec() == 0) {

        sampling_time = fabs((planned_trajectory.points[1].time_from_start).toSec());

    } else {sampling_time = fabs((planned_trajectory.points[0].time_from_start).toSec());}
    

    /*

    1° Equation (matlab polyfit with 4 sperimental points):     y = m*x + q             ->      y = 0.2057*x - 0.0027
    2° Equation (matlab polyfit with 4 sperimental points):     y = a*x^2 + b*x + c     ->      y = -0.0368 * x^2 + 0.2255 *x - 0.0035

    Points (sampling time, tolerance):

    A(0.01, 0.0001)     A(0.05, 0.005)     B(0.1, 0.02)     C(0.5, 0.1)
    
    */

    float m = 0.2057, q = -0.0027;
    float a = -0.0368, b = 0.2255, c = -0.0035;

    // tolerance_temp = fabs(a*pow(sampling_time,2) + b*sampling_time + c);
    float tolerance_temp = fabs(m * sampling_time + q);
    tolerance = std::min(tolerance_temp,float(0.02));

    // ROS_INFO("Sampling Time: %f",sampling_time);
    ROS_INFO("Position Tolerance: %f",tolerance);

}

float prbt_bridge::Compute_Position_Error (trajectory_msgs::JointTrajectory point) {

    std::vector<float> error {0,0,0,0,0,0};
    for (int i = 0; i < 6; i++) {error[i] = fabs(current_position.desired.positions[i] - point.points[0].positions[i]);}

    return (*std::max_element(error.begin(), error.end()));

}

void prbt_bridge::Wait_For_Desired_Position (bool dynamic) {

    ros::Time begin = ros::Time::now();

    if (dynamic) {

        position_error = tolerance + 1;     // needed to enter the while condition

        // Wait for Reaching Desired Position
        while ((position_error > tolerance) || (fabs((begin - ros::Time::now()).toSec()) < (sampling_time * 0.85))) {    
            
            ros::spinOnce();

            position_error = Compute_Position_Error(next_point);

            if (fabs((begin - ros::Time::now()).toSec()) > 2 * sampling_time) {
            
                ROS_ERROR("Wait for Desired Position - Error.");
                ROS_ERROR("Stop Trajectory Execution.");

                // Force While Exit
                trajectory_counter = planned_trajectory.points.size();

                break;

            }

        } 

    } else {
        
        while (fabs((begin - ros::Time::now()).toSec()) < sampling_time * planned_trajectory.points.size()) {
            
            ros::spinOnce();

            if (new_static_trajectory_received || new_dynamic_trajectory_received || new_single_point_trajectory_received) {break;}
            
        }
    
    }

}


//------------------------------------------------------- MAIN --------------------------------------------------------//


void prbt_bridge::spinner (void) {

    ros::spinOnce();

    if (dynamic_planning && static_planning) {

        ROS_ERROR("You cant' publish on \"/prbt_Planned_Trajectory\" and \"prbt_Dynamic_Trajectory\" at the same time");

        dynamic_planning = false;
        static_planning = false;
        single_planning = false;
        new_static_trajectory_received = false;
        new_dynamic_trajectory_received = false;
        new_single_point_trajectory_received = false;

        planned_trajectory.points.clear();

    } else if (dynamic_planning) {

        trajectory_counter = 0;
        new_dynamic_trajectory_received = false;
        new_static_trajectory_received = false;
        new_single_point_trajectory_received = false;

        while ((trajectory_counter < planned_trajectory.points.size()) && (planned_trajectory.points.size() != 0)) {

            if (trajectory_counter == 0) {  // New Trajectory

                Compute_Tolerance(planned_trajectory);

                if (!simulation) {

                    while (get_speed_override_client.call(get_speed_override_srv) && (get_speed_override_srv.response.speed_override == 0.0)) {

                        // Set Operation Mode to AUTO (Mode 3)
                        prbt_hardware_support::OperationModes operation_mode;
                        operation_mode.time_stamp = ros::Time::now();
                        operation_mode.value = 3; //AUTO
                        operation_mode_publisher.publish(operation_mode);
                        
                    }

                    // Turn off Hold Mode
                    if (prbt_unhold_client.call(prbt_unhold_srv)) {ROS_INFO("Hold Mode Deactivated");} else {ROS_ERROR("Failed to Call Service: \"prbt_unhold\"");}

                }

            }

            // Final Position NOT Reached
            position_reached.data = false;
            prbt_position_reached_publisher.publish(position_reached);

            // Check the Trajectory for Dynamic Replanning
            ros::spinOnce();

            // Break if a New Static Trajectory is Received
            if (new_static_trajectory_received || new_single_point_trajectory_received) {break;}
            else if (new_dynamic_trajectory_received) {new_dynamic_trajectory_received = false;}

            // Compute Next Goal
            Next_Goal(planned_trajectory, trajectory_counter);  
            Publish_Next_Goal(next_point);
            Publish_Trajectory_Counter(trajectory_counter); 
            
            // Wait Until Position is Reached
            Wait_For_Desired_Position(true);

            trajectory_counter++;

        }

        if (trajectory_counter != 0) {

            if (!new_static_trajectory_received && !new_single_point_trajectory_received) {

                // Final Position Reached
                position_reached.data = true;
                prbt_position_reached_publisher.publish(position_reached);

                if (!simulation) {

                    // Turn on Hold Mode (Sleep Necessary for Stop Precision)
                    ros::Duration(1).sleep();
                    if (prbt_hold_client.call(prbt_hold_srv)) {ROS_INFO("Hold Mode Activated");} else {ROS_ERROR("Failed to Call Service: \"prbt_hold\"");}
                
                }

                planned_trajectory.points.clear();
                
            }

            next_point.points.clear();
            trajectory_counter = 0;
            if (!new_dynamic_trajectory_received) {dynamic_planning = false;}

        }
    
    } else if (static_planning) {

        new_static_trajectory_received = false;
        new_dynamic_trajectory_received = false;
        new_single_point_trajectory_received = false;
        
        if (!simulation) {

            while (get_speed_override_client.call(get_speed_override_srv) && (get_speed_override_srv.response.speed_override == 0.0)) {

                // Set Operation Mode to AUTO (Mode 3)
                prbt_hardware_support::OperationModes operation_mode;
                operation_mode.time_stamp = ros::Time::now();
                operation_mode.value = 3; //AUTO
                operation_mode_publisher.publish(operation_mode);
                
            }

            // Turn off Hold Mode
            if (prbt_unhold_client.call(prbt_unhold_srv)) {ROS_INFO("Hold Mode Deactivated");} else {ROS_ERROR("Failed to Call Service: \"prbt_unhold\"");}

        }

        Compute_Tolerance(planned_trajectory);
        Check_Joint_Limits(&planned_trajectory);
        Publish_Next_Goal(planned_trajectory);

        // Wait Until Position is Reached
        Wait_For_Desired_Position(false);

        if (!new_dynamic_trajectory_received && !new_static_trajectory_received && !new_single_point_trajectory_received) {

            //Final Position Reached
            position_reached.data = true;
            prbt_position_reached_publisher.publish(position_reached);

            if (!simulation) {

                // Turn on Hold Mode (Sleep Necessary for Stop Precision)
                ros::Duration(1).sleep();
                if (prbt_hold_client.call(prbt_hold_srv)) {ROS_INFO("Hold Mode Activated");} else {ROS_ERROR("Failed to Call Service: \"prbt_hold\"");}
            
            }
            
            planned_trajectory.points.clear();
            static_planning = false;
        
        } else if ((new_dynamic_trajectory_received || new_single_point_trajectory_received) && !new_static_trajectory_received) {

            static_planning = false;

        }

    } else if (single_planning) {

        if (first_single_planning && !simulation) {

            while (get_speed_override_client.call(get_speed_override_srv) && (get_speed_override_srv.response.speed_override == 0.0)) {

                // Set Operation Mode to AUTO (Mode 3)
                prbt_hardware_support::OperationModes operation_mode;
                operation_mode.time_stamp = ros::Time::now();
                operation_mode.value = 3; //AUTO
                operation_mode_publisher.publish(operation_mode);
            
            }

            // Turn off Hold Mode
            if (prbt_unhold_client.call(prbt_unhold_srv)) {ROS_INFO("Hold Mode Deactivated");} else {ROS_ERROR("Failed to Call Service: \"prbt_unhold\"");}

            first_single_planning = false;

        }

        // Check_Joint_Limits(&planned_trajectory);
        Publish_Next_Goal(planned_trajectory);

        single_planning = false;
    
    }

    if ((last_message.toSec() - ros::Time::now().toSec()) > 5) {

        last_message = ros::Time::now();

        if (!simulation) {

            // Turn on Hold Mode (Sleep Necessary for Stop Precision)
            if (prbt_hold_client.call(prbt_hold_srv)) {ROS_INFO("Hold Mode Activated");} else {ROS_ERROR("Failed to Call Service: \"prbt_hold\"");}
        
        }

        first_single_planning = false;
        static_planning = false; dynamic_planning = false; single_planning = false;
        new_dynamic_trajectory_received = false; new_static_trajectory_received = false; new_single_point_trajectory_received = false;

    }

}
