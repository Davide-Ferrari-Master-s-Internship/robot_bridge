#include "robot_bridge/prbt/prbt_Bridge.h"


/****************************************************************************************************************************
 *                                                                                                                          *
 *                                                       NUOVA IDEA                                                         *
 *                                                                                                                          *
 ****************************************************************************************************************************
 *                                                                                                                          *
 *  0. Utilizzo delle ROS Action                                                                                            *
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

    nh.param("/prbt_Bridge_Node/action_decision", action_decision, true);
    //ROS_WARN("action decision %d", action_decision);

    current_position.joint_names = {"empty", "empty", "empty", "empty", "empty", "empty"};

// Initialize Publishers and Subscribers

    trajectory_subscriber = nh.subscribe("/Robot_Bridge/prbt_Planned_Trajectory", 1000, &prbt_bridge::Planned_Trajectory_Callback, this);
    dynamic_trajectory_subscriber = nh.subscribe("/Robot_Bridge/prbt_Dynamic_Trajectory", 1000, &prbt_bridge::Dynamic_Trajectory_Callback, this);
    current_position_subscriber = nh.subscribe("/prbt/manipulator_joint_trajectory_controller/state", 1000, &prbt_bridge::Current_Position_Callback, this);

    trajectory_counter_publisher = nh.advertise<std_msgs::Int32>("/Robot_Bridge/prbt_Trajectory_Counter", 1);
    current_state_position_publisher = nh.advertise<control_msgs::JointTrajectoryControllerState>("/Robot_Bridge/prbt_Current_State_Position", 1000);
    prbt_position_reached_publisher = nh.advertise<std_msgs::Bool>("/Robot_Bridge/prbt_Position_Reached", 1);    
    // manipulator_joint_trajectory_controller_command_publisher = nh.advertise<trajectory_msgs::JointTrajectory>("/prbt/manipulator_joint_trajectory_controller/command", 1);

// Trajectory Control Actions

    trajectory_client = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("/prbt/manipulator_joint_trajectory_controller/follow_joint_trajectory", true);
    manipulator_action_controller_publisher = nh.advertise<control_msgs::FollowJointTrajectoryActionGoal>("/prbt/manipulator_joint_trajectory_controller/follow_joint_trajectory/goal", 1);

// Operation Mode

    prbt_unhold_client = nh.serviceClient<std_srvs::Trigger>("/prbt/manipulator_joint_trajectory_controller/unhold");
    prbt_hold_client = nh.serviceClient<std_srvs::Trigger>("/prbt/manipulator_joint_trajectory_controller/hold");
    prbt_monitor_cartesian_speed_client = nh.serviceClient<std_srvs::SetBool>("/prbt/manipulator_joint_trajectory_controller/monitor_cartesian_speed");
    get_speed_override_client = nh.serviceClient<pilz_msgs::GetSpeedOverride>("/prbt/get_speed_override");
    operation_mode_publisher = nh.advertise<prbt_hardware_support::OperationModes>("/prbt/operation_mode", 1);

    trajectory_counter = 0;
    position_reached.data = false;

}


//------------------------------------------------------ CALLBACK ------------------------------------------------------//


void prbt_bridge::Planned_Trajectory_Callback (const trajectory_msgs::JointTrajectory::ConstPtr &msg) {

    planned_trajectory = *msg;
    dynamic_planning = false;

    while (get_speed_override_client.call(get_speed_override_srv) && (get_speed_override_srv.response.speed_override == 0.0)) {

        // set operation mode to auto
        prbt_hardware_support::OperationModes operation_mode;
        operation_mode.time_stamp = ros::Time::now();
        operation_mode.value = 3; //AUTO
        operation_mode_publisher.publish(operation_mode);
        
    }

    // Turn off Hold Mode
    if (prbt_unhold_client.call(prbt_unhold_srv)) {ROS_INFO("Hold Mode Deactivated");} else {ROS_ERROR("Failed to Call Service: \"prbt_unhold\"");}

    // Turn off Speed Monitoring (Useless in AUTO Mode)
    // prbt_monitor_cartesian_speed_srv.request.data = false;
    // if (prbt_monitor_cartesian_speed_client.call(prbt_monitor_cartesian_speed_srv)) {ROS_INFO("Speed Monitoring Deactivated");} else {ROS_ERROR("Failed to Call Service: \"prbt_monitor_cartesian_speed\"");}

    Publish_Next_Goal(planned_trajectory);

}

void prbt_bridge::Dynamic_Trajectory_Callback (const trajectory_msgs::JointTrajectory::ConstPtr &msg) {

    planned_trajectory = *msg;
    dynamic_planning = true;

}

void prbt_bridge::Current_Position_Callback (const control_msgs::JointTrajectoryControllerState::ConstPtr &msg) {

    current_position = *msg;
    current_state_position_publisher.publish(current_position);

}


//----------------------------------------------------- PUBLISHER ------------------------------------------------------//


void prbt_bridge::Publish_Next_Goal (trajectory_msgs::JointTrajectory goal) {

    int end = goal.points.size() - 1;
    ROS_INFO("PRBT GOTO (%.2f;%.2f;%.2f;%.2f;%.2f;%.2f)", goal.points[end].positions[0], goal.points[end].positions[1], goal.points[end].positions[2], goal.points[end].positions[3], goal.points[end].positions[4], goal.points[end].positions[5]);

    // manipulator_joint_trajectory_controller_command_publisher.publish(goal); // old control topic
    
    if (action_decision) {

        trajectory_goal.trajectory = goal;
        trajectory_client -> waitForServer();
        trajectory_client -> sendGoal(trajectory_goal);

    } else {

        trajectory_action.goal.trajectory = goal;
        manipulator_action_controller_publisher.publish(trajectory_action);

    }

}

void prbt_bridge::Publish_Trajectory_Counter (int counter) {

    ROS_INFO("Trajectory Counter: %d", counter);

    std_msgs::Int32 count;

    count.data = counter;

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

    // tolerance = fabs(a*pow(sampling_time,2) + b*sampling_time + c);
    tolerance = fabs(m * sampling_time + q);

    ROS_INFO("Sampling Time: %f",sampling_time);
    ROS_INFO("Position Tolerance: %f",tolerance);

}

float prbt_bridge::Compute_Position_Error (void) {

    std::vector<float> error {0,0,0,0,0,0};
    for (int i = 0; i < 6; i++) {error[i] = fabs(current_position.desired.positions[i] - next_point.points[0].positions[i]);}

    return (*std::max_element(error.begin(), error.end()));

}

void prbt_bridge::Wait_For_Desired_Position (void) {

    position_error = tolerance + 1;         //needed to enter the while condition  

    while ((position_error > tolerance) || (fabs((begin - ros::Time::now()).toSec()) < (sampling_time * 0.85))) {    //wait for reaching desired position
        
        ros::spinOnce();

        position_error = Compute_Position_Error();

    if (fabs((begin - ros::Time::now()).toSec()) > 10 * sampling_time) {
        
        ROS_ERROR("Wait for Desired Position Error.");
        ROS_ERROR("Stop Trajectory Execution.");

        // force while exit
        trajectory_counter = planned_trajectory.points.size();

        return;

        }

    }

}


//------------------------------------------------------- MAIN --------------------------------------------------------//


void prbt_bridge::spinner (void) {

    ros::spinOnce();

    if (dynamic_planning) {

        while ((trajectory_counter < planned_trajectory.points.size()) && (planned_trajectory.points.size() != 0)) {

            if (trajectory_counter == 0) {  //new trajectory

                Compute_Tolerance(planned_trajectory);

                while (get_speed_override_client.call(get_speed_override_srv) && (get_speed_override_srv.response.speed_override == 0.0)) {

                    // set operation mode to auto
                    prbt_hardware_support::OperationModes operation_mode;
                    operation_mode.time_stamp = ros::Time::now();
                    operation_mode.value = 3; //AUTO
                    operation_mode_publisher.publish(operation_mode);
                    
                }

                // Turn off Hold Mode
                if (prbt_unhold_client.call(prbt_unhold_srv)) {ROS_INFO("Hold Mode Deactivated");} else {ROS_ERROR("Failed to Call Service: \"prbt_unhold\"");}

                // Turn off Speed Monitoring
                prbt_monitor_cartesian_speed_srv.request.data = false;
                if (prbt_monitor_cartesian_speed_client.call(prbt_monitor_cartesian_speed_srv)) {ROS_INFO("Speed Monitoring Deactivated");} else {ROS_ERROR("Failed to Call Service: \"prbt_monitor_cartesian_speed\"");}

            }

            //final position not reached
            position_reached.data = false;
            prbt_position_reached_publisher.publish(position_reached);

            //check the trajectory for dynamic replanning
            ros::spinOnce();

            Next_Goal(planned_trajectory, trajectory_counter);  //compute next goal

            Publish_Trajectory_Counter(trajectory_counter); 
            Publish_Next_Goal(next_point);
            
            begin = ros::Time::now();
            Wait_For_Desired_Position();    //wait until the tolerance is achieved

            trajectory_counter++;

        }

        if (trajectory_counter != 0) {

                //final position reached
                position_reached.data = true;
                prbt_position_reached_publisher.publish(position_reached);

                // Turn on Hold Mode
                if (prbt_hold_client.call(prbt_hold_srv)) {ROS_INFO("Hold Mode Activated");} else {ROS_ERROR("Failed to Call Service: \"prbt_hold\"");}

                trajectory_counter = 0;

                planned_trajectory.points.clear();
                next_point.points.clear();

                idle_publisher = true;
                dynamic_planning = false;

        }
    
    }

}