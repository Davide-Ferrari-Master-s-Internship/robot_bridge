#include "prbt_cmd_controller/prbt_cmd_controller.h"

controller_functions::controller_functions() {

    //Initialize publisher on "/prbt/manipulator_joint_trajectory_controller/command"
    // manipulator_joint_trajectory_controller_command_publisher = nh.advertise<trajectory_msgs::JointTrajectory>("/prbt/manipulator_joint_trajectory_controller/command", 1000);
    manipulator_joint_trajectory_controller_command_publisher = nh.advertise<trajectory_msgs::JointTrajectory>("/Robot_Bridge/prbt_Planned_Trajectory", 1000);

}


void controller_functions::joint_trajectory_positions (double joint_1, double joint_2, double joint_3, double joint_4, double joint_5, double joint_6) {
        
    // planned_trajectory.joint_names = {"pilzjoint_1", "pilzjoint_2", "pilzjoint_3", "pilzjoint_4", "pilzjoint_5", "pilzjoint_6"};
    planned_trajectory.joint_names = {"prbt_joint_1", "prbt_joint_2", "prbt_joint_3", "prbt_joint_4", "prbt_joint_5", "prbt_joint_6"};
    planned_trajectory.points.resize(1);
    planned_trajectory.points[0].positions = {joint_1, joint_2, joint_3, joint_4, joint_5, joint_6};

}

void controller_functions::joint_trajectory_duration (float duration) {
    
    ros::Duration time(duration);

    planned_trajectory.points[0].time_from_start = time;

}

void controller_functions::joint_trajectory_velocities (double joint_1, double joint_2, double joint_3, double joint_4, double joint_5, double joint_6) {

    planned_trajectory.points[0].velocities = {joint_1, joint_2, joint_3, joint_4, joint_5, joint_6};

}

void controller_functions::joint_trajectory_accelerations (double joint_1, double joint_2, double joint_3, double joint_4, double joint_5, double joint_6) {

    planned_trajectory.points[0].accelerations = {joint_1, joint_2, joint_3, joint_4, joint_5, joint_6};

}

void controller_functions::joint_trajectory_effort (double joint_1, double joint_2, double joint_3, double joint_4, double joint_5, double joint_6) {

    planned_trajectory.points[0].effort = {joint_1, joint_2, joint_3, joint_4, joint_5, joint_6};

}



void controller_functions::prbt_Planning_Request (void) {

    std::cout << "\nStart Planning [cm] \n\n" << "Insert Joint Position (6 Joints):\n\n";
    std::cin >> pos_joint_1;

    if (pos_joint_1 == 999) {

        std::cout << "\n";
        ROS_INFO("ROS Shutdown \n");
        ros::shutdown();
        abort = true;
        
    } else {

        std::cin >> pos_joint_2 >> pos_joint_3 >> pos_joint_4 >> pos_joint_5 >> pos_joint_6;
        std::cout << "\n";
        abort = false;

        std::cout << "Insert Movement Time: ";
        time = 1;
        std::cin >> time;
        std::cout << "\n";
    
    }

/*     std::cout << "\nInsert Velocities ? [S/N]: ";
    char response = 'n';
    std::cin >> response;

    if (response == 's' || response == 'S') {

        std::cout << "\n\nInsert Joint Velocities (6 Joints):\n\n";
        std::cin >> vel_joint_1 >> vel_joint_2 >> vel_joint_3 >> vel_joint_4 >> vel_joint_5 >> vel_joint_6;
        std::cout << "\n";
        vel_request = true;
        

    } else {

        std::cout << "\n";  
        vel_request = false;

    } */

 
}

void controller_functions::prbt_Plan (void) {

    ROS_INFO("Plan - Inizio");

    joint_trajectory_positions(pos_joint_1, pos_joint_2, pos_joint_3, pos_joint_4, pos_joint_5, pos_joint_6);

    joint_trajectory_duration(time);

    //if (vel_request == true) {joint_trajectory_velocities(vel_joint_1, vel_joint_2, vel_joint_3, vel_joint_4, vel_joint_5, vel_joint_6);}

    ROS_INFO("Plan - Fine");

}

void controller_functions::prbt_GOTO (void) {

    //publish velocity on cmd_vel

    ROS_INFO("PRBT GOTO (%.2f;%.2f;%.2f;%.2f;%.2f;%.2f)", pos_joint_1, pos_joint_2, pos_joint_3, pos_joint_4, pos_joint_5, pos_joint_6);
    ROS_INFO("Movement Time (%.2f)", time);
    manipulator_joint_trajectory_controller_command_publisher.publish(planned_trajectory);

}


void controller_functions::prbt_Test_Trajectory (void) {

    ROS_INFO("Test Trajectory");

    planned_trajectory.joint_names = {"prbt_joint_1", "prbt_joint_2", "prbt_joint_3", "prbt_joint_4", "prbt_joint_5", "prbt_joint_6"};
    
    planned_trajectory.points.resize(7);

    planned_trajectory.points[0].positions = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
    planned_trajectory.points[0].time_from_start = ros::Duration(2);
    planned_trajectory.points[1].positions = {1, 1, 1, 1, 1, 1};
    planned_trajectory.points[1].time_from_start = ros::Duration(2);
    planned_trajectory.points[2].positions = {1.5, 1.5, 1.5, 1.5, 1.5, 1.5};
    planned_trajectory.points[2].time_from_start = ros::Duration(2);

    planned_trajectory.points[3].positions = {1.5, 1.5, 1.5, 1.5, 1.5, 1.5};
    planned_trajectory.points[3].time_from_start = ros::Duration(4);

    planned_trajectory.points[4].positions = {1, 1, 1, 1, 1, 1};
    planned_trajectory.points[4].time_from_start = ros::Duration(2);
    planned_trajectory.points[5].positions = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
    planned_trajectory.points[5].time_from_start = ros::Duration(2);
    planned_trajectory.points[6].positions = {0, 0, 0, 0, 0, 0};
    planned_trajectory.points[6].time_from_start = ros::Duration(2);
    

    manipulator_joint_trajectory_controller_command_publisher.publish(planned_trajectory);

    ros::Duration(4.0).sleep();

    planned_trajectory.points[3].positions = {-1.5, 1.5, 1.5, 1.5, 1.5, 1.5};
    planned_trajectory.points[3].time_from_start = ros::Duration(4);

    planned_trajectory.points[4].positions = {-1, 1, 1, 1, 1, 1};
    planned_trajectory.points[4].time_from_start = ros::Duration(2);
    planned_trajectory.points[5].positions = {-0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
    planned_trajectory.points[5].time_from_start = ros::Duration(2);
    planned_trajectory.points[6].positions = {0, 0, 0, 0, 0, 0};
    planned_trajectory.points[6].time_from_start = ros::Duration(2);

    manipulator_joint_trajectory_controller_command_publisher.publish(planned_trajectory);

/*  
    //PROVA Video

    planned_trajectory.points.resize(1);

    planned_trajectory.points[0].positions = {1.5, -0.5, 1.1, 0, 0, 0};
    planned_trajectory.points[0].time_from_start = ros::Duration(8);
    

    manipulator_joint_trajectory_controller_command_publisher.publish(planned_trajectory);

    ros::Duration(10.0).sleep();

    planned_trajectory.points[0].positions = {0, 0, 0, 0, 0, 0};
    planned_trajectory.points[0].time_from_start = ros::Duration(8);

    manipulator_joint_trajectory_controller_command_publisher.publish(planned_trajectory); 
    
*/

}