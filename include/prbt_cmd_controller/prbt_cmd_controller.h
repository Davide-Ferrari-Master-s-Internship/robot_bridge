#ifndef PRBT_CMD_CONTROLLER_H
#define PRBT_CMD_CONTROLLER_H

#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <time.h>
#include <math.h>


class controller_functions {

    public:

        controller_functions();
        
        bool abort;

        void prbt_Planning_Request (void);
        void prbt_Plan (void);
        void prbt_GOTO (void);

        void prbt_Test_Trajectory (void);

    private:

        ros::NodeHandle nh;
        ros::Publisher manipulator_joint_trajectory_controller_command_publisher;

        trajectory_msgs::JointTrajectory planned_trajectory;

        bool vel_request;
        float time;

        double pos_joint_1, pos_joint_2, pos_joint_3, pos_joint_4, pos_joint_5, pos_joint_6;
        double vel_joint_1, vel_joint_2, vel_joint_3, vel_joint_4, vel_joint_5, vel_joint_6;
        double acc_joint_1, acc_joint_2, acc_joint_3, acc_joint_4, acc_joint_5, acc_joint_6;
        
        void joint_trajectory_duration (float duration);
        void joint_trajectory_positions (double joint_1, double joint_2, double joint_3, double joint_4, double joint_5, double joint_6);
        void joint_trajectory_velocities (double joint_1, double joint_2, double joint_3, double joint_4, double joint_5, double joint_6);
        void joint_trajectory_accelerations (double joint_1, double joint_2, double joint_3, double joint_4, double joint_5, double joint_6);
        void joint_trajectory_effort (double joint_1, double joint_2, double joint_3, double joint_4, double joint_5, double joint_6);

};

#endif /* PRBT_CMD_CONTROLLER_H */