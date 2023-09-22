//
// Created by lucas on 22.09.23.
//

#include <franka_gripper/MoveAction.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/StopAction.h>
#include "ros/ros.h"
#include "utility.h"
#include <iostream>
#include <vector>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

#ifndef GOAL_STATE_PUBLISHER_DEMO_CLASSES_H
#define GOAL_STATE_PUBLISHER_DEMO_CLASSES_H

class TaskPlanner {
public:
    // Default constructor
    TaskPlanner(moveit::planning_interface::MoveGroupInterface* move_group,  moveit::planning_interface::MoveGroupInterface* gripper_group);
    ros::Publisher move_pub;
    ros::Publisher gripper_move_pub;
    ros::Publisher grasp_pub;
    ros::Publisher stop_pub;

    //methods
    void move(std::vector<double> position, std::vector<double> orientation);
    void open_gripper(double speed=0.1, double width=0.04);
    void grasp_object(double speed=0.1, double width=0.0, double force=25, double tol=0.005);

private:
    ros::NodeHandlePtr nh_;
    moveit::planning_interface::MoveGroupInterface* move_group_ptr; //or panda_arm
    moveit::planning_interface::MoveGroupInterface* gripper_group_ptr;
    actionlib::SimpleActionClient<franka_gripper::GraspAction> gripper_grasp_client;
    actionlib::SimpleActionClient<franka_gripper::MoveAction> gripper_move_client;
    actionlib::SimpleActionClient<franka_gripper::StopAction> gripper_stop_client;
};

#endif //GOAL_STATE_PUBLISHER_DEMO_CLASSES_H

