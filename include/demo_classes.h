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
#include <franka_msgs/FrankaState.h>

#ifndef GOAL_STATE_PUBLISHER_DEMO_CLASSES_H
#define GOAL_STATE_PUBLISHER_DEMO_CLASSES_H

class TaskPlanner {
public:
    // Default constructor
    TaskPlanner(moveit::planning_interface::MoveGroupInterface* move_group,  moveit::planning_interface::MoveGroupInterface* gripper_group);
    Eigen::Vector3d global_ee_position;
    Eigen::Quaterniond global_ee_orientation;
    Eigen::Matrix<double, 6, 1> F_ext;
    ros::Publisher control_mode_pub; // controls control mode on high level (0/1 = normal/free float)
    ros::Publisher equilibrium_pose_pub;

    //methods
    void move(std::vector<double> position, std::vector<double> orientation, ros::Publisher* goal_pose_publisher, double tol = 0.04, bool clear_integrator=true); // equilibrium pose movement
    void multiplan_move(std::vector<double> position, std::vector<double> orientation); //plan multiple times and execute when successsfull with moveit
    void moveit_move(std::vector<double> position, std::vector<double> orientation); //plan and execute one time with moveit
    void open_gripper(double speed=0.1, double width=0.08);
    void grasp_object(double speed=0.1, double width=0.0, double force=40, double tol=0.08);
    void ee_callback(const franka_msgs::FrankaStateConstPtr & msg);

private:
    ros::NodeHandlePtr nh_;
    moveit::planning_interface::MoveGroupInterface* move_group_ptr; //or panda_arm
    actionlib::SimpleActionClient<franka_gripper::GraspAction> gripper_grasp_client;
    actionlib::SimpleActionClient<franka_gripper::MoveAction> gripper_move_client;
    actionlib::SimpleActionClient<franka_gripper::StopAction> gripper_stop_client;

};

#endif //GOAL_STATE_PUBLISHER_DEMO_CLASSES_H

