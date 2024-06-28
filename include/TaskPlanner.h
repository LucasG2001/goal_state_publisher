//
// Created by lucas on 22.09.23.
//

#include <franka_gripper/MoveAction.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/StopAction.h>
#include "ros/ros.h"
#include "Utility.h"
#include <iostream>
#include <vector>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <franka_msgs/FrankaState.h>
#include <Eigen/Dense>

#ifndef GOAL_STATE_PUBLISHER_DEMO_CLASSES_H
#define GOAL_STATE_PUBLISHER_DEMO_CLASSES_H

class TaskPlanner {
public:
    // Default constructor
	TaskPlanner();
    Eigen::Vector3d global_ee_position;
	Eigen::Vector3d global_ee_euler_angles;
    Eigen::Quaterniond global_ee_orientation;
    Eigen::Matrix<double, 6, 1> F_ext;
    ros::Publisher control_mode_pub; // controls control mode on high level (0/1 = normal/free float)
    ros::Publisher equilibrium_pose_pub;
    ros::Publisher grasp_pose_publisher;

    //methods
    void move(std::vector<double> position, std::vector<double> orientation, ros::Publisher* goal_pose_publisher, double tol = 0.04, std::string header_info = "none"); // equilibrium pose movement
	void primitive_move(Eigen::Matrix<double, 3, 1>goal_position, Eigen::Matrix<double, 3, 1> goal_orientation, ros::Publisher* goal_pose_publisher, double tol = 0.04, std::string header_info = "none") const; // equilibrium pose movement
	void stop(ros::Publisher* goal_pose_publisher, bool default_orientation = false) const; // equilibrium pose movement
	void execute_action(Eigen::Matrix<double, 3, 1>goal_position, Eigen::Matrix<double, 3, 1> goal_orientation, ros::Publisher* goal_pose_publisher, double tol = 0.04) const; // same as move for Eigen Maatrix INput
    void open_gripper(double speed=0.1, double width=0.08);
    void grasp_object(double speed=0.1, double width=0.0, double force=40, double tol=0.08);
    void ee_callback(const franka_msgs::FrankaStateConstPtr & msg);
	// Call this method to update the impedance parameters based on the active task
	void updateImpedanceParameters() const;

private:
    actionlib::SimpleActionClient<franka_gripper::GraspAction> gripper_grasp_client;
    actionlib::SimpleActionClient<franka_gripper::MoveAction> gripper_move_client;
    actionlib::SimpleActionClient<franka_gripper::StopAction> gripper_stop_client;

};

#endif //GOAL_STATE_PUBLISHER_DEMO_CLASSES_H

