//
// Created by lucas on 28.03.23.
//

#include <moveit/move_group_interface/move_group_interface.h>
#include <franka_gripper/franka_gripper.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "utility.h"
#include "Eigen/Dense"

Eigen::Matrix<double, 6, 1> convert_pose_to_eigen(const geometry_msgs::Pose& msg){
	Eigen::Matrix<double, 6, 1> pose;
	pose.head(3) << msg.position.x, msg.position.y, msg.position.z;
	Eigen::Quaterniond orientation;
	orientation.coeffs() << msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w;
	pose.tail(3) = orientation.toRotationMatrix().eulerAngles(0, 1, 2);
	return pose;
}



geometry_msgs::Pose createGoalPose(std::vector<double> position, std::vector<double> orientation){
    geometry_msgs::Pose target_pose;
    tf2::Quaternion rotation;
    rotation.setRPY(orientation[0], orientation[1], orientation[2]);
    target_pose.orientation = tf2::toMsg(rotation);
    target_pose.position.x = position[0];
    target_pose.position.y = position[1];
    target_pose.position.z = position[2];

    return target_pose;
}

franka_gripper::GraspGoal createGraspGoal(double width, double force, double eps, double speed){
    //grasp goal (close gripper)
    franka_gripper::GraspGoal grasp_goal;
    grasp_goal.force = force;
    grasp_goal.epsilon.inner = eps;
    grasp_goal.epsilon.outer = eps;
    grasp_goal.width = width;
    grasp_goal.speed = speed;

    return grasp_goal;
}


void setPlanningParameters(moveit::planning_interface::MoveGroupInterface& move_group, double plan_time, int attempts, double max_v, double max_a){
    move_group.setPlanningTime(plan_time);
    move_group.setNumPlanningAttempts(attempts);
    move_group.setMaxVelocityScalingFactor(max_v);
    move_group.setMaxAccelerationScalingFactor(max_a);

}


void move(ros::Publisher& commander, std::vector<double> position, std::vector<double> orientation){
    geometry_msgs::Pose target_pose;
    target_pose = createGoalPose(position, orientation);
    commander.publish(target_pose);
    ROS_INFO_STREAM("published pose " << target_pose.position << " " << target_pose.orientation);
}

void open_gripper(ros::Publisher& gripper_move_publisher, double speed, double width){
    franka_gripper::MoveAction gripper_open;
    gripper_open.action_goal.goal.width = width;
    gripper_open.action_goal.goal.speed = speed;
    gripper_move_publisher.publish(gripper_open);
    ros::Duration(0.001).sleep();
}

void grasp_object(ros::Publisher& grasp_publisher, double speed=0.1, double width=0.01, double force=25, double tol=0.005){
    franka_gripper::GraspAction grasp;
    grasp.action_goal.goal.speed = speed;
    grasp.action_goal.goal.width = width;
    grasp.action_goal.goal.force = force;
    grasp.action_goal.goal.epsilon.inner = tol;
    grasp.action_goal.goal.epsilon.outer = tol;
    grasp_publisher.publish(grasp);
}

