//
// Created by lucas on 28.03.23.
//

#include <moveit/move_group_interface/move_group_interface.h>
#include <franka_gripper/franka_gripper.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "utility.h"

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

