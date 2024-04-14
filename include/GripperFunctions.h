//
// Created by lucas on 21.03.23.
//

#ifndef GOAL_STATE_PUBLISHER_GRIPPERFUNCTIONS_H
#define GOAL_STATE_PUBLISHER_GRIPPERFUNCTIONS_H

#include <ros/ros.h>
// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

moveit_msgs::CollisionObject createBox (std::string id, std::string frame_id, float width, float length, float height);

void setCollisionObjectPose (moveit_msgs::CollisionObject& collision_object, std::vector<float>position, std::vector<float>orientation = {0.0, 0.0, 0.0});

void openGripper(trajectory_msgs::JointTrajectory& posture);

void closedGripper(trajectory_msgs::JointTrajectory& posture);

void pick(moveit::planning_interface::MoveGroupInterface& move_group);

void place(moveit::planning_interface::MoveGroupInterface& group);

std::vector<std::string> addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface);
#endif //GOAL_STATE_PUBLISHER_GRIPPERFUNCTIONS_H
