//
// Created by lucas on 01.04.23.
//

#ifndef GOAL_STATE_PUBLISHER_CALLBACK_CLASSES_H
#define GOAL_STATE_PUBLISHER_CALLBACK_CLASSES_H

#include <actionlib/client/simple_action_client.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <franka_gripper/franka_gripper.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <goal_state_publisher/GraspMsg.h>
#include <goal_state_publisher/MoveGripperMsg.h>
#include <goal_state_publisher/StopGripperMsg.h>


void chatterCallback(const geometry_msgs::Pose::ConstPtr& msg);

class PoseListener
{
public:
    moveit::planning_interface::MoveGroupInterface *move_group_ptr;
    PoseListener(moveit::planning_interface::MoveGroupInterface *move_group);
    void poseCallback(const geometry_msgs::Pose::ConstPtr& msg);
};

class GripperListener
{
public:
    actionlib::SimpleActionClient<franka_gripper::GraspAction> *grasp_pointer;
    actionlib::SimpleActionClient<franka_gripper::MoveAction> *move_pointer;
    actionlib::SimpleActionClient<franka_gripper::StopAction> *stop_pointer;
    GripperListener(actionlib::SimpleActionClient<franka_gripper::GraspAction> *grasp_client, actionlib::SimpleActionClient<franka_gripper::MoveAction> *move_client,
                    actionlib::SimpleActionClient<franka_gripper::StopAction> *stop_client);

    //grasp with desired force width and epsilon
    void graspCallback(const goal_state_publisher::GraspMsg::ConstPtr& msg);
    //open gripper with desired width and speed
    void openCallback(const goal_state_publisher::MoveGripperMsg::ConstPtr& msg);
    void stopCallback(const goal_state_publisher::StopGripperMsg::ConstPtr& msg);
};
#endif //GOAL_STATE_PUBLISHER_CALLBACK_CLASSES_H
