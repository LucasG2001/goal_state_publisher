//
// Created by lucas on 01.04.23.
//
//callback function upon receiving message
#include "callback_classes.h"

void chatterCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    // Convert orientation and position values to string
    std::string orientation_str = "x=" + std::to_string(msg->orientation.x) +
                                  ", y=" + std::to_string(msg->orientation.y) +
                                  ", z=" + std::to_string(msg->orientation.z) +
                                  ", w=" + std::to_string(msg->orientation.w);
    std::string position_str = "x=" + std::to_string(msg->position.x) +
                               ", y=" + std::to_string(msg->position.y) +
                               ", z=" + std::to_string(msg->position.z);

    // Print the strings to the console
    ROS_INFO("Pose orientation: %s", orientation_str.c_str());
    ROS_INFO("Pose position: %s", position_str.c_str());
}


PoseListener::PoseListener(moveit::planning_interface::MoveGroupInterface *move_group){
        move_group_ptr = move_group;
    }


void PoseListener::poseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    chatterCallback(msg);
    ROS_INFO("got new reference pose");
    geometry_msgs::Pose target_pose;
    target_pose.orientation = msg->orientation;
    target_pose.position = msg->position;
    this->move_group_ptr->setPoseTarget(target_pose);
    this->move_group_ptr->setStartStateToCurrentState();
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (this->move_group_ptr->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Plan was computed");
    if (success){
        this->move_group_ptr->execute(plan);
        ROS_INFO("Plan found");
        ROS_INFO("Executing!");
    }
    else { ROS_WARN("Could not Plan Motion successfully");}

}


GripperListener::GripperListener(actionlib::SimpleActionClient<franka_gripper::GraspAction> *grasp_client, actionlib::SimpleActionClient<franka_gripper::MoveAction> *move_client, actionlib::SimpleActionClient<franka_gripper::StopAction> *stop_client) {
        grasp_pointer = grasp_client;
        move_pointer = move_client;
        stop_pointer = stop_client;
    }
    void GripperListener::graspCallback(const goal_state_publisher::GraspMsg::ConstPtr& msg)
    {
        franka_gripper::GraspGoal grasp_goal;
        grasp_goal.width = msg->width;
        grasp_goal.speed = msg->speed;
        grasp_goal.force = msg->force;
        grasp_goal.epsilon.inner = msg->tolerance;
        grasp_goal.epsilon.outer = msg->tolerance;
        this->grasp_pointer->sendGoal(grasp_goal);
    }
//open gripper with desired width and speed
void GripperListener::openCallback(const goal_state_publisher::MoveGripperMsg::ConstPtr& msg)
    {
        franka_gripper::MoveGoal move_goal;
        move_goal.speed = msg->speed;
        move_goal.width = msg->width;
        this->move_pointer->sendGoal(move_goal);
    }
    void GripperListener::stopCallback(const goal_state_publisher::StopGripperMsg::ConstPtr& msg)
    {
        franka_gripper::StopGoal stop_goal;
        this->stop_pointer->sendGoal(stop_goal);

    }

