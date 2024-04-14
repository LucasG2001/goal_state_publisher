//
// Created by lucas on 28.03.23.
//
#include "ros/ros.h"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <actionlib/client/simple_action_client.h>
#include "CallbackClasses.h"
#include "Utility.h"
#include "GripperFunctions.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(4);
    spinner.start();
    // Create a client object for the Franka gripper grasp action
    actionlib::SimpleActionClient<franka_gripper::GraspAction> grasp_client("franka_gripper/grasp", true);
    // Create a client object for the Franka gripper move action
    actionlib::SimpleActionClient<franka_gripper::MoveAction> move_client("franka_gripper/move", true);
    // Create a client object for the Franka gripper stop action
    actionlib::SimpleActionClient<franka_gripper::StopAction> stop_client("franka_gripper/stop", true);
    // Wait for the action servers to start up
    grasp_client.waitForServer();
    move_client.waitForServer();
    stop_client.waitForServer();

    moveit::planning_interface::MoveGroupInterface move_group("panda_arm"); //or panda_arm
    moveit::planning_interface::MoveGroupInterface gripper_group("panda_hand");

    // Instantiate MoveItVisualTools for visualizing plans
    moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
    ROS_INFO("adding objects to the scene");
    //std::vector<std::string> object_names = addCollisionObjects(planning_scene_interface);
    ROS_INFO("Initialized Moveit and Clients");
    ROS_INFO("Listener Node Created");

    // Set the planner parameters
    setPlanningParameters(move_group, 10.0, 10, 0.1, 0.1);
    PoseListener pose_listener(&move_group);
    GripperListener gripper_listener(&grasp_client, &move_client, &stop_client);
    ros::Subscriber pose_sub = n.subscribe("panda_pose_reference", 1, &PoseListener::poseCallback, &pose_listener);
    ros::Subscriber grasp_sub = n.subscribe("grasp_goal", 1, &GripperListener::graspCallback, &gripper_listener);
    ros::Subscriber move_sub = n.subscribe("gripper_move_goal", 1, &GripperListener::openCallback, &gripper_listener);
    ros::Subscriber stop_sub = n.subscribe("gripper_stop_goal", 1, &GripperListener::stopCallback, &gripper_listener);

    ros::waitForShutdown();

    return 0;
}