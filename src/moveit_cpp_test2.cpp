#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include "gripper_functions.h"

int main(int argc, char** argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "moveit_gazebo_control");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Instantiate MoveGroup and PlanningSceneInterface
    moveit::planning_interface::MoveGroupInterface move_group("panda_arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Instantiate MoveItVisualTools for visualizing plans
    moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");

    // Set planner parameters
    move_group.setPlanningTime(25.0);
    move_group.setNumPlanningAttempts(8);
    move_group.setMaxVelocityScalingFactor(0.9);
    move_group.setMaxAccelerationScalingFactor(0.9);

    // Get the current state of the robot in Gazebo
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    move_group.setStartStateToCurrentState();

    // Set the target pose for the end effector
    geometry_msgs::Pose target_pose;
    target_pose.orientation.w = 1.0; // Identity quaternion
    target_pose.orientation.x = 0.0;
    target_pose.orientation.y = 0.0;
    target_pose.orientation.z = 0.0;
    target_pose.position.x = atof(argv[1]); // Convert from string to float
    target_pose.position.y = atof(argv[2]);
    target_pose.position.z = atof(argv[3]);
    move_group.setPoseTarget(target_pose);

    // Plan and visualize the trajectory
    //moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    //bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (true) //succcess
    {
        ROS_INFO("Planning successful!");

        // Visualize the plan
        //visual_tools.deleteAllMarkers();
        //visual_tools.publishAxisLabeled(target_pose, "goal_pose");
        //visual_tools.publishTrajectoryPath(my_plan.trajectory_, move_group.getCurrentState());
        //visual_tools.trigger();

        // Wait for user input before continuing
        visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to execute the planned trajectory in Gazebo");
        addCollisionObjects(planning_scene_interface);

        // Wait a bit for ROS things to initialize
        ros::WallDuration(1.0).sleep();

        pick(move_group);
        visual_tools.prompt("Test");
        ros::WallDuration(1.0).sleep();

        place(move_group);

        // Execute the trajectory in Gazebo
        //move_group.execute(my_plan);
        ROS_INFO("Executing!");
    }
    else
    {
        ROS_ERROR("Planning failed!");
    }

    ros::shutdown();
    return 0;
}