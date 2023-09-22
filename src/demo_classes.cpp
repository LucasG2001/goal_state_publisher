//
// Created by lucas on 21.09.23.
//
#include "ros/ros.h"
#include <vector>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include "demo_classes.h"
#include "utility.h"


// Default constructor
TaskPlanner::TaskPlanner( moveit::planning_interface::MoveGroupInterface* move_group,  moveit::planning_interface::MoveGroupInterface* gripper_group) :
        gripper_grasp_client("franka_gripper/grasp", true),
        gripper_move_client("franka_gripper/move", true),
        gripper_stop_client("franka_gripper/stop", true)
    {
    // Initialize ROS node handle
    nh_.reset(new ros::NodeHandle("~"));
    // Set default callback queue sizes
    move_pub = nh_->advertise<geometry_msgs::Pose>("/panda_pose_reference", 1);
    move_group_ptr = move_group;
    gripper_group_ptr = gripper_group;

    // Wait for the action servers to start up
    gripper_grasp_client.waitForServer();
    gripper_move_client.waitForServer();
    gripper_stop_client.waitForServer();
} // end constructor

void TaskPlanner::move(std::vector<double> position, std::vector<double> orientation){
        geometry_msgs::Pose target_pose;
        target_pose = createGoalPose(position, orientation);
        move_group_ptr->setPoseTarget(target_pose);
        move_group_ptr->setStartStateToCurrentState();
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (this->move_group_ptr->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO("Plan was computed");
        if (success){
            this->move_group_ptr->execute(plan); //this call should be blocking
            ROS_INFO("Plan found");
            ROS_INFO("Executing!");
        }
        else { ROS_WARN("Could not Plan Motion successfully");}
    }

void TaskPlanner::open_gripper(double speed, double width){
    franka_gripper::MoveGoal gripper_open;
    gripper_open.width = width;
    gripper_open.speed = speed;
    gripper_move_client.sendGoal(gripper_open);
    ros::Duration(0.01).sleep();
    bool finished_before_timeout = gripper_move_client.waitForResult(ros::Duration(10.0)); // Adjust the timeout as needed
    if (finished_before_timeout) {
        ROS_INFO("Action finished successfully!");
        // Handle successful completion here
    } else {
        ROS_WARN("Action did not finish before the timeout.");
        // Handle timeout or other error conditions here
    }
}

void TaskPlanner::grasp_object(double speed, double width, double force, double tol){
    franka_gripper::GraspGoal grasp;
    grasp.speed = speed;
    grasp.width = width;
    grasp.force = force;
    grasp.epsilon.inner = tol;
    grasp.epsilon.outer = tol;
    gripper_grasp_client.sendGoal(grasp);
    // Wait for the action to finish
    bool finished_before_timeout = gripper_grasp_client.waitForResult(ros::Duration(10.0)); // Adjust the timeout as needed

    if (finished_before_timeout) {
        ROS_INFO("Action finished successfully!");
        // Handle successful completion here
    } else {
        ROS_WARN("Action did not finish before the timeout.");
        // Handle timeout or other error conditions here
    }
}

void task1(TaskPlanner &task_planner){
    std::vector<double> object_location = {0.5, 0.0, 0.2};
    std::vector<double> place_location = {0.3, 0.2, 0.2};
    std::vector<double> neutral_orientation = {-3.14156, 0.0, -0.785};
    for (size_t i = 0; i < 5; i++){
        task_planner.move(object_location, neutral_orientation);
        task_planner.grasp_object();
        task_planner.move(place_location, neutral_orientation);
        task_planner.open_gripper();
        object_location[1] -= 0.03;
        object_location[2] -= 0.1;
        place_location[1] += 0.05;
        place_location[2] += 0.05;
    }
}


int main(int argc, char **argv) {
    //ToDo: Use argv to set robot (panda/fr3)
    ros::init(argc, argv, "demo_planner");
    ros::NodeHandle n;

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::Rate loop_rate(0.1);
    ros::Rate update_rate(20);

    //create move group_connection
    moveit::planning_interface::MoveGroupInterface move_group("panda_arm"); //or panda_arm
    moveit::planning_interface::MoveGroupInterface gripper_group("panda_hand");


    int grip_action;    //input for flow control
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> force_client(
            "cartesian_impedance_controller/follow_joint_trajectory", true);

    TaskPlanner task_planner(&move_group, &gripper_group);
    while (ros::ok()) {

        // Prompt user to input three values
        std::cout
                << "Enter 0 (move, nothing else) | 1 (open gripper) | 2 (grasp) | 3 (stop) | 4 (try task 1)| ";
        std::cin >> grip_action;
        franka_gripper::StopAction stop;
        franka_gripper::StopGoalConstPtr stop_goal;
        switch (grip_action) {
            case 0:
                task_planner.move({0.4, 0.05, 0.3}, {-3.14156, 0, -0.785});
                break;
            case 1:
                task_planner.open_gripper(0.08, 0.2);
                break;
            case 2:
                task_planner.grasp_object(0.2, 0.02, 20, 0.005);
                break;
            case 3:
                task_planner.stop_pub.publish(stop_goal);
                break;
            case 4:
                task1(task_planner);
                break;

        } //switch case
        ros::spinOnce();
        loop_rate.sleep();
    } //while ros node
    return 0;
} //main