//
// Created by lucas on 21.09.23.
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
#include <franka_msgs/SetLoad.h>

//watch out with these h-files. Only building with catkin build in the workspace will build everything correctly, even when not including these files
//these are for the IDE. If you can't find them in the referenced location, the cMake dependencies should make sure catkin can build and auto-generate the headers
#include "../../../devel/.private/goal_state_publisher/include/goal_state_publisher/GraspMsg.h"
#include "../../../devel/.private/goal_state_publisher/include/goal_state_publisher/MoveGripperMsg.h"
#include "../../../devel/.private/goal_state_publisher/include/goal_state_publisher/StopGripperMsg.h"
#include <cmath>
#include <visualization_msgs/InteractiveMarkerFeedback.h>

class TaskPlanner {
public:
    ros::Publisher move_pub;
    ros::Publisher gripper_move_pub;
    ros::Publisher grasp_pub;
    ros::Publisher stop_pub;
    // Default constructor
    TaskPlanner( moveit::planning_interface::MoveGroupInterface* move_group,  moveit::planning_interface::MoveGroupInterface* gripper_group) {
        // Initialize ROS node handle
        nh_.reset(new ros::NodeHandle("~"));

        // Set default callback queue sizes
        move_pub = nh_->advertise<geometry_msgs::Pose>("panda_pose_reference", 1);
        grasp_pub = nh_->advertise<goal_state_publisher::GraspMsg>("grasp_goal", 1);
        gripper_move_pub = nh_->advertise<goal_state_publisher::MoveGripperMsg>("gripper_move_goal", 1);
        stop_pub = nh_->advertise<goal_state_publisher::StopGripperMsg>("gripper_stop_goal", 1);

        move_group_ptr = move_group;
        gripper_group_ptr = gripper_group;
    } // end constructor

private:
    ros::NodeHandlePtr nh_;
    moveit::planning_interface::MoveGroupInterface* move_group_ptr; //or panda_arm
    moveit::planning_interface::MoveGroupInterface* gripper_group_ptr;
};

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
                << "Enter 0 (move, nothing else) | 1 (open gripper) | 2 (grasp) | 3 (stop) | ";
        std::cin >> grip_action;

        switch (grip_action) {
            case 0:
                move(task_planner.move_pub, {0.4, 0.05, 0.3}, {-3.14156, 0, -0.785});
                break;
            case 1:
                open_gripper(task_planner.gripper_move_pub, 0.08, 0.2);
                break;
            case 2:
                grasp_object(task_planner.grasp_pub, 0.2, 0.02, 20, 0.005);
                break;
            case 3:
                franka_gripper::StopAction stop;
                franka_gripper::StopGoalConstPtr stop_goal;
                task_planner.stop_pub.publish(stop_goal);
                break;

        } //switch case
        ros::spinOnce();
        loop_rate.sleep();
    } //while ros node
    return 0;
} //main