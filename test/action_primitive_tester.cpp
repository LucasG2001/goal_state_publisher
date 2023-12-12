//
// Created by lucas on 12.12.23.
//
#include <ros/node_handle.h>
#include <ros/spinner.h>
#include <iostream>
#include <TaskPlanner.h>
#include <custom_msgs/action_primitive_message.h>
#include "utility.h"

int main(int argc, char** argv) {
	//ROS Specific Initializations
	std::cout << "starting node ";
	ros::init(argc, argv, "impedance_controller_node");
	ros::NodeHandle nh;
	std::cout << "started node";

	ros::Publisher equilibrium_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/cartesian_impedance_controller/reference_pose", 10);
	ros::Publisher task_publisher = nh.advertise<custom_msgs::action_primitive_message>("/action_primitive", 10);

	ros::AsyncSpinner spinner(4);
	spinner.start();

	ros::Rate loop_rate(20); // 20Hz

	int task_type;
	custom_msgs::action_primitive_message action_message;
	action_message.goal_pose = createGoalPose({0.5, 0.0, 0.5}, {3.14156, 0.0, 0.0});
	action_message.object_pose = createGoalPose({0.3, 0.3, 0.15}, {3.14156, 0.0, 0.0});
	action_message.grasp = false;
	action_message.task_type = 1;

	while (ros::ok()) {
		std::cout << "Enter a task (1, 2, 3, 4, or 5): ";
		std::cin >> task_type;
		// Switch-case statement
		switch (task_type) {
			case 1:
				std::cout << "GetMe selected\n";
				action_message.goal_pose = createGoalPose({0.7, 0.0, 0.35}, {3.14156, 0.0, 0.0});
				action_message.object_pose = createGoalPose({0.3, 0.3, 0.15}, {3.14156, 0.0, 0.0});
				action_message.grasp = true;
				action_message.task_type = task_type;
				task_publisher.publish(action_message);
				break;
			case 2:
				std::cout << "FollowMe selected\n";
				action_message.goal_pose = createGoalPose({0.5, 0.0, 0.5}, {3.14156, 0.0, 0.0});
				action_message.object_pose = createGoalPose({0.3, 0.3, 0.15}, {3.14156, 0.0, 0.0});
				action_message.grasp = false;
				action_message.task_type = task_type;
				task_publisher.publish(action_message);
				break;
			case 3:
				std::cout << "HoldThis selected\n";
				action_message.goal_pose = createGoalPose({0.5, 0.0, 0.5}, {3.14156, 0.0, 0.0});
				action_message.object_pose = createGoalPose({0.3, 0.3, 0.15}, {3.14156, 0.0, 0.0});
				action_message.grasp = true;
				action_message.task_type = task_type;
				task_publisher.publish(action_message);
				break;
			case 4:
				std::cout << "TakeThis selected\n";
				action_message.goal_pose = createGoalPose({0.5, 0.0, 0.5}, {3.14156, 0.0, 0.0});
				action_message.object_pose = createGoalPose({0.3, 0.3, 0.15}, {3.14156, 0.0, 0.0});
				action_message.grasp = true;
				action_message.task_type = task_type;
				task_publisher.publish(action_message);
				break;
			case 5:
				std::cout << "AvoidMe selected\n";
				action_message.goal_pose = createGoalPose({0.5, -0.3, 0.3}, {3.14156, -0.7875, 0.0});
				action_message.object_pose = createGoalPose({0.3, 0.3, 0.15}, {3.14156, 0.0, 0.0});
				action_message.grasp = false;
				action_message.task_type = task_type;
				task_publisher.publish(action_message);
				break;
			default:
				std::cout << "Invalid task\n";
				break;
		}

		loop_rate.sleep();
	}
	spinner.stop();

	return 0;
}
