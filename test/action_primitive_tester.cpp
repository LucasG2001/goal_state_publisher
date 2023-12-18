//
// Created by lucas on 12.12.23.
//
#include <ros/node_handle.h>
#include <ros/spinner.h>
#include <iostream>
#include <custom_msgs/action_primitive_message.h>
#include "utility.h"

int main(int argc, char** argv) {
	//ROS Specific Initializations
	std::cout << "starting node ";
	ros::init(argc, argv, "action_primitive_test_node");
	ros::NodeHandle nh;
	std::cout << "started node";

	ros::Publisher equilibrium_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/cartesian_impedance_controller/reference_pose", 10);
	ros::Publisher task_publisher = nh.advertise<custom_msgs::action_primitive_message>("/action_primitive", 10);
	//test follow me mode
	ros::Publisher hand_publisher = nh.advertise<geometry_msgs::Pose>("/right_hand", 10);
	geometry_msgs::Pose hand_pose;

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
		std::cout << "Enter a task. 1 (GetMe), 2 (FollowMe), 3 (HoldThis), 4 (TakeThis), or 5 (AvoidMe): ";
		std::cin >> task_type;
		action_message.task_type = task_type;
		// Switch-case statement
		switch (task_type) {
			case 1:
				std::cout << "GetMe selected\n";
				action_message.goal_pose = createGoalPose({0.7, 0.0, 0.35}, {3.14156, 0.0, 0.0});
				action_message.object_pose = createGoalPose({0.3, 0.3, 0.15}, {3.14156, 0.0, 0.0});
				action_message.grasp = true;

				task_publisher.publish(action_message);
				break;
			case 2:
				std::cout << "FollowMe selected\n";
				action_message.goal_pose = createGoalPose({0.3, 0.5, 0.5}, {3.14156, 0.0, 0.0});
				action_message.object_pose = createGoalPose({0.3, 0.3, 0.15}, {3.14156, 0.0, 0.0});
				hand_pose = action_message.goal_pose;
				action_message.grasp = false;
				task_publisher.publish(action_message);
				for (int i = 0; i < 10; i++){
					hand_pose.position.y = action_message.goal_pose.position.y - 0.1 * i;
					hand_publisher.publish(hand_pose);
					ros::Duration(0.5).sleep();
				}
				break;
			case 3:
				std::cout << "HoldThis selected\n";
				action_message.goal_pose = createGoalPose({0.5, 0.0, 0.5}, {3.14156, 0.0, 0.0});
				action_message.object_pose = createGoalPose({0.3, 0.3, 0.15}, {3.14156, 0.0, 0.0});
				action_message.grasp = true;
				task_publisher.publish(action_message);
				break;
			case 4:
				std::cout << "TakeThis selected\n";
				action_message.goal_pose = createGoalPose({0.65, -0.1, 0.4}, {3.14156, 0.0, 0.0});
				action_message.object_pose = createGoalPose({0.3, 0.3, 0.45}, {3.14156, 0.0, 0.0});
				action_message.grasp = true;
				task_publisher.publish(action_message);
				break;
			case 5:
				std::cout << "AvoidMe selected\n";
				action_message.goal_pose = createGoalPose({0.5, -0.3, 0.3}, {-1.5, -0.7875, -1.5});
				action_message.object_pose = createGoalPose({0.3, 0.3, 0.15}, {3.14156, 0.0, 0.0});
				action_message.grasp = false;
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
