//
// Created by lucas on 12.12.23.
//
#include <ros/node_handle.h>
#include <ros/spinner.h>
#include <iostream>
#include <custom_msgs/action_primitive_message.h>
#include <custom_msgs/PlacePose.h>
#include "Utility.h"

int main(int argc, char** argv) {
	//ROS Specific Initializations
	std::cout << "starting node ";
	ros::init(argc, argv, "action_primitive_test_node");
	ros::NodeHandle nh;
	std::cout << "started node";

	ros::Publisher task_publisher = nh.advertise<custom_msgs::action_primitive_message>("/action_primitive", 10);
	//test follow me mode
	ros::Publisher hand_publisher = nh.advertise<geometry_msgs::Pose>("cartesian_impedance_controller/right_hand", 10);
	//place pose publisher for testing purposes
	ros::Publisher place_pose_pub = nh.advertise<custom_msgs::PlacePose>("/place_pose", 10);
	geometry_msgs::Pose hand_pose;

	ros::AsyncSpinner spinner(4);
	spinner.start();

	ros::Rate loop_rate(50); // 20Hz

	int task_type;
	int constraint = false;
	custom_msgs::action_primitive_message action_message;
	custom_msgs::PlacePose place_pose_msg;
	std::vector<double> direction(3); // this can be either the place position a line constraint (direction) or a plane constraint (normal)
	double norm = 0.0; // norm of direction vector
	action_message.goal_pose = createGoalPose({0.5, 0.0, 0.5}, {3.14156, 0.0, 0.0});
	action_message.object_pose = createGoalPose({0.3, 0.3, 0.15}, {3.14156, 0.0, 0.0});
	action_message.grasp = false;
	action_message.task_type = 1;
	action_message.isFreeFloat = false;
	bool free_float = 0;

	while (ros::ok()) {
		std::cout
				<< "Enter a task. 1 (GetMe), 2 (FollowMe), 3 (HoldThis), 4 (TakeThis), or 5 (AvoidMe), or 6 (sinusoid trajectory 3D): ";
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
				hand_pose.position.x = action_message.goal_pose.position.x;
				hand_pose.position.y = action_message.goal_pose.position.y;
				hand_pose.position.z = action_message.goal_pose.position.z;
				hand_pose.orientation.x = 0.827;
				hand_pose.orientation.y = -0.298;
				hand_pose.orientation.z = -0.18;
				hand_pose.orientation.w = -0.3399;
				action_message.grasp = false;
				task_publisher.publish(action_message);
				for (int i = 0; i < 10; i++) {
					hand_pose.position.y = action_message.goal_pose.position.y - 0.09 * i;
					hand_publisher.publish(hand_pose);
					ros::Duration(0.5).sleep();
				}
				break;
			case 3:
				std::cout << "HoldThis selected\n";
				std::cout << "Enter 0 for normal or 1 for free float";
				std::cin >> free_float;
				action_message.goal_pose = createGoalPose({0.5, 0.0, 0.5}, {3.14156, 0.0, 0.0});
				action_message.object_pose = createGoalPose({0.3, 0.3, 0.15}, {3.14156, 0.0, 0.0});
				action_message.grasp = true;
				action_message.isFreeFloat = free_float;
				task_publisher.publish(action_message);
				// wait, then send place pose message to test free float with plane constraints
				ros::Duration(2.0).sleep();
				std::cout << "Send Place Pose. Type in your place position. If this should be a constraint you can select later which it should be" << std::endl;
				std::cin >> direction[0] >> direction[1] >> direction[2];
				norm = std::sqrt((direction[0] * direction[0] + direction[1] * direction[1] + direction[2] * direction[2]));
				if(free_float){
					if(norm > 1.0){
					// norm the direction of it is bigger than unit vector, but only in free float mode since it is interpreted as position else
					direction[0] = direction[0] / norm;
					direction[1] = direction[1] / norm;
					direction[2] = direction[2] / norm;
				}
					//create a place pose where the position is interpreted as Plane constraint (normal vector)
					place_pose_msg.goalPose = createGoalPose(direction, {3.14156, 0.0, 0.0}); // keep rotation in any case
					std::cout << "Choose line constraint (0) or plane constraint (1)" << std::endl;
					std::cin >> constraint;
					if(constraint == 0){ //line constraint
						place_pose_msg.isLineConstraint.data = true;
						place_pose_msg.isPlaneConstraint.data = false;
					}
					else{ //plane constraint
						place_pose_msg.isPlaneConstraint.data = true;
						place_pose_msg.isLineConstraint.data = false;
					}
				}
				else{
					//create a place pose without constraints
					place_pose_msg.goalPose = createGoalPose(direction, {3.14156, 0.0, 0.0});
					place_pose_msg.isLineConstraint.data = false;
					place_pose_msg.isPlaneConstraint.data = false;
				}
				//publish place pose
				place_pose_pub.publish(place_pose_msg);
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
				action_message.goal_pose = createGoalPose({0.5, -0.2, 0.3}, {-1.3125, -0.7875, -1.1});
				action_message.object_pose = createGoalPose({0.3, 0.3, 0.15}, {3.14156, 0.0, 0.0});
				action_message.grasp = false;
				task_publisher.publish(action_message);
				break;

			default:
				std::cout << "Invalid task\n";
				break;
		}

		if (task_type == 6) {
			ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>(
					"/cartesian_impedance_controller/reference_pose", 10);

			ros::Rate inner_rate(100); // 100 Hz

			double startTime = ros::Time::now().toSec();
			double frequency = 2 * M_PI / 12.0; // Full cycle in 15 seconds

			while (ros::ok()) {
				double currentTime = ros::Time::now().toSec() - startTime;

				geometry_msgs::PoseStamped pose_msg;
				pose_msg.header.stamp = ros::Time::now();
				pose_msg.header.frame_id = "base_link";

				// Define the sinusoidal trajectory
				pose_msg.pose.position.x = 0.5 + 0.1 * sin(5 * frequency * currentTime);
				pose_msg.pose.position.y = 0.0 + 0.4 * sin(frequency * currentTime);
				pose_msg.pose.position.z = 0.3 + 0.1 * sin(5* frequency * currentTime);

				// Orientation (quaternion) - Keeping it constant for simplicity
				pose_msg.pose.orientation.x = 1.0;
				pose_msg.pose.orientation.y = 0.0;
				pose_msg.pose.orientation.z = 0.0;
				pose_msg.pose.orientation.w = 0.0;

				pose_pub.publish(pose_msg);
				inner_rate.sleep();
			}

		}

	}
	spinner.stop();
	return 0;
}
