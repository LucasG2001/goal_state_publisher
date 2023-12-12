//
// Created by lucas on 30.11.23.
//

#include <ros/node_handle.h>
#include <ros/spinner.h>
#include <goal_state_publisher/ImpedanceParameterController.h>
#include <iostream>
#include <TaskPlanner.h>



int main(int argc, char** argv) {
	//ROS Specific Initializations
	std::cout << "starting node ";
	ros::init(argc, argv, "impedance_controller_node");
	ros::NodeHandle nh;
	std::cout << "started node";

	ImpedanceParameterController controller;
	TaskPlanner task_planner;
	//subscribers
	ros::Subscriber rightHandSub = nh.subscribe("/right_hand", 1, &ImpedanceParameterController::rightHandCallback, &controller);
	ros::Subscriber leftHandSub = nh.subscribe("/left_hand", 1, &ImpedanceParameterController::leftHandCallback, &controller);
	ros::Subscriber FextSub = nh.subscribe("/F_ext", 1, &ImpedanceParameterController::FextCallback, &controller);
	ros::Subscriber task_sub = nh.subscribe("/action_primitive", 1, &ImpedanceParameterController::TaskCallback, &controller);
	//publishers
	ros::Publisher equilibrium_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/cartesian_impedance_controller/reference_pose", 10);

	ros::AsyncSpinner spinner(4);
	spinner.start();

	ros::Rate loop_rate(20); // 20Hz
	int task_type;

	while (ros::ok()) {
		// Get input from user
		/**
		std::cout << "Enter a task (1, 2, 3, 4, or 5): ";
		std::cin >> task_type;
		// Switch-case statement
		switch (task_type) {
			case 1:
				std::cout << "GetMe selected\n";
				controller.activeTask = &controller.get_me_task;
				break;
			case 2:
				std::cout << "FollowMe selected\n";
				controller.activeTask = &controller.follow_me_task;
				break;
			case 3:
				std::cout << "HoldThis selected\n";
				controller.activeTask = &controller.hold_this_task;
				break;
			case 4:
				std::cout << "TakeThis selected\n";
				controller.activeTask = &controller.take_this_task;
				break;
			case 5:
				std::cout << "AvoidMe selected\n";
				controller.activeTask = &controller.avoid_me_task;
				break;
			default:
				std::cout << "Invalid task\n";
				break;
		}
		**/

		controller.activeTask->performAction(task_planner, equilibrium_pose_pub);
		loop_rate.sleep();
	}
	spinner.stop();

	return 0;
}

