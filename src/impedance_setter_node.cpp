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

	//publishers
	ros::Publisher equilibrium_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/cartesian_impedance_controller/reference_pose", 10);
	//ImpedanceParamController Object
	ImpedanceParameterController controller(&equilibrium_pose_pub);
	//subscribers
	ros::Subscriber rightHandSub = nh.subscribe("/right_hand", 1, &ImpedanceParameterController::rightHandCallback, &controller);
	ros::Subscriber leftHandSub = nh.subscribe("/left_hand", 1, &ImpedanceParameterController::leftHandCallback, &controller);
	ros::Subscriber FextSub = nh.subscribe("/F_ext", 1, &ImpedanceParameterController::FextCallback, &controller);
	ros::Subscriber task_sub = nh.subscribe("/action_primitive", 1, &ImpedanceParameterController::TaskCallback, &controller);
	ros::Subscriber ee_pose = nh.subscribe("/franka_state_controller/franka_states", 10, &TaskPlanner::ee_callback, &controller.task_planner);


	ros::AsyncSpinner spinner(4);
	spinner.start();

	ros::Rate loop_rate(1); // 1
	//int task_type;

	while (ros::ok()) {
		//wait for callbacks
		loop_rate.sleep();
	}
	spinner.stop();

	return 0;
}

