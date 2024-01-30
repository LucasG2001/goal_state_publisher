//
// Created by lucas on 30.11.23.
//

#include <ros/node_handle.h>
#include <ros/spinner.h>
#include <goal_state_publisher/ImpedanceParameterController.h>
#include <iostream>
#include <TaskPlanner.h>
#include <custom_msgs/ImpedanceParameterMsg.h>



int main(int argc, char** argv) {
	//ROS Specific Initializations
	std::cout << "starting node ";
	ros::init(argc, argv, "impedance_controller_node");
	ros::NodeHandle nh;
	std::cout << "started node";

	ros::AsyncSpinner spinner(4);
	spinner.start();

	//publishers
	ros::Publisher equilibrium_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/cartesian_impedance_controller/reference_pose", 1);
	ros::Publisher impedance_parameter_pub = nh.advertise<custom_msgs::ImpedanceParameterMsg>("/cartesian_impedance_controller/impedance_param_reconfig", 1);
	//ImpedanceParamController Object
	ImpedanceParameterController controller(&equilibrium_pose_pub, &impedance_parameter_pub);
	//subscribers
	ros::Subscriber rightHandSub = nh.subscribe("cartesian_impedance_controller/right_hand", 1, &ImpedanceParameterController::rightHandCallback, &controller);
	ros::Subscriber leftHandSub = nh.subscribe("cartesian_impedance_controller/left_hand", 1, &ImpedanceParameterController::leftHandCallback, &controller);
	ros::Subscriber FextSub = nh.subscribe("/F_ext", 1, &ImpedanceParameterController::FextCallback, &controller);
	ros::Subscriber task_sub = nh.subscribe("/action_primitive", 1, &ImpedanceParameterController::TaskCallback, &controller);
	ros::Subscriber ee_pose = nh.subscribe("/franka_state_controller/franka_states", 10, &TaskPlanner::ee_callback, &controller.task_planner);




	ros::Rate loop_rate(1); // 1
	//int task_type;

	while (ros::ok()) {
		//wait for callbacks
		loop_rate.sleep();
	}
	spinner.stop();

	return 0;
}

