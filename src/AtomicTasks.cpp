//
// Created by lucas on 01.12.23.
//

#include <iostream>
#include <goal_state_publisher/AtomicTasks.h>
#include <TaskPlanner.h>
#include <ros/ros.h>

// Default constructor for GetMe
GetMe::GetMe() : ActionPrimitive() {
	// Custom values for GetMe
	Eigen::Matrix<double, 6, 6> stiffness;  
	stiffness =  Eigen::MatrixXd::Identity(6,6);
	stiffness.topLeftCorner(3, 3) << 300, 0, 0, 0, 300, 0, 0, 0, 300;
	stiffness.bottomRightCorner(3, 3) << 50, 0, 0, 0, 50, 0, 0, 0, 12;

	Eigen::Matrix<double, 6, 6> damping;           
	damping =  Eigen::MatrixXd::Identity(6,6);
	damping.topLeftCorner(3, 3) << 40, 0, 0, 0, 40, 0, 0, 0, 40;
	damping.bottomRightCorner(3, 3) << 15, 0, 0, 0, 15, 0, 0, 0, 3;
	//ToDo: Inerta matrix should be specified as multiple of physical inertia

	Eigen::Matrix<double, 6, 6> inertia;           
	inertia =  Eigen::MatrixXd::Identity(6,6);
	inertia.topLeftCorner(3, 3) << 200, 0, 0, 0, 200, 0, 0, 0, 200;
	inertia.bottomRightCorner(3, 3) << 50, 0, 0, 0, 50, 0, 0, 0, 50;

	Eigen::Matrix<double, 6, 6> bubble_stiffness;  
	bubble_stiffness.topLeftCorner(3, 3) << 150, 0, 0, 0, 150, 0, 0, 0, 150;
	bubble_stiffness.bottomRightCorner(3, 3) << 50, 0, 0, 0, 50, 0, 0, 0, 50;
	Eigen::Matrix<double, 6, 6> bubble_damping;   
	bubble_damping.topLeftCorner(3, 3) << 25, 0, 0, 0, 25, 0, 0, 0, 25;
	bubble_damping.bottomRightCorner(3, 3) << 5, 0, 0, 0, 5, 0, 0, 0, 1;
	setParameters(stiffness, damping, inertia,
	              bubble_stiffness, bubble_damping);

	// Additional custom initialization for GetMe if needed
}

void GetMe::performAction(TaskPlanner& task_planner, ros::Publisher &publisher, ros::Publisher &impedance_publisher) {
	ROS_INFO("starting get me action");
	construct_impedance_message(this->impedance_params);
	impedance_publisher.publish(this->compliance_update);
	// Implementation of performAction for GetMe
	// Custom logic for GetMe
	ROS_INFO("starting gripper open");
	task_planner.open_gripper();
	//go to object here we can have high impedances and low repulsion as we should tipically go away from the human (default)
	ROS_INFO("going towards object ");
	task_planner.execute_action(this->getObjectPose().head(3), this->getObjectPose().tail(3), &publisher, 0.03);
	ROS_INFO("grasping ");
	task_planner.grasp_object();
	//ToDo: goal_pose should be updated as hand pose -> in callbacks?
	//go back to hand
	//now construct more careful impedances to be safe in handover
	ImpedanceMatrices safety_impedance;
	safety_impedance.spring_stiffness = impedance_params.spring_stiffness/2;
	safety_impedance.damping = impedance_params.damping;
	safety_impedance.inertia = impedance_params.inertia;
	safety_impedance.repulsion_stiffness = impedance_params.repulsion_stiffness/4;
	safety_impedance.repulsion_damping = impedance_params.repulsion_damping * 1.5;
	construct_impedance_message(safety_impedance);
	impedance_publisher.publish(this->compliance_update);
	ROS_INFO("bringing you object ");
	task_planner.execute_action(this->goal_pose_.head(3), this->goal_pose_.tail(3), &publisher, 0.03);
	task_planner.open_gripper();
}

// Default constructor for FollowMe
FollowMe::FollowMe() : ActionPrimitive() {
	// Custom values for FollowMe
	Eigen::Matrix<double, 6, 6> stiffness;  
	stiffness.topLeftCorner(3, 3) << 200, 0, 0, 0, 200, 0, 0, 0, 200;
	stiffness.bottomRightCorner(3, 3) << 10, 0, 0, 0, 10, 0, 0, 0, 5;
	Eigen::Matrix<double, 6, 6> damping;           
	damping.topLeftCorner(3, 3) << 28, 0, 0, 0, 28, 0, 0, 0, 28;
	damping.bottomRightCorner(3, 3) << 3, 0, 0, 0, 3, 0, 0, 0, 3;
	//ToDo: Inerta matrix should be specified as multiple of physical inertia
	Eigen::Matrix<double, 6, 6> inertia;           
	inertia.topLeftCorner(3, 3) << 200, 0, 0, 0, 200, 0, 0, 0, 200;
	inertia.bottomRightCorner(3, 3) << 50, 0, 0, 0, 50, 0, 0, 0, 50;
	Eigen::Matrix<double, 6, 6> bubble_stiffness;  
	bubble_stiffness.topLeftCorner(3, 3) << 10, 0, 0, 0, 10, 0, 0, 0, 10;
	bubble_stiffness.bottomRightCorner(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	Eigen::Matrix<double, 6, 6> bubble_damping;   
	bubble_damping.topLeftCorner(3, 3) << 5, 0, 0, 0, 5, 0, 0, 0, 5;
	bubble_damping.bottomRightCorner(3, 3) << 0.5, 0, 0, 0, 0.5, 0, 0, 0, 0.5;
	setParameters(stiffness, damping, inertia,
	              bubble_stiffness, bubble_damping);


}

void FollowMe::performAction(TaskPlanner &task_planner, ros::Publisher &goal_publisher,
                             ros::Publisher &impedance_publisher) {
	// Implementation of performAction for FollowMe
	// Custom logic for FollowMe
	construct_impedance_message(this->impedance_params);
	impedance_publisher.publish(this->compliance_update);
	//go to hand
	//ToDo: Implement the loops in switch case of node -> Active Task will be switched by callback
    // task_planner.execute_action(this->goal_pose_.head(3), this->goal_pose_.tail(3), &goal_publisher, 0.03);
	//execute first action, then we will just update the goal position and publish it in the right_hand callback

}

// Default constructor for HoldThis
HoldThis::HoldThis() : ActionPrimitive() {
	// Custom values for HoldThis
	Eigen::Matrix<double, 6, 6> stiffness;  
	stiffness.topLeftCorner(3, 3) << 650, 0, 0, 0, 650, 0, 0, 0, 650;
	stiffness.bottomRightCorner(3, 3) << 150, 0, 0, 0, 150, 0, 0, 0, 20;
	Eigen::Matrix<double, 6, 6> damping;           
	damping.topLeftCorner(3, 3) << 52, 0, 0, 0, 52, 0, 0, 0, 52;
	damping.bottomRightCorner(3, 3) << 25, 0, 0, 0, 25, 0, 0, 0, 9;
	//ToDo: Inerta matrix should be specified as multiple of physical inertia
	Eigen::Matrix<double, 6, 6> inertia;           
	inertia.topLeftCorner(3, 3) << 200, 0, 0, 0, 200, 0, 0, 0, 200;
	inertia.bottomRightCorner(3, 3) << 50, 0, 0, 0, 50, 0, 0, 0, 50;
	Eigen::Matrix<double, 6, 6> bubble_stiffness;  
	bubble_stiffness.topLeftCorner(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	bubble_stiffness.bottomRightCorner(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	Eigen::Matrix<double, 6, 6> bubble_damping;   
	bubble_damping.topLeftCorner(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	bubble_damping.bottomRightCorner(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	setParameters(stiffness, damping, inertia,
	              bubble_stiffness, bubble_damping);

}

void HoldThis::performAction(TaskPlanner &task_planner, ros::Publisher &goal_publisher,
                             ros::Publisher &impedance_publisher) {
	// Implementation of performAction for HoldThis
	construct_impedance_message(this->impedance_params);
	impedance_publisher.publish(this->compliance_update);
	// Custom logic for HoldThis
	//go back to hand/object
	ros::Rate waiting_time(0.1);
	task_planner.execute_action(this->goal_pose_.head(3), this->goal_pose_.tail(3), &goal_publisher, 0.03);
	task_planner.open_gripper();
	task_planner.grasp_object();
	//stopping here will effectually do nothing until the next callback comes in

}

// Default constructor for TakeThis
TakeThis::TakeThis() : ActionPrimitive() {
	// Custom values for TakeThis
	//Stiffness
	Eigen::Matrix<double, 6, 6> stiffness;  
	stiffness =  Eigen::MatrixXd::Identity(6,6);
	stiffness.topLeftCorner(3, 3) << 300, 0, 0, 0, 300, 0, 0, 0, 300;
	stiffness.bottomRightCorner(3, 3) << 50, 0, 0, 0, 50, 0, 0, 0, 12;
	//Damping
	Eigen::Matrix<double, 6, 6> damping;           
	damping =  Eigen::MatrixXd::Identity(6,6);
	damping.topLeftCorner(3, 3) << 40, 0, 0, 0, 40, 0, 0, 0, 40;
	damping.bottomRightCorner(3, 3) << 15, 0, 0, 0, 15, 0, 0, 0, 3;
	Eigen::Matrix<double, 6, 6> inertia;           
	//ToDo: Inerta matrix should be specified as multiple of physical inertia
	inertia.topLeftCorner(3, 3) << 200, 0, 0, 0, 200, 0, 0, 0, 200;
	inertia.bottomRightCorner(3, 3) << 50, 0, 0, 0, 50, 0, 0, 0, 50;
	Eigen::Matrix<double, 6, 6> bubble_stiffness;  
	bubble_stiffness.topLeftCorner(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	bubble_stiffness.bottomRightCorner(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	Eigen::Matrix<double, 6, 6> bubble_damping;   
	bubble_damping.topLeftCorner(3, 3) << 20, 0, 0, 0, 20, 0, 0, 0, 20;
	bubble_damping.bottomRightCorner(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	setParameters(stiffness, damping, inertia,
	              bubble_stiffness, bubble_damping);

	// Additional custom initialization for GetMe if needed
}

void TakeThis::performAction(TaskPlanner &task_planner, ros::Publisher &goal_publisher,
                             ros::Publisher &impedance_publisher) {
	// Implementation of performAction for TakeThis
	// Custom logic for TakeThis
	task_planner.open_gripper();
	//go to object/hand
	//now construct more careful impedances to be safe in handover
	ImpedanceMatrices safety_impedance;
	safety_impedance.spring_stiffness = impedance_params.spring_stiffness/2;
	safety_impedance.damping = impedance_params.damping;
	safety_impedance.inertia = impedance_params.inertia;
	safety_impedance.repulsion_stiffness = impedance_params.repulsion_stiffness/4;
	safety_impedance.repulsion_damping = impedance_params.repulsion_damping * 1.5;
	construct_impedance_message(safety_impedance);
	impedance_publisher.publish(this->compliance_update);
	// execute handover
	task_planner.execute_action(this->getObjectPose().head(3), this->getObjectPose().tail(3), &goal_publisher, 0.03);
	task_planner.grasp_object();
	//ToDo: goal_pose should be fixed at the start of action
	//go back to delivery pose
	// reset to default impedance
	construct_impedance_message(this->impedance_params);
	impedance_publisher.publish(this->compliance_update);
	task_planner.execute_action(this->goal_pose_.head(3), this->goal_pose_.tail(3), &goal_publisher, 0.03);
	task_planner.open_gripper();

}

// Default constructor for AvoidMe
AvoidMe::AvoidMe() : ActionPrimitive() {
	// Custom values for AvoidMe
	Eigen::Matrix<double, 6, 6> stiffness;  
	stiffness.topLeftCorner(3, 3) << 450, 0, 0, 0, 450, 0, 0, 0, 450;
	stiffness.bottomRightCorner(3, 3) << 100, 0, 0, 0, 100, 0, 0, 0, 10;
	Eigen::Matrix<double, 6, 6> damping;           
	damping.topLeftCorner(3, 3) << 45, 0, 0, 0, 45, 0, 0, 0, 45;
	damping.bottomRightCorner(3, 3) << 20, 0, 0, 0, 20, 0, 0, 0, 4;
	Eigen::Matrix<double, 6, 6> inertia;           
	//ToDo: Inerta matrix should be specified as multiple of physical inertia
	inertia.topLeftCorner(3, 3) << 200, 0, 0, 0, 200, 0, 0, 0, 200;
	inertia.bottomRightCorner(3, 3) << 50, 0, 0, 0, 50, 0, 0, 0, 50;
	//
	Eigen::Matrix<double, 6, 6> bubble_stiffness;  
	bubble_stiffness.topLeftCorner(3, 3) << 350, 0, 0, 0, 350, 0, 0, 0, 350;
	bubble_stiffness.bottomRightCorner(3, 3) << 50, 0, 0, 0, 50, 0, 0, 0, 50;
	Eigen::Matrix<double, 6, 6> bubble_damping;   
	bubble_damping.topLeftCorner(3, 3) << 15, 0, 0, 0, 15, 0, 0, 0, 15;
	bubble_damping.bottomRightCorner(3, 3) << 3, 0, 0, 0, 3, 0, 0, 0, 1;
	setParameters(stiffness, damping, inertia,
	              bubble_stiffness, bubble_damping);
	
}

void
AvoidMe::performAction(TaskPlanner &task_planner, ros::Publisher &goal_publisher, ros::Publisher &impedance_publisher) {
	// Implementation of performAction for AvoidMe
	// Custom logic for AvoidMe
	construct_impedance_message(this->impedance_params);
	impedance_publisher.publish(this->compliance_update);
	task_planner.execute_action(this->goal_pose_.head(3), this->goal_pose_.tail(3), &goal_publisher, 0.03); //just go to goal
}

