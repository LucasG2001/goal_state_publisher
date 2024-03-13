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
	stiffness.bottomRightCorner(3, 3) << 50, 0, 0, 0, 50, 0, 0, 0, 15;

	Eigen::Matrix<double, 6, 6> damping;           
	damping =  Eigen::MatrixXd::Identity(6,6);
	damping.topLeftCorner(3, 3) << 65, 0, 0, 0, 65, 0, 0, 0, 65;
	damping.bottomRightCorner(3, 3) << 15, 0, 0, 0, 15, 0, 0, 0, 4;
	//ToDo: Inerta matrix should be specified as multiple of physical inertia

	Eigen::Matrix<double, 6, 6> inertia;           
	inertia =  Eigen::MatrixXd::Identity(6,6);
	inertia.topLeftCorner(3, 3) << 200, 0, 0, 0, 200, 0, 0, 0, 200;
	inertia.bottomRightCorner(3, 3) << 50, 0, 0, 0, 50, 0, 0, 0, 50;

	Eigen::Matrix<double, 6, 6> bubble_stiffness;  
	bubble_stiffness.topLeftCorner(3, 3) << 220, 0, 0, 0, 220, 0, 0, 0, 180;
	bubble_stiffness.bottomRightCorner(3, 3) << 50, 0, 0, 0, 50, 0, 0, 0, 50;
	Eigen::Matrix<double, 6, 6> bubble_damping;   
	bubble_damping.topLeftCorner(3, 3) << 25, 0, 0, 0, 25, 0, 0, 0, 25;
	bubble_damping.bottomRightCorner(3, 3) << 14, 0, 0, 0, 14, 0, 0, 0, 1;
	setParameters(stiffness, damping, inertia,
	              bubble_stiffness, bubble_damping);

	// Additional custom initialization for GetMe if needed
}

void GetMe::performAction(TaskPlanner& task_planner, ros::Publisher &publisher, ros::Publisher &impedance_publisher) {
	Eigen::Vector3d grasp_offset; grasp_offset << -0.00, 0, 0.08;
	ROS_INFO("starting get me action");
	construct_impedance_message(this->impedance_params);
	impedance_publisher.publish(this->compliance_update);
	// Implementation of performAction for GetMe
	// Custom logic for GetMe
	ROS_INFO("starting gripper open");
	task_planner.open_gripper();
	//go to object here we can have high impedances and low repulsion as we should tipically go away from the human (default)
	ROS_INFO("going towards object ");
	task_planner.primitive_move((this->getObjectPose().head(3)) + grasp_offset, this->getObjectPose().tail(3), &publisher, 0.06);
	task_planner.primitive_move(this->getObjectPose().head(3), this->getObjectPose().tail(3), &publisher, 0.03);
	ROS_INFO("grasping ");
	task_planner.grasp_object();
	//wait for goal pose
	while(!this->hasGrasped){
		ros::Duration(0.1).sleep();
		ros::spinOnce();
	}
	this->hasGrasped = false;
	//go back to hand
	//now construct more different impedances to be safe in handover
	ImpedanceMatrices post_grasp_impedance = this->impedance_params;
	post_grasp_impedance.repulsion_stiffness = impedance_params.repulsion_stiffness/2.0;
	post_grasp_impedance.repulsion_damping = impedance_params.repulsion_damping * 1.25;
	construct_impedance_message(post_grasp_impedance);
	impedance_publisher.publish(this->compliance_update);
	ROS_INFO("bringing you object ");
	//intermediate waypoint
	grasp_offset << -0.05, 0, 0.08;
	task_planner.primitive_move((this->goal_pose_.head(3)) + grasp_offset, this->goal_pose_.tail(3), &publisher, 0.06);
	task_planner.primitive_move(this->goal_pose_.head(3), this->goal_pose_.tail(3), &publisher, 0.08); //higher tolerance for handover
	//do not move
	task_planner.stop(&publisher);
	//SHUT OFF repulsion during opening
	post_grasp_impedance.repulsion_stiffness = impedance_params.repulsion_stiffness * 0;
	post_grasp_impedance.repulsion_damping = impedance_params.repulsion_damping * 0.25;
	construct_impedance_message(post_grasp_impedance);
	impedance_publisher.publish(this->compliance_update);
	task_planner.open_gripper();
}

// Default constructor for FollowMe
FollowMe::FollowMe() : ActionPrimitive() {
	// Custom values for FollowMe
	Eigen::Matrix<double, 6, 6> stiffness;  
	stiffness.topLeftCorner(3, 3) << 200, 0, 0, 0, 200, 0, 0, 0, 200;
	stiffness.bottomRightCorner(3, 3) << 100, 0, 0, 0, 100, 0, 0, 0, 20;
	Eigen::Matrix<double, 6, 6> damping;           
	damping.topLeftCorner(3, 3) << 28, 0, 0, 0, 28, 0, 0, 0, 28;
	damping.bottomRightCorner(3, 3) << 20, 0, 0, 0, 20, 0, 0, 0, 5;
	//ToDo: Inerta matrix should be specified as multiple of physical inertia
	Eigen::Matrix<double, 6, 6> inertia;           
	inertia.topLeftCorner(3, 3) << 200, 0, 0, 0, 200, 0, 0, 0, 200;
	inertia.bottomRightCorner(3, 3) << 50, 0, 0, 0, 50, 0, 0, 0, 50;
	Eigen::Matrix<double, 6, 6> bubble_stiffness;  
	bubble_stiffness.topLeftCorner(3, 3) << 150, 0, 0, 0, 150, 0, 0, 0, 150;
	bubble_stiffness.bottomRightCorner(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	Eigen::Matrix<double, 6, 6> bubble_damping;   
	bubble_damping.topLeftCorner(3, 3) << 10, 0, 0, 0, 10, 0, 0, 0, 10;
	bubble_damping.bottomRightCorner(3, 3) << 0.5, 0, 0, 0, 0.5, 0, 0, 0, 0.5;
	setParameters(stiffness, damping, inertia,
	              bubble_stiffness, bubble_damping);
	//offset for leash
	fixed_offset << 0.1, 0.0, -0.05, 0.0, 0.0, 0.0;
	hand_pose << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;


}

void FollowMe::performAction(TaskPlanner &task_planner, ros::Publisher &goal_publisher,
                             ros::Publisher &impedance_publisher) {
	// Implementation of performAction for FollowMe
	// Custom logic for FollowMe
	ROS_INFO("performing follow me");
	construct_impedance_message(this->impedance_params);
	impedance_publisher.publish(this->compliance_update);
	fixed_offset.head(3) = task_planner.global_ee_position - hand_pose.head(3); //construct the fixed offset once, change orientation upon callback
	fixed_offset.tail(3) << 0.0, 0.0, 0.0;
	// Clamp the fixed offset between -0.3 and 0.3 in the first three dimensions
	fixed_offset.head(3) = fixed_offset.head(3).cwiseMax(-0.3 * Eigen::Vector3d::Ones()).cwiseMin(0.3 * Eigen::Vector3d::Ones());
	ROS_INFO_STREAM("fixed offset leash is " << fixed_offset.head(3));
	//go to hand
	// continuous following is handled in impedance parameter controller
}

// Default constructor for HoldThis
HoldThis::HoldThis() : ActionPrimitive() {
	// Custom values for HoldThis
	Eigen::Matrix<double, 6, 6> stiffness;  
	stiffness.topLeftCorner(3, 3) << 4000, 0, 0, 0, 4000, 0, 0, 0, 4000;
	stiffness.bottomRightCorner(3, 3) << 150, 0, 0, 0, 150, 0, 0, 0, 20;
	Eigen::Matrix<double, 6, 6> damping;           
	damping.topLeftCorner(3, 3) << 85, 0, 0, 0, 85, 0, 0, 0, 85;
	damping.bottomRightCorner(3, 3) << 18, 0, 0, 0, 18, 0, 0, 0, 6;
	//ToDo: Inertia matrix should be specified as multiple of physical inertia
	Eigen::Matrix<double, 6, 6> inertia;           
	inertia.topLeftCorner(3, 3) << 200, 0, 0, 0, 200, 0, 0, 0, 200;
	inertia.bottomRightCorner(3, 3) << 50, 0, 0, 0, 50, 0, 0, 0, 50;
	Eigen::Matrix<double, 6, 6> bubble_stiffness;  
	bubble_stiffness.topLeftCorner(3, 3) << 0, 0, 0, 0, 0, 0, 0, 0, 0;
	bubble_stiffness.bottomRightCorner(3, 3) << 0, 0, 0, 0, 0, 0, 0, 0, 0;
	Eigen::Matrix<double, 6, 6> bubble_damping;   
	bubble_damping.topLeftCorner(3, 3) << 0, 0, 0, 0, 0, 0, 0, 0, 0;
	bubble_damping.bottomRightCorner(3, 3) << 0, 0, 0, 0, 0, 0, 0, 0, 0;
	setParameters(stiffness, damping, inertia,
	              bubble_stiffness, bubble_damping);

	free_float = false;

}

void HoldThis::performAction(TaskPlanner &task_planner, ros::Publisher &goal_publisher,
                             ros::Publisher &impedance_publisher) {
	// Implementation of performAction for HoldThis
	if (this->free_float){
		construct_impedance_message(this->impedance_params * 0.0);
		impedance_publisher.publish(this->compliance_update);
		task_planner.stop(&goal_publisher);
	}
	else {
		construct_impedance_message(this->impedance_params);
		impedance_publisher.publish(this->compliance_update);
		task_planner.stop(&goal_publisher);
	}


	this->free_float = false;

	// Custom logic for HoldThis
	//go back to hand/object
	// ros::Rate waiting_time(0.1);
	//TODO: Wit for goal pose (like bring me)
	// task_planner.execute_action(this->goal_pose_.head(3), this->goal_pose_.tail(3), &goal_publisher, 0.01);
	// task_planner.open_gripper();
	// task_planner.grasp_object();
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
	damping.topLeftCorner(3, 3) << 50, 0, 0, 0, 50, 0, 0, 0, 50;
	damping.bottomRightCorner(3, 3) << 15, 0, 0, 0, 15, 0, 0, 0, 3;
	Eigen::Matrix<double, 6, 6> inertia;           
	//ToDo: Inerta matrix should be specified as multiple of physical inertias
	inertia.topLeftCorner(3, 3) << 200, 0, 0, 0, 200, 0, 0, 0, 200;
	inertia.bottomRightCorner(3, 3) << 50, 0, 0, 0, 50, 0, 0, 0, 50;
	Eigen::Matrix<double, 6, 6> bubble_stiffness;  
	bubble_stiffness.topLeftCorner(3, 3) << 110, 0, 0, 0, 110, 0, 0, 0, 90;
	bubble_stiffness.bottomRightCorner(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	Eigen::Matrix<double, 6, 6> bubble_damping;   
	bubble_damping.topLeftCorner(3, 3) << 20, 0, 0, 0, 20, 0, 0, 0, 18;
	bubble_damping.bottomRightCorner(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	setParameters(stiffness, damping, inertia,
	              bubble_stiffness, bubble_damping);

	// Additional custom initialization for GetMe if needed
}

void TakeThis::performAction(TaskPlanner &task_planner, ros::Publisher &goal_publisher,
                             ros::Publisher &impedance_publisher) {
	ImpedanceMatrices post_grasp_impedance = this->impedance_params;
	Eigen::Vector3d grasp_offset; grasp_offset << -0.04, 0, 0.08;
	// Implementation of performAction for TakeThis
	// Custom logic for TakeThis
	task_planner.open_gripper();
	//go to object/hand
	construct_impedance_message(this->impedance_params);
	impedance_publisher.publish(this->compliance_update);
	// execute handover ( go to hand)
	task_planner.primitive_move((this->getObjectPose().head(3)) + grasp_offset, this->getObjectPose().tail(3), &goal_publisher, 0.06);
	task_planner.primitive_move(this->getObjectPose().head(3), this->getObjectPose().tail(3), &goal_publisher, 0.08); //higher tolerance in handover
	//do not move
	task_planner.stop(&goal_publisher);
	//lower repulsive stiffness during handover
	post_grasp_impedance.repulsion_stiffness = impedance_params.repulsion_stiffness/1000.0;
	post_grasp_impedance.repulsion_damping = impedance_params.repulsion_damping * 0.25;
	construct_impedance_message(post_grasp_impedance);
	impedance_publisher.publish(this->compliance_update);
	//grasp object out of hand
	task_planner.grasp_object();
	while(!this->hasGrasped){
		ros::Duration(0.1).sleep();
		ros::spinOnce();
	}
	this->hasGrasped = false;
	// now that object is grasped increase safety bubble stiffness after a transistory period
	post_grasp_impedance.repulsion_stiffness = impedance_params.repulsion_stiffness * 2;
	post_grasp_impedance.repulsion_damping = impedance_params.repulsion_damping * 1.414; //sqrt of 2
	construct_impedance_message(post_grasp_impedance);
	ros::Duration(1.5).sleep();
	//go back to delivery pose
	impedance_publisher.publish(this->compliance_update);
	//no grasp offset needed
	task_planner.primitive_move(this->goal_pose_.head(3), this->goal_pose_.tail(3), &goal_publisher, 0.03);
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
	bubble_stiffness.topLeftCorner(3, 3) << 500, 0, 0, 0, 500, 0, 0, 0, 500;
	bubble_stiffness.bottomRightCorner(3, 3) << 50, 0, 0, 0, 50, 0, 0, 0, 50;
	Eigen::Matrix<double, 6, 6> bubble_damping;   
	bubble_damping.topLeftCorner(3, 3) << 20, 0, 0, 0, 20, 0, 0, 0, 20;
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
	task_planner.primitive_move(this->goal_pose_.head(3), this->goal_pose_.tail(3), &goal_publisher, 0.02); //just go to goal
}

