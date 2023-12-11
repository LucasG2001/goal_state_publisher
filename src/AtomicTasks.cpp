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
	Eigen::Matrix<double, 6, 6> stiffness;  // customize this matrix
	stiffness.topLeftCorner(3, 3) << 200, 0, 0, 0, 200, 0, 0, 0, 200;
	stiffness.bottomRightCorner(3, 3) << 50, 0, 0, 0, 50, 0, 0, 0, 50;
	Eigen::Matrix<double, 6, 6> damping;           // customize this matrix
	damping.topLeftCorner(3, 3) << 200, 0, 0, 0, 200, 0, 0, 0, 200;
	damping.bottomRightCorner(3, 3) << 50, 0, 0, 0, 50, 0, 0, 0, 50;
	Eigen::Matrix<double, 6, 6> inertia;           // customize this matrix
	inertia.topLeftCorner(3, 3) << 200, 0, 0, 0, 200, 0, 0, 0, 200;
	inertia.bottomRightCorner(3, 3) << 50, 0, 0, 0, 50, 0, 0, 0, 50;
	Eigen::Matrix<double, 6, 6> bubble_stiffness;  // customize this matrix
	bubble_stiffness.topLeftCorner(3, 3) << 200, 0, 0, 0, 200, 0, 0, 0, 200;
	bubble_stiffness.bottomRightCorner(3, 3) << 50, 0, 0, 0, 50, 0, 0, 0, 50;
	Eigen::Matrix<double, 6, 6> bubble_damping;   // customize this matrix
	bubble_damping.topLeftCorner(3, 3) << 200, 0, 0, 0, 200, 0, 0, 0, 200;
	bubble_damping.bottomRightCorner(3, 3) << 50, 0, 0, 0, 50, 0, 0, 0, 50;
	setParameters(stiffness, damping, inertia,
	              bubble_stiffness, bubble_damping);

	// Additional custom initialization for GetMe if needed
}

void GetMe::performAction(TaskPlanner& task_planner, ros::Publisher &publisher) {
	// Implementation of performAction for GetMe
	// Custom logic for GetMe
	task_planner.open_gripper();
	//go to object
	task_planner.execute_action(this->getObjectPose().head(3), this->getObjectPose().tail(3), &publisher, 0.03);
	task_planner.grasp_object();
	//ToDo: goal_pose should be updated as hand pose -> in callbacks?
	//go back to hand
	task_planner.execute_action(this->goal_pose_.head(3), this->goal_pose_.tail(3), &publisher, 0.03);
	task_planner.open_gripper();
}

// Default constructor for FollowMe
FollowMe::FollowMe() : ActionPrimitive() {
	// Custom values for FollowMe
	Eigen::Matrix<double, 6, 6> stiffness;  // customize this matrix
	stiffness.topLeftCorner(3, 3) << 200, 0, 0, 0, 200, 0, 0, 0, 200;
	stiffness.bottomRightCorner(3, 3) << 50, 0, 0, 0, 50, 0, 0, 0, 50;
	Eigen::Matrix<double, 6, 6> damping;           // customize this matrix
	damping.topLeftCorner(3, 3) << 200, 0, 0, 0, 200, 0, 0, 0, 200;
	damping.bottomRightCorner(3, 3) << 50, 0, 0, 0, 50, 0, 0, 0, 50;
	Eigen::Matrix<double, 6, 6> inertia;           // customize this matrix
	inertia.topLeftCorner(3, 3) << 200, 0, 0, 0, 200, 0, 0, 0, 200;
	inertia.bottomRightCorner(3, 3) << 50, 0, 0, 0, 50, 0, 0, 0, 50;
	Eigen::Matrix<double, 6, 6> bubble_stiffness;  // customize this matrix
	bubble_stiffness.topLeftCorner(3, 3) << 200, 0, 0, 0, 200, 0, 0, 0, 200;
	bubble_stiffness.bottomRightCorner(3, 3) << 50, 0, 0, 0, 50, 0, 0, 0, 50;
	Eigen::Matrix<double, 6, 6> bubble_damping;   // customize this matrix
	bubble_damping.topLeftCorner(3, 3) << 200, 0, 0, 0, 200, 0, 0, 0, 200;
	bubble_damping.bottomRightCorner(3, 3) << 50, 0, 0, 0, 50, 0, 0, 0, 50;
	setParameters(stiffness, damping, inertia,
	              bubble_stiffness, bubble_damping);


}

void FollowMe::performAction(TaskPlanner& task_planner, ros::Publisher &publisher){
	// Implementation of performAction for FollowMe
	// Custom logic for FollowMe
	//ToDo: goal_pose should be updated as hand pose -> in callbacks?
	//go to hand
	ros::Rate hand_rate(50);
	//ToDo: Implement the loops in switch case of node -> Active Task will be switched by callback
	while (true){
		task_planner.execute_action(this->goal_pose_.head(3), this->goal_pose_.tail(3), &publisher, 0.03);
		hand_rate.sleep();
	}

}

// Default constructor for HoldThis
HoldThis::HoldThis() : ActionPrimitive() {
	// Custom values for HoldThis
	Eigen::Matrix<double, 6, 6> stiffness;  // customize this matrix
	stiffness.topLeftCorner(3, 3) << 200, 0, 0, 0, 200, 0, 0, 0, 200;
	stiffness.bottomRightCorner(3, 3) << 50, 0, 0, 0, 50, 0, 0, 0, 50;
	Eigen::Matrix<double, 6, 6> damping;           // customize this matrix
	damping.topLeftCorner(3, 3) << 200, 0, 0, 0, 200, 0, 0, 0, 200;
	damping.bottomRightCorner(3, 3) << 50, 0, 0, 0, 50, 0, 0, 0, 50;
	Eigen::Matrix<double, 6, 6> inertia;           // customize this matrix
	inertia.topLeftCorner(3, 3) << 200, 0, 0, 0, 200, 0, 0, 0, 200;
	inertia.bottomRightCorner(3, 3) << 50, 0, 0, 0, 50, 0, 0, 0, 50;
	Eigen::Matrix<double, 6, 6> bubble_stiffness;  // customize this matrix
	bubble_stiffness.topLeftCorner(3, 3) << 200, 0, 0, 0, 200, 0, 0, 0, 200;
	bubble_stiffness.bottomRightCorner(3, 3) << 50, 0, 0, 0, 50, 0, 0, 0, 50;
	Eigen::Matrix<double, 6, 6> bubble_damping;   // customize this matrix
	bubble_damping.topLeftCorner(3, 3) << 200, 0, 0, 0, 200, 0, 0, 0, 200;
	bubble_damping.bottomRightCorner(3, 3) << 50, 0, 0, 0, 50, 0, 0, 0, 50;
	setParameters(stiffness, damping, inertia,
	              bubble_stiffness, bubble_damping);

}

void HoldThis::performAction(TaskPlanner& task_planner, ros::Publisher &publisher) {
	// Implementation of performAction for HoldThis
	// Custom logic for HoldThis
	//go back to hand/object
	ros::Rate waiting_time(0.1);
	task_planner.execute_action(this->goal_pose_.head(3), this->goal_pose_.tail(3), &publisher, 0.03);
	task_planner.open_gripper();
	task_planner.grasp_object();
	//wait while active task is not changed
	//ToDo: Implement the loops in switch case of node
	while(true){
		waiting_time.sleep();
	}
}

// Default constructor for TakeThis
TakeThis::TakeThis() : ActionPrimitive() {
	// Custom values for TakeThis
	Eigen::Matrix<double, 6, 6> stiffness;  // customize this matrix
	stiffness.topLeftCorner(3, 3) << 200, 0, 0, 0, 200, 0, 0, 0, 200;
	stiffness.bottomRightCorner(3, 3) << 50, 0, 0, 0, 50, 0, 0, 0, 50;
	Eigen::Matrix<double, 6, 6> damping;           // customize this matrix
	damping.topLeftCorner(3, 3) << 200, 0, 0, 0, 200, 0, 0, 0, 200;
	damping.bottomRightCorner(3, 3) << 50, 0, 0, 0, 50, 0, 0, 0, 50;
	Eigen::Matrix<double, 6, 6> inertia;           // customize this matrix
	inertia.topLeftCorner(3, 3) << 200, 0, 0, 0, 200, 0, 0, 0, 200;
	inertia.bottomRightCorner(3, 3) << 50, 0, 0, 0, 50, 0, 0, 0, 50;
	Eigen::Matrix<double, 6, 6> bubble_stiffness;  // customize this matrix
	bubble_stiffness.topLeftCorner(3, 3) << 20, 0, 0, 0, 20, 0, 0, 0, 20;
	bubble_stiffness.bottomRightCorner(3, 3) << 5, 0, 0, 0, 5, 0, 0, 0, 5;
	Eigen::Matrix<double, 6, 6> bubble_damping;   // customize this matrix
	bubble_damping.topLeftCorner(3, 3) << 5, 0, 0, 0, 5, 0, 0, 0, 5;
	bubble_damping.bottomRightCorner(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	setParameters(stiffness, damping, inertia,
	              bubble_stiffness, bubble_damping);

	// Additional custom initialization for GetMe if needed
}

void TakeThis::performAction(TaskPlanner& task_planner, ros::Publisher &publisher) {
	// Implementation of performAction for TakeThis
	// Custom logic for TakeThis
	task_planner.open_gripper();
	//go to object/hand
	task_planner.execute_action(this->getObjectPose().topLeftCorner(3, 3), this->getObjectPose().bottomRightCorner(3, 3), &publisher, 0.03);
	task_planner.grasp_object();
	//ToDo: goal_pose should be fixed at the start of action
	//go back to delivery pose
	task_planner.execute_action(this->goal_pose_.head(3), this->goal_pose_.tail(3), &publisher, 0.03);
	task_planner.open_gripper();

}

// Default constructor for AvoidMe
AvoidMe::AvoidMe() : ActionPrimitive() {
	// Custom values for AvoidMe
	Eigen::Matrix<double, 6, 6> stiffness;  // customize this matrix
	stiffness.topLeftCorner(3, 3) << 200, 0, 0, 0, 200, 0, 0, 0, 200;
	stiffness.bottomRightCorner(3, 3) << 50, 0, 0, 0, 50, 0, 0, 0, 50;
	Eigen::Matrix<double, 6, 6> damping;           // customize this matrix
	damping.topLeftCorner(3, 3) << 200, 0, 0, 0, 200, 0, 0, 0, 200;
	damping.bottomRightCorner(3, 3) << 50, 0, 0, 0, 50, 0, 0, 0, 50;
	Eigen::Matrix<double, 6, 6> inertia;           // customize this matrix
	inertia.topLeftCorner(3, 3) << 200, 0, 0, 0, 200, 0, 0, 0, 200;
	inertia.bottomRightCorner(3, 3) << 50, 0, 0, 0, 50, 0, 0, 0, 50;
	Eigen::Matrix<double, 6, 6> bubble_stiffness;  // customize this matrix
	bubble_stiffness.topLeftCorner(3, 3) << 200, 0, 0, 0, 200, 0, 0, 0, 200;
	bubble_stiffness.bottomRightCorner(3, 3) << 50, 0, 0, 0, 50, 0, 0, 0, 50;
	Eigen::Matrix<double, 6, 6> bubble_damping;   // customize this matrix
	bubble_damping.topLeftCorner(3, 3) << 200, 0, 0, 0, 200, 0, 0, 0, 200;
	bubble_damping.bottomRightCorner(3, 3) << 50, 0, 0, 0, 50, 0, 0, 0, 50;
	setParameters(stiffness, damping, inertia,
	              bubble_stiffness, bubble_damping);
	
}

void AvoidMe::performAction(TaskPlanner& task_planner, ros::Publisher &publisher) {
	// Implementation of performAction for AvoidMe
	// Custom logic for AvoidMe
	task_planner.execute_action(this->goal_pose_.head(3), this->goal_pose_.tail(3), &publisher, 0.03); //just go to goal
}

