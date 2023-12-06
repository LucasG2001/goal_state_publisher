//
// Created by lucas on 01.12.23.
//

#include <iostream>
#include <goal_state_publisher/AtomicTasks.h>
#include <goal_state_publisher/ImpedanceParameterController.h>
#include <TaskPlanner.h>
#include <ros/ros.h>

// Default constructor for GetMe
GetMe::GetMe() : ActionPrimitive() {
	// Custom values for GetMe
	Eigen::Matrix<double, 6, 6> custom_spring_stiffness;  // customize this matrix
	Eigen::Matrix<double, 6, 6> custom_damping;           // customize this matrix
	Eigen::Matrix<double, 6, 6> custom_inertia;           // customize this matrix
	Eigen::Matrix<double, 6, 6> custom_repulsion_stiffness;  // customize this matrix
	Eigen::Matrix<double, 6, 6> custom_repulsion_damping;   // customize this matrix
	setParameters(custom_spring_stiffness, custom_damping, custom_inertia,
	              custom_repulsion_stiffness, custom_repulsion_damping);

	// Additional custom initialization for GetMe if needed
}

void GetMe::performAction(ImpedanceParameterController &impedance_param_controller, ros::Publisher &publisher) {
	// Implementation of performAction for GetMe
	// Custom logic for GetMe
	impedance_param_controller.task_planner.open_gripper();
	//go to object
	impedance_param_controller.task_planner.execute_action(this->getObjectPose().head(3), this->getObjectPose().tail(3), &publisher, 0.03);
	impedance_param_controller.task_planner.grasp_object();
	//ToDo: goal_pose should be updated as hand pose -> in callbacks?
	//go back to hand
	impedance_param_controller.task_planner.execute_action(this->goal_pose_.head(3), this->goal_pose_.tail(3), &publisher, 0.03);
	impedance_param_controller.task_planner.open_gripper();
}

// Default constructor for FollowMe
FollowMe::FollowMe() : ActionPrimitive() {
	// Custom values for FollowMe
	Eigen::Matrix<double, 6, 6> custom_spring_stiffness;  // customize this matrix
	Eigen::Matrix<double, 6, 6> custom_damping;           // customize this matrix
	Eigen::Matrix<double, 6, 6> custom_inertia;           // customize this matrix
	Eigen::Matrix<double, 6, 6> custom_repulsion_stiffness;  // customize this matrix
	Eigen::Matrix<double, 6, 6> custom_repulsion_damping;   // customize this matrix
	setParameters(custom_spring_stiffness, custom_damping, custom_inertia,
	              custom_repulsion_stiffness, custom_repulsion_damping);


}

void FollowMe::performAction(ImpedanceParameterController &impedance_param_controller, ros::Publisher &publisher){
	// Implementation of performAction for FollowMe
	// Custom logic for FollowMe
	//ToDo: goal_pose should be updated as hand pose -> in callbacks?
	//go to hand
	//while true
	impedance_param_controller.task_planner.execute_action(this->goal_pose_.head(3), this->goal_pose_.tail(3), &publisher, 0.03);
	impedance_param_controller.task_planner.open_gripper();
}

// Default constructor for HoldThis
HoldThis::HoldThis() : ActionPrimitive() {
	// Custom values for HoldThis
	Eigen::Matrix<double, 6, 6> custom_spring_stiffness;  // customize this matrix
	Eigen::Matrix<double, 6, 6> custom_damping;           // customize this matrix
	Eigen::Matrix<double, 6, 6> custom_inertia;           // customize this matrix
	Eigen::Matrix<double, 6, 6> custom_repulsion_stiffness;  // customize this matrix
	Eigen::Matrix<double, 6, 6> custom_repulsion_damping;   // customize this matrix
	setParameters(custom_spring_stiffness, custom_damping, custom_inertia,
	              custom_repulsion_stiffness, custom_repulsion_damping);

}

void HoldThis::performAction(ImpedanceParameterController &impedance_param_controller, ros::Publisher &publisher)) {
	// Implementation of performAction for HoldThis
	// Custom logic for HoldThis
	//go back to hand/object
	impedance_param_controller.task_planner.execute_action(this->goal_pose_.head(3), this->goal_pose_.tail(3), &publisher, 0.03);
	impedance_param_controller.task_planner.open_gripper();
	impedance_param_controller.task_planner.grasp_object();
	//ToDo: wait while active task is not changed
}

// Default constructor for TakeThis
TakeThis::TakeThis() : ActionPrimitive() {
	// Custom values for TakeThis
	Eigen::Matrix<double, 6, 6> custom_spring_stiffness;  // customize this matrix
	Eigen::Matrix<double, 6, 6> custom_damping;           // customize this matrix
	Eigen::Matrix<double, 6, 6> custom_inertia;           // customize this matrix
	Eigen::Matrix<double, 6, 6> custom_repulsion_stiffness;  // customize this matrix
	Eigen::Matrix<double, 6, 6> custom_repulsion_damping;   // customize this matrix
	setParameters(custom_spring_stiffness, custom_damping, custom_inertia,
	              custom_repulsion_stiffness, custom_repulsion_damping);

	// Additional custom initialization for GetMe if needed
}

void TakeThis::performAction(ImpedanceParameterController &impedance_param_controller, ros::Publisher &publisher)) {
	// Implementation of performAction for TakeThis
	// Custom logic for TakeThis
	impedance_param_controller.task_planner.open_gripper();
	//go to object/hand
	impedance_param_controller.task_planner.execute_action(this->getObjectPose().head(3), this->getObjectPose().tail(3), &publisher, 0.03);
	impedance_param_controller.task_planner.grasp_object();
	//ToDo: goal_pose should be fixed at the start of action
	//go back to delivery pose
	impedance_param_controller.task_planner.execute_action(this->goal_pose_.head(3), this->goal_pose_.tail(3), &publisher, 0.03);
	impedance_param_controller.task_planner.open_gripper();

}

// Default constructor for AvoidMe
AvoidMe::AvoidMe() : ActionPrimitive() {
	// Custom values for AvoidMe
	Eigen::Matrix<double, 6, 6> custom_spring_stiffness;  // customize this matrix
	Eigen::Matrix<double, 6, 6> custom_damping;           // customize this matrix
	Eigen::Matrix<double, 6, 6> custom_inertia;           // customize this matrix
	Eigen::Matrix<double, 6, 6> custom_repulsion_stiffness;  // customize this matrix
	Eigen::Matrix<double, 6, 6> custom_repulsion_damping;   // customize this matrix
	setParameters(custom_spring_stiffness, custom_damping, custom_inertia,
	              custom_repulsion_stiffness, custom_repulsion_damping);
	
}

void AvoidMe::performAction(ImpedanceParameterController &impedance_param_controller, ros::Publisher &publisher)) {
	// Implementation of performAction for AvoidMe
	// Custom logic for AvoidMe
	impedance_param_controller.task_planner.execute_action(this->goal_pose_.head(3), this->goal_pose_.tail(3), &publisher, 0.03); //just go to goal
}

