//
// Created by lucas on 01.12.23.
//

#include <iostream>
#include <goal_state_publisher/AtomicTasks.h>

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

void GetMe::performAction() {
	// Implementation of performAction for GetMe
	// Custom logic for GetMe
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

void FollowMe::performAction() {
	// Implementation of performAction for FollowMe
	// Custom logic for FollowMe
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

void HoldThis::performAction() {
	// Implementation of performAction for HoldThis
	// Custom logic for HoldThis
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

void TakeThis::performAction() {
	// Implementation of performAction for TakeThis
	// Custom logic for TakeThis
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

void AvoidMe::performAction() {
	// Implementation of performAction for AvoidMe
	// Custom logic for AvoidMe
}

