//
// Created by lucas on 01.12.23.
//
#include "goal_state_publisher/ActionPrimitive.h"  // Update with your actual package name

// Constructor
ActionPrimitive::ActionPrimitive()
		: start_pose_(), goal_pose_(), object_pose_(), grasp_(false),
		  spring_stiffness_(Eigen::MatrixXd::Identity(6, 6)),  // Arbitrary 6x6 identity matrix
		  damping_(Eigen::MatrixXd::Identity(6, 6)),  // Arbitrary 6x6 identity matrix
		  inertia_(Eigen::MatrixXd::Identity(6, 6)),  // Arbitrary 6x6 identity matrix
		  repulsion_stiffness_(Eigen::MatrixXd::Identity(6, 6)),  // Arbitrary 6x6 identity matrix
		  repulsion_damping_(Eigen::MatrixXd::Identity(6, 6)) {  // Arbitrary 6x6 identity matrix
	// Initialize other members if needed
}

// Setters
void ActionPrimitive::setStartPose(const Eigen::Matrix<double, 6, 1>& start_pose) {
	start_pose_ = start_pose;
}

void ActionPrimitive::setGoalPose(const Eigen::Matrix<double, 6, 1>& goal_pose) {
	goal_pose_ = goal_pose;
}

void ActionPrimitive::setObjectPose(const Eigen::Matrix<double, 6, 1>& object_pose) {
	object_pose_ = object_pose;
}

void ActionPrimitive::setGrasp(bool grasp) {
	grasp_ = grasp;
}

void ActionPrimitive::setParameters(const Eigen::Matrix<double, 6, 6>& spring_stiffness,
                                    const Eigen::Matrix<double, 6, 6>& damping,
                                    const Eigen::Matrix<double, 6, 6>& inertia,
                                    const Eigen::Matrix<double, 6, 6>& repulsion_stiffness,
                                    const Eigen::Matrix<double, 6, 6>& repulsion_damping) {
	spring_stiffness_ = spring_stiffness;
	damping_ = damping;
	inertia_ = inertia;
	repulsion_stiffness_ = repulsion_stiffness;
	repulsion_damping_ = repulsion_damping;
}


// Getters
Eigen::Matrix<double, 6, 1> ActionPrimitive::getStartPose() const {
	return start_pose_;
}

Eigen::Matrix<double, 6, 1> ActionPrimitive::getGoalPose() const {
	return goal_pose_;
}

Eigen::Matrix<double, 6, 1> ActionPrimitive::getObjectPose() const {
	return object_pose_;
}

bool ActionPrimitive::getGrasp() const {
	return grasp_;
}

Eigen::Matrix<double, 6, 6> ActionPrimitive::getSpringStiffness() const {
	return spring_stiffness_;
}

Eigen::Matrix<double, 6, 6> ActionPrimitive::getDamping() const {
	return damping_;
}

Eigen::Matrix<double, 6, 6> ActionPrimitive::getInertia() const {
	return inertia_;
}

Eigen::Matrix<double, 6, 6> ActionPrimitive::getRepulsionStiffness() const {
	return repulsion_stiffness_;
}

Eigen::Matrix<double, 6, 6> ActionPrimitive::getRepulsionDamping() const {
	return repulsion_damping_;
}
