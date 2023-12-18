//
// Created by lucas on 01.12.23.
//
#include "goal_state_publisher/ActionPrimitive.h"  // Update with your actual package name


// Constructor
ActionPrimitive::ActionPrimitive()
//ToDo: Handle Creation of smart default goal, and object poses
		: start_pose_(Eigen::MatrixXd::Identity(6, 1)),
		  goal_pose_(Eigen::VectorXd::Ones(6)), object_pose_(0.5* Eigen::VectorXd::Ones(6)), grasp_(false),
		  spring_stiffness_(Eigen::MatrixXd::Identity(6, 6)),  // Arbitrary 6x6 identity matrix
		  damping_(Eigen::MatrixXd::Identity(6, 6)),  // Arbitrary 6x6 identity matrix
		  inertia_(Eigen::MatrixXd::Identity(6, 6)),  // Arbitrary 6x6 identity matrix
		  repulsion_stiffness_(Eigen::MatrixXd::Identity(6, 6)),  // Arbitrary 6x6 identity matrix
		  repulsion_damping_(Eigen::MatrixXd::Identity(6, 6)) {  // Arbitrary 6x6 identity matrix
	// Initialize other members if needed
	//create some useful values
	goal_pose_ << 0.5, 0.0, 0.45, 3.14156, 0.0, 0.0;
	object_pose_ << 0.3, 0.5, 0.15, 3.14156, 0, 0;
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

Eigen::Matrix<double, 3, 3> ActionPrimitive::getRepulsionStiffness() const {
	//ToDo: make matrix sizes of impedance parameters consistent over all nodes
	return repulsion_stiffness_.topLeftCorner(3,3);
}

Eigen::Matrix<double, 3, 3> ActionPrimitive::getRepulsionDamping() const {
	//ToDo: make matrix sizes of impedance parameters consistent over all nodes
	return repulsion_damping_.topLeftCorner(3,3);
}
