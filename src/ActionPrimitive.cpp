//
// Created by lucas on 01.12.23.
//
#include "goal_state_publisher/ActionPrimitive.h"  // Update with your actual package name


// Constructor
ActionPrimitive::ActionPrimitive()
//ToDo: Handle Creation of smart default goal, and object poses
		: start_pose_(Eigen::MatrixXd::Identity(6, 1)),
		  goal_pose_(Eigen::VectorXd::Ones(6)), object_pose_(0.5* Eigen::VectorXd::Ones(6)), grasp_(false),
		  impedance_params(){  // Arbitrary 6x6 identity matrix
	// Initialize other members if needed
	//create some useful values
	goal_pose_ << 0.5, 0.0, 0.45, 3.14156, 0.0, 0.0;
	object_pose_ << 0.3, 0.5, 0.15, 3.14156, 0, 0;
	impedance_params.spring_stiffness = Eigen::MatrixXd::Identity(6, 6);  // Arbitrary 6x6 identity matrix
	impedance_params.damping = Eigen::MatrixXd::Identity(6, 6);  // Arbitrary 6x6 identity matrix
	impedance_params.inertia = Eigen::MatrixXd::Identity(6, 6); // Arbitrary 6x6 identity matrix
	impedance_params.repulsion_stiffness = Eigen::MatrixXd::Identity(6, 6);  // Arbitrary 6x6 identity matrix
	impedance_params.repulsion_damping = Eigen::MatrixXd::Identity(6, 6);
}

//construction of Impedance paramter message
void ActionPrimitive::construct_impedance_message(const ImpedanceMatrices &impedance_matrices){
	//copy data
	std::copy(impedance_matrices.spring_stiffness.data(), impedance_matrices.spring_stiffness.data() + 36, compliance_update.stiffness.begin());
	std::copy(impedance_matrices.damping.data(), impedance_matrices.damping.data() + 36, compliance_update.damping.begin());
	std::copy(impedance_matrices.inertia.data(), impedance_matrices.inertia.data() + 36, compliance_update.inertia_factors.begin());
	std::copy(impedance_matrices.repulsion_stiffness.data(), impedance_matrices.repulsion_stiffness.data() + 36, compliance_update.safety_bubble_stiffness.begin());
	std::copy(impedance_matrices.repulsion_damping.data(), impedance_matrices.repulsion_damping.data() + 36, compliance_update.safety_bubble_damping.begin());

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
	impedance_params.spring_stiffness = spring_stiffness;
	impedance_params.damping = damping;
	impedance_params.inertia = inertia;
	impedance_params.repulsion_stiffness = repulsion_stiffness;
	impedance_params.repulsion_damping = repulsion_damping;
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

Eigen::Matrix<double, 6, 6> ActionPrimitive::getSpringStiffness() const {
	return impedance_params.spring_stiffness;
}

Eigen::Matrix<double, 6, 6> ActionPrimitive::getDamping() const {
	return impedance_params.damping;
}

Eigen::Matrix<double, 6, 6> ActionPrimitive::getInertia() const {
	return impedance_params.inertia;
}

Eigen::Matrix<double, 6, 6> ActionPrimitive::getRepulsionStiffness() const {
	//ToDo: make matrix sizes of impedance parameters consistent over all nodes
	return impedance_params.repulsion_stiffness.topLeftCorner(3,3);
}

Eigen::Matrix<double, 6, 6> ActionPrimitive::getRepulsionDamping() const {
	//ToDo: make matrix sizes of impedance parameters consistent over all nodes
	return impedance_params.repulsion_damping.topLeftCorner(3,3);
}
