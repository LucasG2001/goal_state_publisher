//
// Created by lucas on 01.12.23.
//
#include <goal_state_publisher/ImpedanceParameterController.h>
#include <std_msgs/Int32.h>

ImpedanceParameterController::ImpedanceParameterController()
		: get_me_task(), follow_me_task(), hold_this_task(), take_this_task(), avoid_me_task(),
		  activeTask(&hold_this_task), rightHandPose(), leftHandPose(), externalForce() {
	// Initialize other members if needed
}
void ImpedanceParameterController::rightHandCallback(const geometry_msgs::Pose::ConstPtr& msg) {
	// Extract relevant information from the message and set the right hand pose
	Eigen::Vector3d right_hand_position(msg->position.x, msg->position.y, msg->position.z);
	Eigen::Vector3d right_hand_orientation(msg->orientation.x, msg->orientation.y, msg->orientation.z);
	rightHandPose << right_hand_position, right_hand_orientation;
}

void ImpedanceParameterController::leftHandCallback(const geometry_msgs::Pose::ConstPtr& msg) {
	// Extract relevant information from the message and set the left hand pose
	Eigen::Vector3d left_hand_position(msg->position.x, msg->position.y, msg->position.z);
	Eigen::Vector3d left_hand_orientation(msg->orientation.x, msg->orientation.y, msg->orientation.z);
	leftHandPose << left_hand_position, left_hand_orientation;
}

void ImpedanceParameterController::FextCallback(const geometry_msgs::Pose::ConstPtr& msg) {
	// Extract relevant information from the message and set the external force
	Eigen::Vector3d F_ext_position(msg->position.x, msg->position.y, msg->position.z);
	Eigen::Vector3d F_ext_orientation(msg->orientation.x, msg->orientation.y, msg->orientation.z);
	externalForce << F_ext_position, F_ext_orientation;
}

// Assume the callback function receives an int message
void ImpedanceParameterController::TaskCallback(const std_msgs::Int32::ConstPtr& int_msg) {
	// Extract the integer value from the message
	int task_type = int_msg->data;

	// Switch the active task pointer based on the task_type
	switch (task_type) {
		case 1:
			activeTask = &get_me_task;
			break;
		case 2:
			activeTask = &follow_me_task;
			break;
		case 3:
			activeTask = &hold_this_task;
			break;
		case 4:
			activeTask = &take_this_task;
			break;
		case 5:
			activeTask = &avoid_me_task;
			break;
			// Add other cases for additional task types if needed
		default:
			// Handle the default case (if any)
			ROS_WARN("Unknown task type: %d", task_type);
			break;
	}
}


void ImpedanceParameterController::setActiveTask(ActionPrimitive& desired_task) {
	this->activeTask = &desired_task;
}

Eigen::Matrix<double, 6, 6> ImpedanceParameterController::getStiffness() {
	return this->activeTask->getSpringStiffness();
}

void ImpedanceParameterController::updateImpedanceParameters() {
	//ToDo: Implement if useful here
}
