//
// Created by lucas on 01.12.23.
//
#include <goal_state_publisher/ImpedanceParameterController.h>
#include <std_msgs/Int32.h>
#include <utility.h>

ImpedanceParameterController::ImpedanceParameterController(ros::Publisher* pub)
		: reference_pose_publisher_(pub), get_me_task(), follow_me_task(), hold_this_task(), take_this_task(), avoid_me_task(),
		  activeTask(&hold_this_task), task_planner(), rightHandPose(), leftHandPose(), externalForce() {
	// Initialize other members if needed
}
void ImpedanceParameterController::rightHandCallback(const geometry_msgs::Pose::ConstPtr& msg) {
	// Extract relevant information from the message and set the right hand pose
	Eigen::Vector3d right_hand_position(msg->position.x, msg->position.y, msg->position.z);
	Eigen::Vector3d right_hand_orientation(msg->orientation.x, msg->orientation.y, msg->orientation.z);
	rightHandPose << right_hand_position, right_hand_orientation;
	//if task is FOLLOW ME update the goal pose
	if(activeTask == &follow_me_task){
		activeTask->setGoalPose(rightHandPose);
		reference_pose_publisher_->publish(msg);
	}
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
void ImpedanceParameterController::TaskCallback(const custom_msgs::action_primitive_messageConstPtr& msg) {
	// Extract the integer value from the message
	ROS_INFO("Received Action Primitive Message");
	int task_type = msg->task_type;
	Eigen::Matrix<double, 6, 1> goal_pose = convert_pose_to_eigen(msg->goal_pose);
	Eigen::Matrix<double, 6, 1> object_pose = convert_pose_to_eigen(msg->object_pose);

	// Switch the active task pointer based on the task_type
	switch (task_type) {
		case 1:
			activeTask = &get_me_task;
			ROS_INFO("active task is now GET ME");
			break;
		case 2:
			activeTask = &follow_me_task;
			ROS_INFO("active task is now FOLLOW ME");
			break;
		case 3:
			activeTask = &hold_this_task;
			ROS_INFO("active task is now  HOLD THIS");
			break;
		case 4:
			activeTask = &take_this_task;
			ROS_INFO("active task is now TAKE THIS");
			break;
		case 5:
			activeTask = &avoid_me_task;
			ROS_INFO("active task is now AVOID ME");
			break;
			// Add other cases for additional task types if needed
		default:
			// Handle the default case (if any)
			ROS_WARN("Unknown task type: %d", task_type);
			break;

	}
	activeTask->setGoalPose(goal_pose);
	activeTask->setObjectPose(object_pose);
	activeTask->setGrasp(msg->grasp);
	ROS_INFO("executing task");
	activeTask->performAction(task_planner, *reference_pose_publisher_);

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
