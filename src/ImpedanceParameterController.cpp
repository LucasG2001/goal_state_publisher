//
// Created by lucas on 01.12.23.
//
#include <goal_state_publisher/ImpedanceParameterController.h>
#include <std_msgs/Int32.h>
#include <Utility.h>

#define POSE_NEUTRAL 0.45, 0.0, 0.45, 3.14156, 0.0, 0.0

ImpedanceParameterController::ImpedanceParameterController(ros::Publisher* ref_pub, ros::Publisher* impedance_pub, ros::Publisher* task_finish_pub)
		: reference_pose_publisher_(ref_pub), impedance_param_pub(impedance_pub), task_finished_publisher(task_finish_pub), get_me_task(), follow_me_task(), hold_this_task(), take_this_task(), avoid_me_task(),
		  activeTask(&hold_this_task), task_planner(), rightHandPose(), leftHandPose(), externalForce() {
	is_task_finished.data = false;
	// Initialize other members if needed
}
void ImpedanceParameterController::rightHandCallback(const geometry_msgs::Pose::ConstPtr& msg) {
	// Extract relevant information from the message and set the right hand pose
	// ROS_INFO("Got Hand Position");
	geometry_msgs::PoseStamped goal;
	goal.header.frame_id = "none";
	rightHandPose << msg->position.x, msg->position.y, msg->position.z, 3.14156, 0.0, 0.0;
	//if task is FOLLOW ME update the goal pose
	if(activeTask == &follow_me_task){
		//ROS_INFO(" following hand ");
		activeTask->setGoalPose(rightHandPose);
		goal.pose.position.x = msg->position.x + follow_me_task.fixed_offset.x();
		goal.pose.position.y = msg->position.y + follow_me_task.fixed_offset.y();
		goal.pose.position.z = msg->position.z + follow_me_task.fixed_offset.z();
		goal.pose.orientation.x = msg->orientation.x;
		goal.pose.orientation.y = msg->orientation.y;
		goal.pose.orientation.z = msg->orientation.z;
		goal.pose.orientation.w = msg->orientation.w;
		reference_pose_publisher_->publish(goal);
	}
}

void ImpedanceParameterController::leftHandCallback(const geometry_msgs::Pose::ConstPtr& msg) {
	// Extract relevant information from the message and set the left hand pose
	Eigen::Vector3d left_hand_position(msg->position.x, msg->position.y, msg->position.z);
	Eigen::Vector3d left_hand_orientation(msg->orientation.x, msg->orientation.y, msg->orientation.z);
	leftHandPose << left_hand_position, left_hand_orientation;
}

void ImpedanceParameterController::placePoseCallback(const geometry_msgs::Pose::ConstPtr& msg) {
	// Extract relevant information from the message and set the left hand pose
	ROS_INFO("got place pose");
	if (activeTask == &get_me_task || activeTask == &take_this_task){
		Eigen::Matrix<double, 6, 1> new_goal_pose;
		new_goal_pose.head(3) << msg->position.x, msg->position.y, msg->position.z;
		Eigen::Quaterniond new_goal_orientation;
		new_goal_orientation.coeffs() << msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w;
		new_goal_pose.tail(3) = new_goal_orientation.toRotationMatrix().eulerAngles(0, 1, 2);
		ROS_INFO_STREAM("goal pose orientation" << new_goal_pose.tail(3));
		activeTask->setGoalPose(new_goal_pose);
		activeTask->hasGrasped = true;
	}
}

void ImpedanceParameterController::FextCallback(const geometry_msgs::Pose::ConstPtr& msg) {
	// Extract relevant information from the message and set the external force
	Eigen::Vector3d F_ext_position(msg->position.x, msg->position.y, msg->position.z);
	Eigen::Vector3d F_ext_orientation(msg->orientation.x, msg->orientation.y, msg->orientation.z);
	externalForce << F_ext_position, F_ext_orientation;
}

void ImpedanceParameterController::TaskCallback(const custom_msgs::action_primitive_messageConstPtr& msg) {
	// Extract the integer value from the message
	ROS_INFO("Received Action Primitive Message");
	is_task_finished.data = false;
	activeTask->hasGrasped = false; //do not wait for place pose
	//task_finished_publisher->publish(is_task_finished);
	int task_type = msg->task_type;
	Eigen::Matrix<double, 6, 1> goal_pose = convert_pose_to_eigen(msg->goal_pose);
	Eigen::Matrix<double, 6, 1> object_pose = convert_pose_to_eigen(msg->object_pose);
	Eigen::Matrix<double, 6, 1> neutral_pose;
	neutral_pose << POSE_NEUTRAL;

	// Switch the active task pointer based on the task_type
	switch (task_type) {
		case 1:
			object_pose.tail(3).x() = 3.14156; // EE orientation for grasp should be -PI (neutral) + x rotation of BBOX
			object_pose.tail(3).y() = 0.0;
			object_pose.tail(3).z() += msg->grasp * M_PI/2; // EE orientation in z direction is turned about 90 degrees if x extent is smaller than y
			activeTask = &get_me_task;
			ROS_INFO("active task is now GET ME");
			break;
		case 2:
			activeTask = &follow_me_task;
			follow_me_task.hand_pose = this->rightHandPose; //initial hand pose for leash creation
			ROS_INFO("active task is now FOLLOW ME");
			break;
		case 3:
			activeTask = &hold_this_task;
			hold_this_task.free_float = msg->isFreeFloat;
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
	//impedance is updated in action execution
	ros::Duration(0.05).sleep();
	ROS_INFO("will perform action");
	activeTask->performAction(task_planner, *reference_pose_publisher_, *impedance_param_pub, *task_finished_publisher);

	if (activeTask == &get_me_task || activeTask == &take_this_task){
		ROS_INFO("publishing info over goal state reached");
		is_task_finished.data = true;
		task_finished_publisher->publish(is_task_finished);
		//go back to neutral
		activeTask = &avoid_me_task;
		activeTask->setGoalPose(neutral_pose);
		activeTask->setObjectPose(neutral_pose);
		activeTask->setGrasp(false);
		ROS_INFO("going back to neutral");
		activeTask->performAction(task_planner, *reference_pose_publisher_, *impedance_param_pub, *task_finished_publisher);
	}

}


void ImpedanceParameterController::setActiveTask(ActionPrimitive& desired_task) {
	this->activeTask = &desired_task;
}

Eigen::Matrix<double, 6, 6> ImpedanceParameterController::getStiffness() {
	return this->activeTask->getSpringStiffness();
}

void ImpedanceParameterController::updateImpedanceParameters() const {
	//ToDo: Implement if useful here
	custom_msgs::ImpedanceParameterMsg compliance_update;
	Eigen::Matrix<double, 6, 6> stiffness, damping, inertia;
	Eigen::Matrix<double, 6, 6> bubble_stiffness, bubble_damping;
	stiffness << activeTask->getSpringStiffness();
	damping << activeTask->getDamping();
	inertia << activeTask->getInertia();
	bubble_stiffness << activeTask->getRepulsionStiffness();
	bubble_damping << activeTask->getRepulsionDamping();

	ROS_INFO("Updating Impedance parameters");
	std::copy(stiffness.data(), stiffness.data() + 36, compliance_update.stiffness.begin());
	ROS_INFO("Updated Stiffness");
	std::copy(damping.data(), damping.data() + 36, compliance_update.damping.begin());
	ROS_INFO("Updated Damping");
	std::copy(inertia.data(), inertia.data() + 36, compliance_update.inertia_factors.begin());
	ROS_INFO("Updated Inertia");
	std::copy(bubble_stiffness.data(), bubble_stiffness.data() + 36, compliance_update.safety_bubble_stiffness.begin());
	ROS_INFO("Updated Safety bubble Stiffness");
	std::copy(bubble_damping.data(), bubble_damping.data() + 36, compliance_update.safety_bubble_damping.begin());
	ROS_INFO("Updated safety bubble damping");

	impedance_param_pub->publish(compliance_update);
	ROS_INFO("published message");
}
