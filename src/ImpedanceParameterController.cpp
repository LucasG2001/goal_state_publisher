//
// Created by lucas on 01.12.23.
//
#include <goal_state_publisher/ImpedanceParameterController.h>
#include <std_msgs/Int32.h>
#include <Utility.h>
#include <custom_msgs/HandPose.h>

#define POSE_NEUTRAL 0.45, 0.0, 0.45, 3.14156, 0.0, 0.0

ImpedanceParameterController::ImpedanceParameterController(ros::Publisher* ref_pub, ros::Publisher* impedance_pub, ros::Publisher* task_finish_pub, ros::Publisher* forcing_pub)
		: reference_pose_publisher_(ref_pub), impedance_param_pub(impedance_pub), task_finished_publisher(task_finish_pub), forcing_publisher(forcing_pub),
		get_me_task(), follow_me_task(), hold_this_task(), take_this_task(), avoid_me_task(),
		  activeTask(&hold_this_task), task_planner(), rightHandPose(), leftHandPose(), externalForce() {
	is_task_finished.data = false;
	// Initialize other members if needed
}
void ImpedanceParameterController::rightHandCallback(const custom_msgs::HandPoseConstPtr msg) {
	// Extract relevant information from the message and set the right hand pose
	// ROS_INFO("Got Hand Position");
	geometry_msgs::PoseStamped goal;
	goal.header.frame_id = "none";
	double angle = M_PI / 2.0;  // 90 degrees in radians
	Eigen::Quaterniond rotationQuat(std::cos(angle / 2), 0, 0, std::sin(angle / 2));  // Rotating about z-axis, we follow the hand rotated by 90 degrees
	Eigen::Quaterniond hand_orientation;
	if (msg->isTracked) {
		hand_orientation.coeffs() << msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w;
		Eigen::Quaterniond follow_orientation =
				rotationQuat * hand_orientation; //rotate the EE by 90 degrees in z direction
		rightHandPose
				<< msg->position.x, msg->position.y, msg->position.z, 3.14156, 0.0, 1.51; //turn by 90 deg around z
		//if task is FOLLOW ME update the goal pose
		if (activeTask == &follow_me_task) {
			//ROS_INFO(" following hand ");
			activeTask->setGoalPose(rightHandPose);
			goal.pose.position.x = msg->position.x + follow_me_task.fixed_offset.x();
			goal.pose.position.y = msg->position.y + follow_me_task.fixed_offset.y();
			goal.pose.position.z = msg->position.z + follow_me_task.fixed_offset.z();
			goal.pose.orientation.x = follow_orientation.x();
			goal.pose.orientation.y = follow_orientation.y();
			goal.pose.orientation.z = follow_orientation.z();
			goal.pose.orientation.w = follow_orientation.w();
			reference_pose_publisher_->publish(goal);
		}
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
	if (activeTask == &get_me_task || activeTask == &take_this_task || activeTask == &hold_this_task){
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

void ImpedanceParameterController::FrankaStateCallback(const franka_msgs::FrankaStateConstPtr & msg) {
	// for the time being we only need ee positions without orientations
	task_planner.global_ee_position.x() = msg->O_T_EE[12]; // O_T_EE is a float[16] array in COLUMN MAJOR format!
	task_planner.global_ee_position.y() = msg->O_T_EE[13];
	task_planner.global_ee_position.z() = msg->O_T_EE[14];
	task_planner.F_ext = Eigen::Map<const Eigen::Matrix<double, 6, 1>>((msg->O_F_ext_hat_K).data());
	Eigen::Affine3d transform(Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::ColMajor>>(msg->O_T_EE.data()));
	task_planner.global_ee_orientation = transform.rotation();
	task_planner.global_ee_euler_angles = task_planner.global_ee_orientation.toRotationMatrix().eulerAngles(0, 1, 2);
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
	double total_orientation; // EE orientation in z direction is turned about 90 degrees if x extent is smaller than y
	neutral_pose << POSE_NEUTRAL;

	// Switch the active task pointer based on the task_type
	switch (task_type) {
		case 1:
			object_pose.tail(3).x() = 3.14156; // EE orientation for grasp should be -PI (neutral) + x rotation of BBOX
			object_pose.tail(3).y() = 0.0;
			object_pose.tail(3).z() = msg->object_orientation_euler.z * M_PI/180; // convert to radian
			if (true){
				ROS_INFO(" situation 0, x smaller than y");
				std::cout << " case 0 got object pose as, " << object_pose.tail(3).z() << std::endl;
				// no 90 deg rotation
			}
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
	//impedance is updated in action execution
	ros::Duration(0.05).sleep();
	ROS_INFO("will perform action");
	activeTask->performAction(task_planner, *reference_pose_publisher_, *impedance_param_pub, *task_finished_publisher);

	if (activeTask == &get_me_task || activeTask == &take_this_task || activeTask == &hold_this_task){
		is_task_finished.data = false; //false if not waiting for forcing
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

	std::copy(damping.data(), damping.data() + 36, compliance_update.damping.begin());

	std::copy(inertia.data(), inertia.data() + 36, compliance_update.inertia_factors.begin());

	std::copy(bubble_stiffness.data(), bubble_stiffness.data() + 36, compliance_update.safety_bubble_stiffness.begin());

	std::copy(bubble_damping.data(), bubble_damping.data() + 36, compliance_update.safety_bubble_damping.begin());

	impedance_param_pub->publish(compliance_update);
}
