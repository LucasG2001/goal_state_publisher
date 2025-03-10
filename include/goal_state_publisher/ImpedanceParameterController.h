//
// Created by lucas on 01.12.23.
//
#ifndef IMPEDANCE_PARAMETER_CONTROLLER_H
#define IMPEDANCE_PARAMETER_CONTROLLER_H


#include <goal_state_publisher/AtomicTasks.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Dense>
#include <TaskPlanner.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <custom_msgs/action_primitive_message.h>
#include <custom_msgs/ImpedanceParameterMsg.h>
#include <geometry_msgs/Point.h>
#include <custom_msgs/HandPose.h>
#include <custom_msgs/PlacePose.h>
class ImpedanceParameterController {
public:
	explicit ImpedanceParameterController(ros::Publisher* ref_pub, ros::Publisher* impedance_pub, ros::Publisher* task_finish_pub, ros::Publisher* forcing_pub);
	// Callback functions
	void rightHandCallback(const custom_msgs::HandPoseConstPtr msg);
	void leftHandCallback(const geometry_msgs::Pose::ConstPtr& msg);
	void placePoseCallback(const custom_msgs::PlacePoseConstPtr& msg);
	void FrankaStateCallback(const franka_msgs::FrankaStateConstPtr & msg);
	void TaskCallback(const custom_msgs::action_primitive_messageConstPtr& msg);

	// Setters for active task
	void setActiveTask(ActionPrimitive& desired_task);
	// setter to execute a task

	// Call this method to update the impedance parameters based on the active task
	void updateImpedanceParameters() const;

	//dummy function
	//ToDo: implement fully
	Eigen::Matrix<double, 6, 6> getStiffness();
	ActionPrimitive* activeTask;

	GetMe get_me_task;
	FollowMe follow_me_task;
	HoldThis hold_this_task;
	TakeThis take_this_task;
	AvoidMe avoid_me_task;

	//fields to execute and publish action primitive trajectories
	TaskPlanner task_planner;
	ros::Publisher* reference_pose_publisher_;  // Pointer to ROS publisher
	ros::Publisher* impedance_param_pub;  // Pointer to ROS publisher
	ros::Publisher* task_finished_publisher;  // Pointer to ROS publisher
	ros::Publisher* forcing_publisher;  // Pointer to ROS publisher
	//feedback message
	std_msgs::Bool is_task_finished;


private:

	/* Add other actions here */
	Eigen::Matrix<double, 6, 1> rightHandPose;
	Eigen::Matrix<double, 6, 1> leftHandPose;
	Eigen::Matrix<double, 6, 1> externalForce;
	std::mutex mutex; // Mutex member variable
};

#endif // IMPEDANCE_PARAMETER_CONTROLLER_H