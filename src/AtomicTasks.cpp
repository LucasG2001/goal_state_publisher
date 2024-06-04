//
// Created by lucas on 01.12.23.
//

#include <iostream>
#include <goal_state_publisher/AtomicTasks.h>
#include <TaskPlanner.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#define IDENTITY Eigen::MatrixXd::Identity(6,6)

// Default constructor for GetMe
GetMe::GetMe() : ActionPrimitive() {
	// Custom values for GetMe
	Eigen::Matrix<double, 6, 6> stiffness;  
	stiffness =  Eigen::MatrixXd::Identity(6,6);
	stiffness.topLeftCorner(3, 3) << 300, 0, 0, 0, 300, 0, 0, 0, 300;
	stiffness.bottomRightCorner(3, 3) << 50, 0, 0, 0, 50, 0, 0, 0, 15;

	Eigen::Matrix<double, 6, 6> damping;           
	damping =  Eigen::MatrixXd::Identity(6,6);
	damping.topLeftCorner(3, 3) << 75, 0, 0, 0, 75, 0, 0, 0, 75;
	damping.bottomRightCorner(3, 3) << 15, 0, 0, 0, 15, 0, 0, 0, 7.5;
	//ToDo: Inerta matrix should be specified as multiple of physical inertia

	Eigen::Matrix<double, 6, 6> inertia;           
	inertia =  Eigen::MatrixXd::Identity(6,6);
	inertia.topLeftCorner(3, 3) << 200, 0, 0, 0, 200, 0, 0, 0, 200;
	inertia.bottomRightCorner(3, 3) << 50, 0, 0, 0, 50, 0, 0, 0, 50;

	Eigen::Matrix<double, 6, 6> bubble_stiffness;  
	bubble_stiffness.topLeftCorner(3, 3) << 150, 0, 0, 0, 150, 0, 0, 0, 150;
	bubble_stiffness.bottomRightCorner(3, 3) << 50, 0, 0, 0, 50, 0, 0, 0, 50;
	Eigen::Matrix<double, 6, 6> bubble_damping;   
	bubble_damping.topLeftCorner(3, 3) << 25, 0, 0, 0, 25, 0, 0, 0, 25;
	bubble_damping.bottomRightCorner(3, 3) << 14, 0, 0, 0, 14, 0, 0, 0, 1;
	setParameters(stiffness, damping, inertia,
	              bubble_stiffness, bubble_damping);

	// Additional custom initialization for GetMe if needed
}

void GetMe::performAction(TaskPlanner &task_planner, ros::Publisher &goal_publisher, ros::Publisher &impedance_publisher,
                          ros::Publisher &is_task_finished_publisher) {
	std::random_device dev;
	std::mt19937 rng(dev());
	std::uniform_int_distribution<std::mt19937::result_type> dist6(1,6); // distribution in range [1, 6]
	int rand_number = dist6(rng);
	//
	Eigen::Vector3d grasp_offset; grasp_offset << 0, 0, 0.08;
	ROS_INFO("starting get me action");
	construct_impedance_message(this->impedance_params);
	impedance_publisher.publish(this->compliance_update);
	// Implementation of performAction for GetMe
	// Custom logic for GetMe
	ROS_INFO("starting gripper open");
	task_planner.open_gripper();
	//go to object here we can have high impedances and low repulsion as we should tipically go away from the human (default)
	ROS_INFO("going towards object ");
	task_planner.primitive_move((this->getObjectPose().head(3)) + grasp_offset, this->getObjectPose().tail(3), &goal_publisher, 0.08, "grasp");
	task_planner.primitive_move(this->getObjectPose().head(3), this->getObjectPose().tail(3), &goal_publisher, 0.03, "grasp");
	ROS_INFO("grasping ");
	ros::Duration(0.1).sleep();
	task_planner.grasp_object();
	//publish that it finished grasping
	std_msgs::Bool done_msg; done_msg.data = false; //publish false when waiting for goal pse publish true when waiting for forcing
	is_task_finished_publisher.publish(done_msg);
	//wait for goal pose
	ROS_INFO("Finished grasping, waiting for goal ");
	while(!this->hasGrasped){
		ros::spinOnce();
		ros::Duration(0.1).sleep();
	}
	//TODO: add publihing of goal pose reached here
	this->hasGrasped = false;
	//go back to hand
	ROS_INFO("finished waiting ");
	//now construct more different impedances to be safe in handover
	ImpedanceMatrices post_grasp_impedance = this->impedance_params;
	post_grasp_impedance.repulsion_stiffness = impedance_params.repulsion_stiffness/2.0;
	post_grasp_impedance.repulsion_damping = impedance_params.repulsion_damping/1.41;
	construct_impedance_message(post_grasp_impedance);
	impedance_publisher.publish(this->compliance_update);
	ROS_INFO("bringing you object ");
	//intermediate waypoint
	//TODO: should we handle if going to hand or to goal pose directly?
	grasp_offset << -0.05, 0, 0.08;
	task_planner.primitive_move((this->goal_pose_.head(3)) + grasp_offset, this->goal_pose_.tail(3), &goal_publisher, 0.04, "grasp");
	task_planner.primitive_move(this->goal_pose_.head(3), this->goal_pose_.tail(3), &goal_publisher, 0.04, "grasp"); //higher tolerance for handover

	//SHUT OFF repulsion during opening
	post_grasp_impedance.repulsion_stiffness = impedance_params.repulsion_stiffness * 0;
	post_grasp_impedance.repulsion_damping = impedance_params.repulsion_damping * 0.25;
	construct_impedance_message(post_grasp_impedance);
	impedance_publisher.publish(this->compliance_update);
	//do not move
	task_planner.stop(&goal_publisher);
	ros::Duration(0.1).sleep();
	//wait for human input, i.e. forcing to open gripper
	ROS_INFO("Waiting for forcing");
	done_msg.data = true; //true for waiting for forcing
	is_task_finished_publisher.publish(done_msg);
	// Start time
    ros::Time start_time = ros::Time::now();
	double measured_force = 0.0;
	while(measured_force < 5.5){
		double elapsed_time = (ros::Time::now() - start_time).toSec();
		measured_force = task_planner.F_ext.norm();
		std::cout << "Fext is " << measured_force << "and random number is" << rand_number <<std::endl;
		ros::spinOnce();
		ros::Duration(0.2).sleep();
		if(elapsed_time > 4.0){
			break;
		}
	}
	std::cout << "finished while loop" << std::endl;
	task_planner.open_gripper();
}

// Default constructor for FollowMe
FollowMe::FollowMe() : ActionPrimitive() {
	// Custom values for FollowMe
	Eigen::Matrix<double, 6, 6> stiffness;  
	stiffness.topLeftCorner(3, 3) << 200, 0, 0, 0, 200, 0, 0, 0, 200;
	stiffness.bottomRightCorner(3, 3) << 50, 0, 0, 0, 50, 0, 0, 0, 15;
	Eigen::Matrix<double, 6, 6> damping;           
	damping.topLeftCorner(3, 3) << 35, 0, 0, 0, 35, 0, 0, 0, 35;
	damping.bottomRightCorner(3, 3) << 20, 0, 0, 0, 20, 0, 0, 0, 6;
	//ToDo: Inerta matrix should be specified as multiple of physical inertia
	Eigen::Matrix<double, 6, 6> inertia;           
	inertia.topLeftCorner(3, 3) << 200, 0, 0, 0, 200, 0, 0, 0, 200;
	inertia.bottomRightCorner(3, 3) << 50, 0, 0, 0, 50, 0, 0, 0, 50;
	Eigen::Matrix<double, 6, 6> bubble_stiffness;  
	bubble_stiffness.topLeftCorner(3, 3) << 150, 0, 0, 0, 150, 0, 0, 0, 150;
	bubble_stiffness.bottomRightCorner(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	Eigen::Matrix<double, 6, 6> bubble_damping;   
	bubble_damping.topLeftCorner(3, 3) << 10, 0, 0, 0, 10, 0, 0, 0, 10;
	bubble_damping.bottomRightCorner(3, 3) << 0.5, 0, 0, 0, 0.5, 0, 0, 0, 0.5;
	setParameters(stiffness, damping, inertia,
	              bubble_stiffness, bubble_damping);
	//offset for leash
	fixed_offset << 0.1, 0.0, -0.05, 0.0, 0.0, 0.0;
	hand_pose << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;


}

void
FollowMe::performAction(TaskPlanner &task_planner, ros::Publisher &goal_publisher, ros::Publisher &impedance_publisher,
                        ros::Publisher &is_task_finished_publisher) {
	// Implementation of performAction for FollowMe
	// Custom logic for FollowMe
	ROS_INFO("performing follow me");
	construct_impedance_message(this->impedance_params);
	impedance_publisher.publish(this->compliance_update);
	fixed_offset.head(3) = task_planner.global_ee_position - hand_pose.head(3); //construct the fixed offset once, change orientation upon callback
	fixed_offset.tail(3) << 0.0, 0.0, 0.0;
	// Clamp the fixed offset between -0.3 and 0.3 in the first three dimensions
	fixed_offset.head(3) = fixed_offset.head(3).cwiseMax(-0.3 * Eigen::Vector3d::Ones()).cwiseMin(0.3 * Eigen::Vector3d::Ones());
	ROS_INFO_STREAM("fixed offset leash is " << fixed_offset.head(3));
	//go to hand
	// continuous following is handled in impedance parameter controller
}

// Default constructor for HoldThis
HoldThis::HoldThis() : ActionPrimitive() {
	// Custom values for HoldThis
	Eigen::Matrix<double, 6, 6> stiffness;  
	stiffness.topLeftCorner(3, 3) << 4000, 0, 0, 0, 4000, 0, 0, 0, 4000;
	stiffness.bottomRightCorner(3, 3) << 150, 0, 0, 0, 150, 0, 0, 0, 15;
	Eigen::Matrix<double, 6, 6> damping;           
	damping.topLeftCorner(3, 3) << 100, 0, 0, 0, 100, 0, 0, 0, 100;
	damping.bottomRightCorner(3, 3) << 25, 0, 0, 0, 25, 0, 0, 0, 8;
	//ToDo: Inertia matrix should be specified as multiple of physical inertia
	Eigen::Matrix<double, 6, 6> inertia;           
	inertia.topLeftCorner(3, 3) << 200, 0, 0, 0, 200, 0, 0, 0, 200;
	inertia.bottomRightCorner(3, 3) << 50, 0, 0, 0, 50, 0, 0, 0, 50;
	Eigen::Matrix<double, 6, 6> bubble_stiffness;  
	bubble_stiffness.topLeftCorner(3, 3) << 0, 0, 0, 0, 0, 0, 0, 0, 0;
	bubble_stiffness.bottomRightCorner(3, 3) << 0, 0, 0, 0, 0, 0, 0, 0, 0;
	Eigen::Matrix<double, 6, 6> bubble_damping;   
	bubble_damping.topLeftCorner(3, 3) << 0, 0, 0, 0, 0, 0, 0, 0, 0;
	bubble_damping.bottomRightCorner(3, 3) << 0, 0, 0, 0, 0, 0, 0, 0, 0;
	setParameters(stiffness, damping, inertia,
	              bubble_stiffness, bubble_damping);

	free_float = false;

}

void
HoldThis::performAction(TaskPlanner &task_planner, ros::Publisher &goal_publisher, ros::Publisher &impedance_publisher,
                        ros::Publisher &is_task_finished_publisher) {
	// stop at current position
	if (this->free_float){
		construct_impedance_message(this->impedance_params * 0.0);
		impedance_publisher.publish(this->compliance_update);
		task_planner.stop(&goal_publisher);
	}
	else {
		construct_impedance_message(this->impedance_params);
		impedance_publisher.publish(this->compliance_update);
		task_planner.stop(&goal_publisher);
	}

	this->free_float = false;
	//wait for goal pose and handle it
	ROS_INFO("Stopped, waiting for goal ");
	this->hasGrasped = false;
	while(!this->hasGrasped){
		ros::spinOnce();
		ros::Duration(0.1).sleep();
	}
	//finished waiting now either be passive and move on a plane/line or to goal
	//TODO: add publihing of goal pose reached here
	this->hasGrasped = false;
	//go back to hand
	ROS_INFO("finished waiting ");
	//now construct more different impedances to be safe in handover
	ImpedanceMatrices post_grasp_impedance;
	post_grasp_impedance.spring_stiffness = IDENTITY * 250;
	post_grasp_impedance.spring_stiffness.bottomRightCorner(3, 3) << 40, 0, 0, 0, 40, 0, 0, 0, 15;
	post_grasp_impedance.damping = IDENTITY * 35;
	post_grasp_impedance.damping.bottomRightCorner(3, 3) << 18, 0, 0, 0, 18, 0, 0, 0, 6;
	post_grasp_impedance.repulsion_stiffness = IDENTITY * 0;
	post_grasp_impedance.repulsion_damping = IDENTITY * 0;
	construct_impedance_message(post_grasp_impedance);
	impedance_publisher.publish(this->compliance_update);
	ros::Duration(1.0).sleep(); //waiting for impedances to converge to new value
	ROS_INFO("going from HOLD THIS to GOAL ");
	task_planner.primitive_move(this->goal_pose_.head(3), this->goal_pose_.tail(3), &goal_publisher, 0.02, ""); //higher tolerance for handover
	ros::Duration(1.0).sleep();
	task_planner.open_gripper();
	std_msgs::Bool done_msg; done_msg.data = false;
	is_task_finished_publisher.publish(done_msg);
	ros::Duration(0.2).sleep();
}

// Default constructor for TakeThis
TakeThis::TakeThis() : ActionPrimitive() {
	// Custom values for TakeThis
	//Stiffness
	Eigen::Matrix<double, 6, 6> stiffness;  
	stiffness =  Eigen::MatrixXd::Identity(6,6);
	stiffness.topLeftCorner(3, 3) << 300, 0, 0, 0, 300, 0, 0, 0, 300;
	stiffness.bottomRightCorner(3, 3) << 50, 0, 0, 0, 50, 0, 0, 0, 12;
	//Damping
	Eigen::Matrix<double, 6, 6> damping;           
	damping =  Eigen::MatrixXd::Identity(6,6);
	damping.topLeftCorner(3, 3) << 65, 0, 0, 0, 65, 0, 0, 0, 65;
	damping.bottomRightCorner(3, 3) << 15, 0, 0, 0, 15, 0, 0, 0, 7;
	Eigen::Matrix<double, 6, 6> inertia;           
	//ToDo: Inerta matrix should be specified as multiple of physical inertias
	inertia.topLeftCorner(3, 3) << 200, 0, 0, 0, 200, 0, 0, 0, 200;
	inertia.bottomRightCorner(3, 3) << 50, 0, 0, 0, 50, 0, 0, 0, 50;
	Eigen::Matrix<double, 6, 6> bubble_stiffness;  
	bubble_stiffness.topLeftCorner(3, 3) << 110, 0, 0, 0, 110, 0, 0, 0, 90;
	bubble_stiffness.bottomRightCorner(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	Eigen::Matrix<double, 6, 6> bubble_damping;   
	bubble_damping.topLeftCorner(3, 3) << 20, 0, 0, 0, 20, 0, 0, 0, 18;
	bubble_damping.bottomRightCorner(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	setParameters(stiffness, damping, inertia,
	              bubble_stiffness, bubble_damping);

	// Additional custom initialization for GetMe if needed
}

void
TakeThis::performAction(TaskPlanner &task_planner, ros::Publisher &goal_publisher, ros::Publisher &impedance_publisher,
                        ros::Publisher &is_task_finished_publisher) {
	std::random_device dev;
	std::mt19937 rng(dev());
	std::uniform_int_distribution<std::mt19937::result_type> dist6(1,6); // distribution in range [1, 6]
	int rand_number = dist6(rng);
	//
	ImpedanceMatrices post_grasp_impedance = this->impedance_params;
	Eigen::Vector3d grasp_offset; grasp_offset << -0.05, 0, 0.08;
	// Implementation of performAction for TakeThis
	// Custom logic for TakeThis
	//go to object/hand
	construct_impedance_message(this->impedance_params);
	impedance_publisher.publish(this->compliance_update);
	// execute handover ( go to hand)
	task_planner.primitive_move((this->getObjectPose().head(3)) + grasp_offset, this->getObjectPose().tail(3), &goal_publisher, 0.08, "grasp");
	task_planner.primitive_move(this->getObjectPose().head(3), this->getObjectPose().tail(3), &goal_publisher, 0.04, "grasp"); //higher tolerance in handover
	//do not move
	//lower repulsive stiffness during handover
	post_grasp_impedance.repulsion_stiffness = impedance_params.repulsion_stiffness/1000.0;
	post_grasp_impedance.repulsion_damping = impedance_params.repulsion_damping * 0.25;
	construct_impedance_message(post_grasp_impedance);
	impedance_publisher.publish(this->compliance_update);
	task_planner.stop(&goal_publisher);
	ros::Duration(0.1).sleep();
	//now is ready -> open gripper
	task_planner.open_gripper();
	//wait for human input, i.e. forcing to close gripper
	ROS_INFO("Waiting for forcing");
	std_msgs::Bool done_msg; done_msg.data = true; //false for goal pose wait, true for force wait
	is_task_finished_publisher.publish(done_msg);
	ros::Time start_time = ros::Time::now();
	double measured_force = 0.0;
	while(measured_force < 5.5){
		double elapsed_time = (ros::Time::now() - start_time).toSec();
		measured_force = task_planner.F_ext.norm();
		std::cout << "Fext is " << measured_force << "and random number is" << rand_number <<std::endl;
		ros::spinOnce();
		ros::Duration(0.2).sleep();
		if(elapsed_time > 4.0){
			break;
		}
	}
	//grasp object out of hand
	std::cout << "finished while loop" << std::endl;
	task_planner.grasp_object();
	//publish that it finished grasping
	done_msg.data = false; //false for goal pose wait, true for force wait
	is_task_finished_publisher.publish(done_msg);
	//now wait for goal posiiton
	ROS_INFO("Waiting for goal position");
	while(!this->hasGrasped){
		ros::spinOnce();
		ros::Duration(0.1).sleep();
	}
	ROS_INFO("ended wait time");
	//TODO:: publish "goal reached"
	this->hasGrasped = false;
	// now that object is grasped increase safety bubble stiffness after a transitory period
	post_grasp_impedance.repulsion_stiffness = impedance_params.repulsion_stiffness * 2;
	post_grasp_impedance.repulsion_damping = impedance_params.repulsion_damping * 1.414; //sqrt of 2
	construct_impedance_message(post_grasp_impedance);
	ros::Duration(1.5).sleep();
	//go back to delivery pose
	impedance_publisher.publish(this->compliance_update);
	ROS_INFO("published impedance message");
	//no grasp offset needed
	task_planner.primitive_move(this->goal_pose_.head(3), this->goal_pose_.tail(3), &goal_publisher, 0.03);
	ros::Duration(0.1).sleep();
	task_planner.open_gripper();

}

// Default constructor for AvoidMe
AvoidMe::AvoidMe() : ActionPrimitive() {
	// Custom values for AvoidMe
	Eigen::Matrix<double, 6, 6> stiffness;  
	stiffness.topLeftCorner(3, 3) << 450, 0, 0, 0, 450, 0, 0, 0, 450;
	stiffness.bottomRightCorner(3, 3) << 100, 0, 0, 0, 100, 0, 0, 0, 12;
	Eigen::Matrix<double, 6, 6> damping;           
	damping.topLeftCorner(3, 3) << 45, 0, 0, 0, 45, 0, 0, 0, 45;
	damping.bottomRightCorner(3, 3) << 20, 0, 0, 0, 20, 0, 0, 0, 7;
	Eigen::Matrix<double, 6, 6> inertia;           
	//ToDo: Inerta matrix should be specified as multiple of physical inertia
	inertia.topLeftCorner(3, 3) << 200, 0, 0, 0, 200, 0, 0, 0, 200;
	inertia.bottomRightCorner(3, 3) << 50, 0, 0, 0, 50, 0, 0, 0, 50;
	//
	Eigen::Matrix<double, 6, 6> bubble_stiffness;  
	bubble_stiffness.topLeftCorner(3, 3) << 500, 0, 0, 0, 500, 0, 0, 0, 500;
	bubble_stiffness.bottomRightCorner(3, 3) << 50, 0, 0, 0, 50, 0, 0, 0, 50;
	Eigen::Matrix<double, 6, 6> bubble_damping;   
	bubble_damping.topLeftCorner(3, 3) << 20, 0, 0, 0, 20, 0, 0, 0, 20;
	bubble_damping.bottomRightCorner(3, 3) << 3, 0, 0, 0, 3, 0, 0, 0, 1;
	setParameters(stiffness, damping, inertia,
	              bubble_stiffness, bubble_damping);
	
}

void
AvoidMe::performAction(TaskPlanner &task_planner, ros::Publisher &goal_publisher, ros::Publisher &impedance_publisher,
                       ros::Publisher &is_task_finished_publisher) {
	// Implementation of performAction for AvoidMe
	// Custom logic for AvoidMe
	construct_impedance_message(this->impedance_params);
	impedance_publisher.publish(this->compliance_update);
	task_planner.primitive_move(this->goal_pose_.head(3), this->goal_pose_.tail(3), &goal_publisher, 0.02); //just go to goal
}

