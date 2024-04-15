//
// Created by lucas on 27.09.23.
//
// Default constructor
#include <TaskPlanner.h>

//Default Constructor
TaskPlanner::TaskPlanner() :
		gripper_grasp_client("franka_gripper/grasp", true),
		gripper_move_client("franka_gripper/move", true),
		gripper_stop_client("franka_gripper/stop", true)
{
	// Wait for the action servers to start up
	//gripper_grasp_client.waitForServer();
	//gripper_move_client.waitForServer();
	//gripper_stop_client.waitForServer();
} // end constructor

/**
void TaskPlanner::move(std::vector<double> position, std::vector<double> orientation, ros::Publisher* goal_pose_publisher, double tol, std::string header_info){
    double trajectory_intervals = 0.15;
    geometry_msgs::PoseStamped target_pose;
    target_pose.header.frame_id = "waypoint";
    Eigen::Map<Eigen::Vector3d> goal_position(position.data());
    Eigen::Map<Eigen::Vector3d> goal_orientation(position.data());
    Eigen::Matrix<double, 6, 1> waypoint_number;
    waypoint_number.head(3) = ((goal_position-global_ee_position)/trajectory_intervals).array().abs().ceil(); //position with 0.1 spaced waypoints
	//EXCEPTION HANDLING Waypoint number = 0 (is at goal):
	waypoint_number.head(3) = waypoint_number.head(3).cwiseMax(Eigen::Vector3d::Ones());
	waypoint_number.tail(3).setZero();
	//END of EXCEPTION
    ROS_INFO_STREAM(" waypoints are distributed as " << waypoint_number);
    waypoint_number.tail(3).setZero();
    //at the moment only use position
    //waypoint_number.head(3) = (goal_position/0.785).array().abs().ceil(); //orientation with 0.785 spaced waypoints
    Eigen::Vector3d step_size = (goal_position - global_ee_position).array() / waypoint_number.head(3).array();
    ROS_INFO_STREAM("step sizes are " << step_size);
    Eigen::Vector3d start_position = global_ee_position;
    Eigen::Vector3d lower_bound = start_position.cwiseMin(goal_position);
    Eigen::Vector3d upper_bound = start_position.cwiseMax(goal_position);
    ROS_INFO_STREAM("going from " << start_position.transpose() << " to " << " " << goal_position.transpose());
    tol *= 3; //double tolerance for middle waypoints
    double goal_time = 1;
    // publish end goal once to delete the object in the planning scene, then proceed by quickly publishing the first waypoint
    for (int i = 1; i < waypoint_number.maxCoeff() + 1; i++){
        if (i == waypoint_number.maxCoeff()){
            tol *= 0.25;//get back tolerance for last waypoint
            target_pose.header.frame_id = "goal";
            goal_time = 10;
        }
        Eigen::Vector3d reference_pos = (start_position + i * step_size); //just add up correct later
        reference_pos = reference_pos.cwiseMax(lower_bound).cwiseMin(upper_bound);
        std::vector<double> reference_for_msg = {reference_pos(0,0), reference_pos(1,0), reference_pos(2,0)};
        ROS_INFO_STREAM("waypoint " << i << " is " << reference_pos);
        //generate a trajectory with waypoints
        target_pose.pose = createGoalPose(reference_for_msg, orientation);
        goal_pose_publisher->publish(target_pose);
        Eigen::Map<Eigen::Vector3d> goal_vector(position.data());
        double elapsed_time = 0.0;
        double start_time = ros::Time::now().toSec();
        while((goal_vector-global_ee_position).norm() > tol && elapsed_time < goal_time){
            double current_time = ros::Time::now().toSec();
            elapsed_time = current_time - start_time;
            ros::Duration(0.1).sleep();
        }
    }//for loop

}
**/

void TaskPlanner::move(std::vector<double> position, std::vector<double> orientation, ros::Publisher* goal_pose_publisher, double tol, std::string header_info){
	double goal_time = 6.5;
    geometry_msgs::PoseStamped target_pose;
    Eigen::Vector3d start_position = global_ee_position;

	Eigen::Map<Eigen::Vector3d> goal_vector(position.data());
	target_pose.pose = createGoalPose(position, orientation);
	goal_pose_publisher->publish(target_pose);


	double elapsed_time = 0.0;
	double start_time = ros::Time::now().toSec();
	while((goal_vector-global_ee_position).norm() > tol && elapsed_time < goal_time){
		double current_time = ros::Time::now().toSec();
		elapsed_time = current_time - start_time;
		ros::Duration(0.1).sleep();
	}
}//for loop


// with Eigen
void TaskPlanner::primitive_move(Eigen::Matrix<double, 3, 1>goal_position, Eigen::Matrix<double, 3, 1> goal_orientation, ros::Publisher* goal_pose_publisher, double tol, std::string header_info) const{
	double goal_time = 3.5;
    geometry_msgs::PoseStamped target_pose;

	std::vector<double> reference_for_msg = {goal_position(0,0), goal_position(1,0), goal_position(2,0)};
	target_pose.pose = createGoalPose(reference_for_msg, {goal_orientation(0,0), goal_orientation(1,0), goal_orientation(2,0)});
	target_pose.header.frame_id = header_info;
	goal_pose_publisher->publish(target_pose);

	double elapsed_time = 0.0;
	double start_time = ros::Time::now().toSec();
	while((goal_position-global_ee_position).norm() > tol && elapsed_time < goal_time && (goal_orientation-global_ee_euler_angles).norm() > 0.075 ){
		double current_time = ros::Time::now().toSec();
		elapsed_time = current_time - start_time;
		ros::Duration(0.1).sleep();
	}
}//for loop


void TaskPlanner::execute_action(Eigen::Matrix<double, 3, 1>goal_position, Eigen::Matrix<double, 3, 1> goal_orientation, ros::Publisher* goal_pose_publisher, double tol) const{
	//ToDo: Make the first part of the code as callback , that sets the local goal according to the new one. Watch out for threading and waypoints
	ros::Rate loop_rate(10);
	double trajectory_intervals = 0.15;
	geometry_msgs::PoseStamped target_pose;
	Eigen::Matrix<double, 6, 1> waypoint_number;
	waypoint_number.head(3) = ((goal_position-global_ee_position)/trajectory_intervals).array().abs().ceil(); //position with 0.1 spaced waypoints
	//EXCEPTION HANDLING Waypoint number = 0 (is at goal):
	waypoint_number.head(3) = waypoint_number.head(3).cwiseMax(Eigen::Vector3d::Ones());
	waypoint_number.tail(3).setZero();
	//END of EXCEPTION
	//at the moment only use position
	Eigen::Vector3d step_size = (goal_position - global_ee_position).array() / waypoint_number.head(3).array();
	//ROS_INFO_STREAM("step sizes are " << step_size);
	Eigen::Vector3d start_position = global_ee_position;
	Eigen::Vector3d lower_bound = start_position.cwiseMin(goal_position);
	Eigen::Vector3d upper_bound = start_position.cwiseMax(goal_position);
	ROS_INFO_STREAM("going from " << start_position.transpose() << " to " << " " << goal_position.transpose());
	tol *= 5; //double tolerance for middle waypoints
	double goal_time = 0.7;
	// publish end goal once to delete the object in the planning scene, then proceed by quickly publishing the first waypoint
	for (int i = 1; i < waypoint_number.maxCoeff() + 1; i++){
		Eigen::Vector3d reference_pos = (start_position + i * step_size); //just add up correct later
		reference_pos = reference_pos.cwiseMax(lower_bound).cwiseMin(upper_bound);
		if (i == waypoint_number.maxCoeff()){
			tol *= 0.2;//get back tolerance for last waypoint
			goal_time = 2;
			target_pose.header.frame_id = "goal"; // will give the controller w_des = 0
		} else{ reference_pos.z() += 0.08; target_pose.header.frame_id = "waypoint"; } //for grasping and will give controller w_des = 0.07
		std::vector<double> reference_for_msg = {reference_pos(0,0), reference_pos(1,0), reference_pos(2,0)};
		target_pose.pose = createGoalPose(reference_for_msg, {goal_orientation(0,0), goal_orientation(1,0), goal_orientation(2,0)});
		//publish target pose
		goal_pose_publisher->publish(target_pose);
		double elapsed_time = 0.0;
		double start_time = ros::Time::now().toSec();
		while((goal_position-global_ee_position).norm() > tol && elapsed_time < goal_time){
			double current_time = ros::Time::now().toSec();
			elapsed_time = current_time - start_time;
			loop_rate.sleep();
		}
		ROS_INFO("Reached waypoint");
	}//for loop

}


void TaskPlanner::open_gripper(double speed, double width){
    franka_gripper::MoveGoal gripper_open;
    gripper_open.width = width;
    gripper_open.speed = speed;
    gripper_move_client.sendGoal(gripper_open);
    ros::Duration(0.01).sleep();
    bool finished_before_timeout = gripper_move_client.waitForResult(ros::Duration(10.0)); // Adjust the timeout as needed
    if (finished_before_timeout) {
        ROS_INFO("Action finished successfully!");
        // Handle successful completion here
    } else {
        ROS_WARN("Action did not finish before the timeout.");
        // Handle timeout or other error conditions here
    }
}

void TaskPlanner::grasp_object(double speed, double width, double force, double tol){
    franka_gripper::GraspGoal grasp;
    grasp.speed = speed;
    grasp.width = width;
    grasp.force = force;
    grasp.epsilon.inner = tol;
    grasp.epsilon.outer = tol;
    gripper_grasp_client.sendGoal(grasp);
    // Wait for the action to finish
    bool finished_before_timeout = gripper_grasp_client.waitForResult(ros::Duration(10.0)); // Adjust the timeout as needed

    if (finished_before_timeout) {
        ROS_INFO("Action finished successfully!");
        // Handle successful completion here
    } else {
        ROS_WARN("Action did not finish before the timeout.");
        // Handle timeout or other error conditions here
    }
    //move upward a little bit after grasp
    geometry_msgs::PoseStamped target_pose;
    target_pose.pose.position.x = global_ee_position.x();
    target_pose.pose.position.y = global_ee_position.y();
    target_pose.pose.position.z = global_ee_position.z() + 0.1;
    target_pose.pose.orientation.x = global_ee_orientation.x();
    target_pose.pose.orientation.y = global_ee_orientation.y();
    target_pose.pose.orientation.z = global_ee_orientation.z();
    target_pose.pose.orientation.w = global_ee_orientation.w();
	ROS_INFO(" publishing intermediate pose after grasp ");
    equilibrium_pose_pub.publish(target_pose);
    ros::Duration(1.0).sleep();

}

void TaskPlanner::ee_callback(const franka_msgs::FrankaStateConstPtr & msg){
    // for the time being we only need ee positions without orientations
    global_ee_position.x() = msg->O_T_EE[12]; // O_T_EE is a float[16] array in COLUMN MAJOR format!
    global_ee_position.y() = msg->O_T_EE[13];
    global_ee_position.z() = msg->O_T_EE[14];
    F_ext = Eigen::Map<const Eigen::Matrix<double, 6, 1>>((msg->O_F_ext_hat_K).data());
	Eigen::Affine3d transform(Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::ColMajor>>(msg->O_T_EE.data()));
	global_ee_orientation = transform.rotation();
	global_ee_euler_angles = global_ee_orientation.toRotationMatrix().eulerAngles(0, 1, 2);
}

void TaskPlanner::stop(ros::Publisher *goal_pose_publisher) const {
	//send actual pose as goal pose and clear integrator such that we dont move
	geometry_msgs::PoseStamped stop_goal;
	ROS_INFO("stopping");
	stop_goal.pose.position.x = global_ee_position.x();
	stop_goal.pose.position.y = global_ee_position.y();
	stop_goal.pose.position.z = global_ee_position.z();
	stop_goal.pose.orientation.x = global_ee_orientation.x();
	stop_goal.pose.orientation.y = global_ee_orientation.y();
	stop_goal.pose.orientation.z = global_ee_orientation.z();
	stop_goal.pose.orientation.w = global_ee_orientation.w();
	stop_goal.header.frame_id = "CLEAR";
	goal_pose_publisher->publish(stop_goal);

}


