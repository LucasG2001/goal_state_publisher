//
// Created by lucas on 10.04.24.
//
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <fstream>
#include <iostream>
#include <chrono>
#include <custom_msgs/action_primitive_message.h>
#include <Eigen/Dense>
#include <std_msgs/Float64.h>
#include <franka_msgs/FrankaState.h>
#include <sys/stat.h> // For mkdir

using namespace std::chrono;

class DataLogger {
public:
	std::string timestamp;
	std::string subfolder;

	DataLogger(ros::NodeHandle & nh) {
		reference_pose_sub = nh.subscribe("/cartesian_impedance_controller/reference_pose", 1, &DataLogger::referencePoseCallback, this);
		ee_pose_sub = nh.subscribe("/franka_state_controller/franka_states", 1, &DataLogger::eePoseCallback, this);
		action_primitive_sub = nh.subscribe("/action_primitive", 1, &DataLogger::actionPrimitiveCallback, this);
		nearest_point_sub = nh.subscribe("/nearest_distance", 1, &DataLogger::nearestPointCallback, this);
		hand_pose_sub = nh.subscribe("/cartesian_impedance_controller/right_hand", 1, &DataLogger::handPoseCallback, this);
		// Set up timer for continuous logging
		std::cout << " created subscribers " << std::endl;
	}

	void referencePoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
		reference_pose_msg = *msg;
	}

	void eePoseCallback(const franka_msgs::FrankaStateConstPtr msg) {
		ee_position.x() = msg->O_T_EE[12]; // O_T_EE is a float[16] array in COLUMN MAJOR format!
		ee_position.y() = msg->O_T_EE[13];
		ee_position.z() = msg->O_T_EE[14];
		Eigen::Affine3d transform(Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::ColMajor>>(msg->O_T_EE.data()));
		ee_orientation = transform.rotation();
	}

	void actionPrimitiveCallback(const custom_msgs::action_primitive_messageConstPtr& msg) {
		action_primitive_msg = *msg;
	}

	void nearestPointCallback(const std_msgs::Float64ConstPtr & msg) {
		nearest_distance = msg->data;
	}

	void handPoseCallback(const geometry_msgs::Pose::ConstPtr& msg) {
		hand_pose_msg = *msg;
	}

	void startLogging(const std::string& participant_id) {
		//create date timestamp
		auto now = std::chrono::system_clock::now();
		std::time_t currentTime = std::chrono::system_clock::to_time_t(now);
		std::tm* timeinfo = std::localtime(&currentTime);
		std::ostringstream oss;
		oss << std::put_time(timeinfo, "%Y%m%d%H%M");
		timestamp = oss.str();
		std::string participant_folder = "/home/lucas/catkin_ws/src/goal_state_publisher/src/logging/data/" + participant_id + "/";
		if (mkdir(participant_folder.c_str(), 0777) == -1) {
			std::cerr << "Error creating directory: " << participant_folder << std::endl;
			std::cerr << "Error message: " << strerror(errno) << std::endl;
			// Handle error if needed
		}
		// Construct the path to the subfolder
		subfolder = participant_folder + "/" + timestamp + "/";
		// Create the subfolder using mkdir
		if (mkdir(subfolder.c_str(), 0777) == -1) {
			std::cerr << "Error creating directory: " << subfolder << std::endl;
			std::cerr << "Error message: " << strerror(errno) << std::endl;
			// Handle error if needed
		}

		reference_pose_file.open(subfolder + "reference_pose_" + timestamp + ".csv");
		reference_pose_file << "x, y, z, ry, ry, rz, w" << std::endl;
		reference_pose_file.close();
		ee_pose_file.open(subfolder + "ee_pose_" + timestamp + ".csv");
		ee_pose_file << "x, y, z, ry, ry, rz, w" << std::endl;
		ee_pose_file.close(),
		action_primitive_file.open(subfolder + "action_primitive_" + timestamp + ".csv");
		action_primitive_file.close();
		action_primitive_file << "primitive_type" << std::endl;
		nearest_distance_file.open(subfolder + "nearest_point_" + timestamp + ".csv");
		nearest_distance_file << "nearest_distance" << std::endl;
		nearest_distance_file.close();
		hand_pose_file.open(subfolder + "hand_pose_" + timestamp + ".csv");
		hand_pose_file << "x, y, z, ry, ry, rz, w" << std::endl;
														hand_pose_file.close();
		std::cout << "started logging and opened files" << std::endl;
		//start logging time
		start_time = system_clock::now();
	}

	void stopLogging() {
		end_time = system_clock::now();
		auto total_time = duration_cast<seconds>(end_time - start_time).count();
		std::ofstream total_time_file("total_time.csv");
		total_time_file << "Total time: " << total_time << " seconds" << std::endl;
		reference_pose_file.close();
		ee_pose_file.close();
		action_primitive_file.close();
		nearest_distance_file.close();
		hand_pose_file.close();
	}

	void logData() {
		reference_pose_file.open(subfolder + "reference_pose_" + timestamp + ".csv", std::ios::app);
		ee_pose_file.open(subfolder + "ee_pose_" + timestamp + ".csv", std::ios::app);
		action_primitive_file.open(subfolder + "action_primitive_" + timestamp + ".csv", std::ios::app);
		nearest_distance_file.open(subfolder + "nearest_point_" + timestamp + ".csv", std::ios::app);
		hand_pose_file.open(subfolder + "hand_pose_" + timestamp + ".csv", std::ios::app);

		if (reference_pose_file.is_open()) {
			std::cout << "true" << std::endl;
			reference_pose_file << reference_pose_msg.pose.position.x << ","
			                    << reference_pose_msg.pose.position.y << ","
			                    << reference_pose_msg.pose.position.z << ","
			                    << reference_pose_msg.pose.orientation.x << ","
			                    << reference_pose_msg.pose.orientation.y << ","
			                    << reference_pose_msg.pose.orientation.z << ","
			                    << reference_pose_msg.pose.orientation.w << std::endl;
		}

		if (ee_pose_file.is_open()) {
			ee_pose_file << ee_position.x() << ","
			             << ee_position.y() << ","
			             << ee_position.z() << ","
			             << ee_orientation.x() << ","
			             << ee_orientation.y() << ","
			             << ee_orientation.z() << ","
			             << ee_orientation.w() << std::endl;
		}

		if (action_primitive_file.is_open()) {
			action_primitive_file << action_primitive_msg.task_type << std::endl;
		}

		if (nearest_distance_file.is_open()) {
			nearest_distance_file << nearest_distance << std::endl;
		}

		if (hand_pose_file.is_open()) {
			hand_pose_file << hand_pose_msg.position.x << ","
			               << hand_pose_msg.position.y << ","
			               << hand_pose_msg.position.z << ","
			               << hand_pose_msg.orientation.x << ","
			               << hand_pose_msg.orientation.y << ","
			               << hand_pose_msg.orientation.z << ","
			               << hand_pose_msg.orientation.w << std::endl;
		}

		reference_pose_file.close();
		ee_pose_file.close();
		action_primitive_file.close();
		nearest_distance_file.close();
		hand_pose_file.close();
	}

private:
	ros::NodeHandle nh;
	ros::Subscriber reference_pose_sub;
	ros::Subscriber ee_pose_sub;
	ros::Subscriber action_primitive_sub;
	ros::Subscriber nearest_point_sub;
	ros::Subscriber hand_pose_sub;
	ros::Subscriber start_log_sub;
	ros::Subscriber stop_log_sub;

	geometry_msgs::PoseStamped reference_pose_msg;
	Eigen::Vector3d ee_position;
	Eigen::Quaterniond  ee_orientation;
	custom_msgs::action_primitive_message action_primitive_msg;
	double nearest_distance;
	geometry_msgs::Pose hand_pose_msg;

	time_point<system_clock> start_time;
	time_point<system_clock> end_time;

	std::ofstream reference_pose_file;
	std::ofstream ee_pose_file;
	std::ofstream action_primitive_file;
	std::ofstream nearest_distance_file;
	std::ofstream hand_pose_file;
};

bool checkUserInput(char& input) {
	if (std::cin.peek() != EOF) {
		std::cin >> input;
		return true;
	}
	return false;
}

int main(int argc, char** argv) {
	if (argc != 2) {
		ROS_ERROR("Usage: No participant ID!");
		return 1; // Exit with error
	}
	// Save the participant id in a local variable
	std::string participant_id = argv[1];
	ros::init(argc, argv, "logger_node");
	ros::AsyncSpinner spinner(2);
	spinner.start();
	ros::NodeHandle nh;
	ros::Rate rate(20); // 30 Hz
	//create data logger object
	DataLogger logger(nh);

	// Inform the user to press 's' to start logging
	std::cout << "Press 's' to start logging." << std::endl;
	// Wait for the user to input 's' to start logging
	char input;
	std::cin >> input;
	if (input != 's') {
		std::cout << "Invalid input. Exiting program." << std::endl;
		return 1;
	}
	// Inform the user that logging has started
	std::cout << "Logging started. Press 'q' to stop." << std::endl;

	//main loop
	bool stopLogging = false;
	logger.startLogging(participant_id);
	while (ros::ok() && !stopLogging) {
			// Log data if logging has started
			logger.logData();
			// Check for user input to stop logging
			/*
			if (checkUserInput(input) && input == 'q') {
				stopLogging = true;
				logger.stopLogging();
			}
			 */

			// Inform the user about the current status
			std::cout << "Logging data..." << std::endl;
			rate.sleep();
		}

	// Inform the user that logging has stopped
	std::cout << "Logging stopped." << std::endl;

	return 0;
}
