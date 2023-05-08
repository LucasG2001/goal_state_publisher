//
// Created by lucas on 11.04.23.
//
#include <ros/ros.h>
#include <franka_hw/franka_hw.h>
#include <Eigen/Dense>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <sensor_msgs/JointState.h>
#include<franka_msgs/FrankaState.h>

void callback(const franka_msgs::FrankaState::ConstPtr& msg){

}
int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "franka_gazebo_interface");
    ros::NodeHandle nh;
    // Subscribe to the /joint_states topic
    franka_hw::ModelBase model;
    franka::RobotState robot_state;
    // Create a FrankaModelHandle object
    franka_hw::FrankaModelInterface interface;
    interface.
    franka_hw::FrankaModelHandle model_handle("panda", );


    // Loop rate (in Hz)
    ros::Rate loop_rate(100);
    ros::Subscriber state_sub = nh.subscribe("panda_pose_reference", 1, callback);
    while (ros::ok()) {
        // Get the Mass Matrix
        std::array<double, 49> mass_v = model_handle.getMass();
        // Transform the vectorized matrix into Eigen::Matrix3d
        Eigen::Matrix<double, 7, 7, Eigen::ColMajor> M = Eigen::Map<Eigen::Matrix<double, 7, 7>>(mass_v.data());

        // Get the gravitational terms
        std::array<double, 7> gravity_v = model_handle.getGravity();
        Eigen::Matrix<double, 7, 1, Eigen::ColMajor> g = Eigen::Map<Eigen::Matrix<double, 7, 1>>(gravity_v.data());

        // Print the Mass Matrix
        ROS_INFO_STREAM("Mass Matrix:");
        for (int i = 0; i < M.rows(); ++i) {
            std::stringstream ss;
            ss << "Row " << i << ": ";
            for (int j = 0; j < M.cols(); ++j) {
                ss << M(i, j) << " ";
            }
            ROS_INFO_STREAM(ss.str());
        }

        // Print the gravitational terms
        ROS_INFO_STREAM("Gravitational Terms:");
        std::stringstream ss;
        for (int i = 0; i < g.rows(); ++i) {
            ss << g(i) << " ";
        }
        ROS_INFO_STREAM(ss.str());

        // Spin once to process ROS callbacks
        ros::spinOnce();

        // Sleep to achieve the desired loop rate
        loop_rate.sleep();
    }

    return 0;
}
