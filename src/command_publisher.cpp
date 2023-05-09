//
// Created by lucas on 28.03.23.
//

#include "ros/ros.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "utility.h"
#include <iostream>
#include <vector>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <franka_msgs/SetLoad.h>
//watch out with these h-files. Only building with catkin build in the workspace will build everything correctly, even when not including these files
//these are for the IDE. If you can't find them in the referenced location, the cMake dependencies should make sure catkin can build and auto-generate the headers
#include "../../../devel/.private/goal_state_publisher/include/goal_state_publisher/GraspMsg.h"
#include "../../../devel/.private/goal_state_publisher/include/goal_state_publisher/MoveGripperMsg.h"
#include "../../../devel/.private/goal_state_publisher/include/goal_state_publisher/StopGripperMsg.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "command_publisher");
    ros::NodeHandle n;

    ros::AsyncSpinner spinner(6);
    spinner.start();

    ros::Publisher commander = n.advertise<geometry_msgs::Pose>("panda_pose_reference", 1);
    ros::Publisher grasp_publisher = n.advertise<goal_state_publisher::GraspMsg>("grasp_goal", 1);
    ros::Publisher gripper_move_publisher = n.advertise<goal_state_publisher::MoveGripperMsg>("gripper_move_goal", 1);
    ros::Publisher gripper_stop_publisher = n.advertise<goal_state_publisher::StopGripperMsg>("gripper_stop_goal", 1);

    ros::Rate loop_rate(0.1);

// Create vector to store input values
    std::vector<double> position(3);
    std::vector<double> orientation(3);
    int grip_action;
    goal_state_publisher::GraspMsg gripper_grasp;
    goal_state_publisher::MoveGripperMsg gripper_move;
    goal_state_publisher::StopGripperMsg gripper_stop;
    geometry_msgs::Pose target_pose;

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> force_client("force_action_controller/follow_joint_trajectory", true);
    force_client.waitForServer();
    // create a ROS service client for the SetLoad service
    ros::ServiceClient client = n.serviceClient<franka_msgs::SetLoad>("/franka_control/set_load");

    // create a SetLoad service message
    franka_msgs::SetLoad srv;
    boost::array<double, 9> inertia = {0.005, 0 ,0, 0, 0.005, 0, 0, 0, 0.005};
    // set the mass and inertia tensor of the object
    srv.request.mass = 0.3;
    srv.request.load_inertia = inertia;
    srv.request.F_x_center_load = {0, 0, 0.1}; //translation of object center of mass form flange frame in [m]

    control_msgs::FollowJointTrajectoryGoal force_command;
    force_command.trajectory.joint_names = {"0"};

    while (ros::ok())
    {

        // Prompt user to input three values
        std::cout << "Enter 0 (move, nothing else) | 1 (open gripper) | 2 (grasp) | 3 (stop) | 4 (apply force and move) | 5 (std move)";
        std::cin >> grip_action;

        switch(grip_action){
            case 0:
                std::cout << "Enter Reference Position: ";
                std::cin >> position[0] >> position[1] >> position[2];
                std::cout << "Enter Reference Orientation: ";
                std::cin >> orientation[0] >> orientation[1] >> orientation[2];
                target_pose = createGoalPose(position, orientation);
                commander.publish(target_pose);
                break;
            case 1:
                std::cout << "Enter reference Width and Speed (in that order)";
                std::cin >> gripper_move.width >> gripper_move.speed;
                gripper_move_publisher.publish(gripper_move);
                break;
            case 2:
                std::cout << "Enter reference Width, Speed, Force and Tolerance (in that order)";
                std::cin >> gripper_grasp.width >> gripper_grasp.speed >> gripper_grasp.force >> gripper_grasp.tolerance;
                grasp_publisher.publish(gripper_grasp);
                break;
            case 3:
                gripper_stop.isStop = true;
                gripper_stop_publisher.publish(gripper_stop);
                break;
            case 4:
                force_command.trajectory.points[0].velocities = {0, 0.1, 0, 0, 0, 0, 0};
                force_command.trajectory.points[0].effort = {0, 0, -5, 0, 0, 0, 0};
                force_client.sendGoal(force_command);
                if (client.call(srv)) {
                    ROS_INFO("Load parameters set successfully!");
                } else {
                    ROS_ERROR("Failed to set load parameters.");
                }
                break;
            case 5:
                gripper_move.speed = 1;
                gripper_move.width = 0.08;
                gripper_move_publisher.publish(gripper_move);
                ros::Duration(0.2).sleep();
                target_pose = createGoalPose({0.4, 0, 0.475}, {-M_PI/2, -M_PI/4, -M_PI/2});
                commander.publish(target_pose);
                break;

        }

        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}