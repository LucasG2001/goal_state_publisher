#include "ros/ros.h"
#include "utility.h"
#include <iostream>
#include <vector>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <geometry_msgs/Pose.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int8MultiArray.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>

//we get pose_msgs
//we publish multiarray int8
//publish jointpose
void singleCallback(const geometry_msgs::Pose::ConstPtr& p, moveit::planning_interface::MoveGroupInterface* move_group_ptr, ros::NodeHandle& nh)
{
    //get: pose
    //send: true/false (boolMSg)
    //send: JointStateMsg (q 7x1)

    ros::Publisher reachability_pub = nh.advertise<std_msgs::Bool>("reachability_single_pose", 10);
    ros::Publisher q_pub = nh.advertise<sensor_msgs::JointState>("joint_angles", 10);
    ros::Publisher equilibrium_pose_reference = nh.advertise<visualization_msgs::InteractiveMarkerFeedback>("/equilibrium_pose_marker/feedback", 10); //cartesian impedance controller

    ROS_INFO("got new reference pose");
    // Get a pointer to the current robot state
    moveit::core::RobotState start_state(*move_group_ptr->getCurrentState());
    start_state.setToDefaultValues();
    // Update the planning scene with the current robot state
    move_group_ptr->setStartState(start_state);

    geometry_msgs::Pose target_pose;
    target_pose.orientation = p->orientation;
    target_pose.position = p->position;
    move_group_ptr->setPoseTarget(target_pose);

    std_msgs::Bool is_reachable;
    sensor_msgs::JointState q;
    q.header.stamp = ros::Time::now();
    q.name = move_group_ptr->getJointNames();
    q.position.resize(q.name.size());
    std::cout << "size of q is  " << q.position.size() << std::endl;
    std::cout << "Joint names are  " << std::endl;
    for (int i = 0; i <q.position.size(); i++){
        std::cout << q.name[i] << std::endl;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_ptr->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success) {
        ROS_INFO("Plan was computed");
        // Get the joint trajectory from the plan
        const auto& joint_values = plan.trajectory_.joint_trajectory.points.back().positions;
        // Update the joint state message with the last joint positions
        std::copy(joint_values.begin(), joint_values.end(), q.position.begin());
        is_reachable.data = true;
        ROS_INFO("set reachable");
        q_pub.publish(q);
    }
    else{
        ROS_INFO("No Plan found");
        is_reachable.data = false;
    }

    reachability_pub.publish(is_reachable);
    ROS_INFO("published reachable");
    //equilibirum pose for cartesiam impedance controller
    visualization_msgs::InteractiveMarkerFeedback reference_pose;
    reference_pose.pose = target_pose;
    reference_pose.mouse_point_valid = true;
    reference_pose.client_id = "/rviz/InteractiveMarkers";
    reference_pose.event_type = 1;
    reference_pose.marker_name = "equilibrium_pose";
    reference_pose.header.frame_id = "panda_link0";
    equilibrium_pose_reference.publish(reference_pose);
    ROS_INFO_STREAM("published pose " << reference_pose.pose);
    ROS_INFO("published reference pose");
}

 //hier nur zurückschicken: letzter Zustand bevor unfeasible
    //get: POseArrayMsg
    //get: length of pose array int8
    //send: int8MultiArrayMsg (1/0 feasible/unfeasible)
    //send: JointPoseMsg q für den letzten feasible Zustand (wenn es 2 gibt für den ersten)
    //update state of the robot if new point is feasible, if unfeasible is encountered plan for the next from last reachable

void fullCallback(const geometry_msgs::PoseArray::ConstPtr& pose_array_msg, moveit::planning_interface::MoveGroupInterface* move_group_ptr, ros::NodeHandle& nh)
{
    ROS_INFO("Got Full Path Message");
    //create publishers with node handle
    ros::Publisher q_pub = nh.advertise<sensor_msgs::JointState>("joint_angles", 10);
    ros::Publisher reachability_pub = nh.advertise<std_msgs::Int8MultiArray>("reachability_full_path", 10);
    // Create a JointState message to publish the joint positions
    sensor_msgs::JointState joint_state_msg;
    joint_state_msg.header.stamp = ros::Time::now();
    joint_state_msg.name = move_group_ptr->getJointNames();
    joint_state_msg.position.resize(joint_state_msg.name.size());
    std::cout << "size of q is  " << joint_state_msg.position.size() << std::endl;

    std_msgs::Int8MultiArray reachable_msg;
    reachable_msg.data.resize(pose_array_msg->poses.size());

    moveit::core::RobotState start_state(*move_group_ptr->getCurrentState());
    start_state.setToDefaultValues();
    // Update the planning scene with the current robot state
    move_group_ptr->setStartState(start_state);

    // Loop through all poses in the PoseArray message
    int i = 0;
    bool isLastReachable = false;
    for (const auto& pose : pose_array_msg->poses)
    {
        // Set the target pose for the planning scene
        move_group_ptr->setPoseTarget(pose);
        // Compute a plan to reach the target pose
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_ptr->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (success)
        {
            ROS_INFO("Found solution");
            if (!isLastReachable){
                // Get the last joint positions from the plan
                const auto& joint_values = plan.trajectory_.joint_trajectory.points.back().positions;
                // Update the joint state message with the last joint positions
                std::copy(joint_values.begin(), joint_values.end(), joint_state_msg.position.begin());
            }
            reachable_msg.data[i] = 1;
            start_state.setVariableValues(joint_state_msg);
        }
        else
        {   // Publish a joint state message full of -1 if the plan fails
            ROS_INFO("Failed to find path");
            isLastReachable = true;
            reachable_msg.data[i] = 0;
        }
        i++;
    } //for loop
    // Publish the joint state message
    q_pub.publish(joint_state_msg);
    reachability_pub.publish(reachable_msg);
}


int main(int argc, char** argv) {
    // Initialize ROS node
    ros::init(argc, argv, "unity_handler");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);
    spinner.start();

    auto* move_group = new moveit::planning_interface::MoveGroupInterface("fr3_arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    move_group->setPlanningTime(1.5);
    ros::Subscriber single_point = nh.subscribe<geometry_msgs::Pose>("check_single_pose", 1000, boost::bind(&singleCallback, _1, move_group, nh));
    ros::Subscriber full_path = nh.subscribe<geometry_msgs::PoseArray>("check_full_path", 1000, boost::bind(&fullCallback, _1, move_group, nh));

    ros::waitForShutdown();

    return 0;
}