#include "ros/ros.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <trajectory_msgs/JointTrajectory.h>
#include "utility.h"
#include <iostream>
#include <vector>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <geometry_msgs/Pose.h>
#include <std_msgs/Bool.h>

//we get pose_msgs
//we publish multiarray int8
//publish jointpose
void singleCallback(const geometry_msgs::PoseConstPtr p, moveit::planning_interface::MoveGroupInterface* move_group_ptr)
{
    //get: pose
    //send: true/false (boolMSg)
    //send: JointStateMsg (q 7x1)
    ROS_INFO("got new reference pose");
    // Get a pointer to the current robot state
    const robot_state::RobotStatePtr& current_state = move_group_ptr->getCurrentState();
    // Set the current state to the "ready" configuration
    current_state->setToDefaultValues();
    // Update the planning scene with the current robot state
    move_group_ptr->setStartState(*current_state);

    geometry_msgs::Pose target_pose;
    target_pose.orientation = p->orientation;
    target_pose.position = p->position;
    move_group_ptr->setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_ptr->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>("joint_trajectory", 10);
    trajectory_msgs::JointTrajectory joint_trajectory;
    ROS_INFO("Plan was computed");
    if (success) {
        // Get the joint trajectory from the plan
        joint_trajectory = plan.trajectory_.joint_trajectory;
        pub.publish(joint_trajectory);
    }
    else{
        ROS_INFO("No Plan found");
        plan.trajectory_.joint_trajectory.joint_names[0] = "f";
        pub.publish(joint_trajectory);
    }

}

void fullCallback(const geometry_msgs::PoseArrayConstPtr p, moveit::planning_interface::MoveGroupInterface* move_group_ptr)
{
    //hier nur zurückschicken: letzter Zustand bevor unfeasible
    //get: POseArrayMsg
    //get: length of pose array int8
    //send: int8MultiArrayMsg (1/0 feasible/unfeasible)
    //send: JointPoseMsg q für den letzten feasible Zustand (wenn es 2 gibt für den ersten)

    ROS_INFO("got new reference pose");
    //setup ros node utilities
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>("joint_angles", 10);
    ros::Publisher reach = nh.advertise<std_msgs::Bool>("reachability_full_path", 10);

    // Set the current state to the "ready" configuration
    const robot_state::RobotStatePtr& current_state = move_group_ptr->getCurrentState();
    current_state->setToDefaultValues();
    move_group_ptr->setStartState(*current_state);


    // Loop through all poses and compute the corresponding joint positions
    trajectory_msgs::JointTrajectory joint_trajectory;
    joint_trajectory.joint_names = move_group_ptr->getJointNames();
    size_t size = p->poses.size();
    bool reachable[size];
    int i = 0;
    for (const auto& pose : p->poses) {
        move_group_ptr->setPoseTarget(pose);
        // Compute the joint positions using inverse kinematics
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_ptr->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success) {
            ROS_INFO("Planning succeeded for pose!");
            // Get the joint positions from the computed plan and add them to the trajectory message
            const auto q = plan.trajectory_.joint_trajectory.points.back();
            joint_trajectory.points.push_back(q);

            // Update the current state to the next pose
            current_state->setJointGroupPositions(move_group_ptr->getName(), plan.trajectory_.joint_trajectory.points.back().positions);
            move_group_ptr->setStartState(*current_state);
            joint_trajectory.points[i].effort.push_back(1);
            i += 1;
        }
        else {
            ROS_WARN("Planning failed for pose!");
            joint_trajectory.points[i].effort.push_back(1);
            i += 1;
        }

    }


}

int main(int argc, char** argv) {
    // Initialize ROS node
    ros::init(argc, argv, "unity_handler");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);
    spinner.start();

    auto* move_group = new moveit::planning_interface::MoveGroupInterface("panda_arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    ros::Subscriber single_point = nh.subscribe<geometry_msgs::Pose>("check_single_pose", 1000, boost::bind(&singleCallback, _1, move_group));
    ros::Subscriber full_path = nh.subscribe<geometry_msgs::PoseArray>("check_full_path", 1, boost::bind(&fullCallback, _1, move_group));

    ros::waitForShutdown();

    return 0;
}