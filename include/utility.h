//
// Created by lucas on 28.03.23.
//

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <franka_gripper/franka_gripper.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#ifndef GOAL_STATE_PUBLISHER_UTILITY_H
#define GOAL_STATE_PUBLISHER_UTILITY_H


geometry_msgs::Pose createGoalPose(std::vector<double> position, std::vector<double> orientation = {0.0, 0.0, 0.0});

franka_gripper::GraspGoal createGraspGoal(double width = 0.02, double force = 40.0, double eps = 0.005, double speed = 0.1);

void setPlanningParameters(moveit::planning_interface::MoveGroupInterface& move_group, double plan_time= 25.0, int attempts = 10, double max_v = 0.5, double max_a = 0.5);

class GripperTask
{
public:
    double width;
    double speed;
    double force;
    double tolerance;
    bool stop;
    GripperTask() {
        width = 0.08;
        speed = 0.1;
        force = 0;
        tolerance = 0.005;
        stop = false;
    }
};

#endif //GOAL_STATE_PUBLISHER_UTILITY_H
