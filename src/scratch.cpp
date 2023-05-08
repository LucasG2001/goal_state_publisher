#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <franka_gripper/franka_gripper.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "gripper_functions.h"
#include "utility.h"

int main(int argc, char** argv) {
    // Initialize ROS node
    ros::init(argc, argv, "pick_and_place");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(8);
    spinner.start();

    // Create a client object for the Franka gripper grasp action
    actionlib::SimpleActionClient<franka_gripper::GraspAction> grasp_client("franka_gripper/grasp", true);

    // Create a client object for the Franka gripper move action
    actionlib::SimpleActionClient<franka_gripper::MoveAction> move_client("franka_gripper/move", true);

    // Create a client object for the Franka gripper stop action
    actionlib::SimpleActionClient<franka_gripper::StopAction> stop_client("franka_gripper/stop", true);

    // Wait for the action servers to start up
    grasp_client.waitForServer();
    move_client.waitForServer();
    stop_client.waitForServer();

    // Initialize MoveIt
    moveit::planning_interface::MoveGroupInterface move_group("panda_arm");
    moveit::planning_interface::MoveGroupInterface gripper_group("panda_hand");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Instantiate MoveItVisualTools for visualizing plans
    moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
    ROS_INFO("Initialized Moveit and Clients");

    // Set the planner parameters
    setPlanningParameters(move_group, 25.0, 10, 0.8, 0.8);

    // Set the start and goal poses
    // Get the current state of the robot in Gazebo
    ROS_INFO("Setting current state");
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    move_group.setStartStateToCurrentState();


    geometry_msgs::Pose target_pose = createGoalPose({atof(argv[1]), atof(argv[2]), atof(argv[3])}, {-M_PI/2, -M_PI/4, -M_PI/2});
    move_group.setPoseTarget(target_pose);
    ROS_INFO("goal state set");

    geometry_msgs::Pose place_pose= createGoalPose({0.405, 0.0, 0.8}, {-M_PI/2, -M_PI/4, -M_PI/2});
    ROS_INFO("placing state set");

    //set collision objects
    ROS_INFO("adding objects to the scene");
    std::vector<std::string> object_names = addCollisionObjects(planning_scene_interface);
    //object to be picked up
    moveit_msgs::AttachedCollisionObject object;
    object.object.header.frame_id = move_group.getPlanningFrame();
    object.link_name = "panda_hand";
    object.touch_links = std::vector<std::string>{"panda_hand","panda_leftfinger","panda_rightfinger"};
    object.object.id = object_names[2];
    object.weight = 0.5;

    ros::WallDuration(2.5).sleep();

    ROS_INFO("Collisions set");
    // Plan the path
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Plan was computed");
    if (success){
        move_group.execute(plan);
        ROS_INFO("Plan found");
        ROS_INFO("Executing!");
    }
    else { ROS_WARN("Could not Plan Motion successfully");}


    // create goals
    franka_gripper::GraspGoal grasp_goal = createGraspGoal(atof(argv[4]), atof(argv[5]), 0.005, 0.15);
    franka_gripper::MoveGoal move_goal;
    franka_gripper::StopGoal stop_goal;
    //move goal (open gripper)
    move_goal.speed = 0.1;
    move_goal.width = 0.08;

    //wait for the gripper to open
    move_client.sendGoal(move_goal);
    bool gripper_success = move_client.waitForResult(ros::Duration(5.0));
    //Wait for gripper to grasp
    grasp_client.sendGoal(grasp_goal);
    gripper_success = grasp_client.waitForResult(ros::Duration(5.0));
    //wait for everything to finish

    planning_scene_interface.applyAttachedCollisionObject(object);
    move_group.setSupportSurfaceName(object_names[0]);

    ROS_INFO("attached object");
    //planning_scene_interface.removeCollisionObjects({"object"});
    ROS_INFO("deleted redundant object");
    ros::WallDuration(2.5).sleep();
    move_group.setPoseTarget(place_pose);
    success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Plan for placing was computed");
    if (success){
        move_group.execute(plan);
        ROS_INFO("Plan found");
        ROS_INFO("Executing!");
    }
    else { ROS_WARN("Could not Plan Motion successfully");}

    //detach and remove objects
    move_group.detachObject(object.object.id);
    planning_scene_interface.removeCollisionObjects(planning_scene_interface.getKnownObjectNames());

    // Shutdown the node
    ros::shutdown();
    return 0;

}