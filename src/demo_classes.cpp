//
// Created by lucas on 21.09.23.
//
#include "ros/ros.h"
#include <vector>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include "demo_classes.h"
#include "utility.h"
#include <franka_msgs/FrankaState.h>
#include <Eigen/Dense>
#include <std_msgs/Int16.h>

bool plan_until_successful(moveit::planning_interface::MoveGroupInterface* move_group_ptr, int tries){
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = false;
    int attempt = 0;
    while(!success && attempt <= tries){
        success = (move_group_ptr->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO("Plan was computed");
        if (success){
            move_group_ptr->execute(plan); //this call should be blocking
            ROS_INFO("Plan found");
            ROS_INFO("Executing!");
            return success;
        }
        else { ROS_WARN("Could not Plan Motion successfully");
            ROS_INFO_STREAM("proceeding to attempt nr " << (attempt +1));
        }
        attempt += 1;
    }
    ROS_WARN("Even after multiple attempts no plan was found");
    return false;
}
// Default constructor
TaskPlanner::TaskPlanner( moveit::planning_interface::MoveGroupInterface* move_group,  moveit::planning_interface::MoveGroupInterface* gripper_group) :
        gripper_grasp_client("franka_gripper/grasp", true),
        gripper_move_client("franka_gripper/move", true),
        gripper_stop_client("franka_gripper/stop", true)
    {
    // Initialize ROS node handle
    nh_.reset(new ros::NodeHandle("~"));
    // Set default callback queue sizes
    move_pub = nh_->advertise<geometry_msgs::Pose>("/panda_pose_reference", 1);
    move_group_ptr = move_group;
    gripper_group_ptr = gripper_group;

    // Wait for the action servers to start up
    gripper_grasp_client.waitForServer();
    gripper_move_client.waitForServer();
    gripper_stop_client.waitForServer();
} // end constructor

/*
void TaskPlanner::move(std::vector<double> position, std::vector<double> orientation){
        geometry_msgs::Pose target_pose;
        target_pose = createGoalPose(position, orientation);
        move_group_ptr->setPoseTarget(target_pose);
        move_group_ptr->setStartStateToCurrentState();
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (this->move_group_ptr->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO("Plan was computed");
        if (success){
            this->move_group_ptr->execute(plan); //this call should be blocking
            ROS_INFO("Plan found");
            ROS_INFO("Executing!");
        }
        else { ROS_WARN("Could not Plan Motion successfully");}
    }
    */

    /*
void TaskPlanner::move(std::vector<double> position, std::vector<double> orientation){
        geometry_msgs::Pose target_pose;
        target_pose = createGoalPose(position, orientation);
        bool success = plan_until_successful(this->move_group_ptr, 3);
    }
     */
    void TaskPlanner::move(std::vector<double> position, std::vector<double> orientation, ros::Publisher* goal_pose_publisher){
        geometry_msgs::PoseStamped target_pose;
        target_pose.pose = createGoalPose(position, orientation);
        goal_pose_publisher->publish(target_pose);
        Eigen::Map<Eigen::Vector3d> goal_vector(position.data());
        while((goal_vector-global_ee_position).norm() > 0.03){
            ros::Duration(0.1).sleep();
        }
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
}

void task1(TaskPlanner &task_planner, ros::Publisher* goal_pose_publisher){

    std::vector<double> object_location = {0.5, 0.0, 0.2};
    std::vector<double> place_location = {0.3, 0.2, 0.2};
    std::vector<double> neutral_orientation = {-3.14156, 0.0, -0.0}; //moveit has it at -0.785 while for ee direct it's 0 ?????
    for (size_t i = 0; i < 5; i++){
        task_planner.open_gripper(); //reset open gripper
        ROS_INFO("going to next object");
        task_planner.move(object_location, neutral_orientation, goal_pose_publisher);
        ros::Duration(0.3).sleep();
        ROS_INFO("Move1 done");
        task_planner.grasp_object();
        ros::Duration(0.3).sleep();
        ROS_INFO("grasp done");
        task_planner.move(place_location, neutral_orientation,goal_pose_publisher);
        ros::Duration(0.3).sleep();
        ROS_INFO("move2 done");
        task_planner.open_gripper();
        ros::Duration(0.3).sleep();
        ROS_INFO("placed object");
        object_location[0] -= 0.03;
        object_location[1] -= 0.1;
        place_location[0] += 0.05;
        place_location[1] += 0.05;
    }
    //go back to ready position
    task_planner.move({0.4, 0.0, 0.5}, neutral_orientation, goal_pose_publisher);
}

void task2(TaskPlanner &task_planner, ros::Publisher* goal_pose_publisher){
        std_msgs::Int16 control_mode;
        control_mode.data = 1;
   while(task_planner.F_ext.norm() < 5.0){ //wait until human pushes robot arm
       task_planner.control_mode_pub.publish(control_mode);
       ros::Duration(0.2).sleep();
   }
    control_mode.data = 0;
    task_planner.control_mode_pub.publish(control_mode);
   //go to bowl pick it up and throw it into the pan
    std::vector<double> bowl_location = {0.5, 0.0, 0.2};
    std::vector<double> pan_location = {0.1, 0.4, 0.3};
    std::vector<double> neutral_orientation = {-3.14156, 0.0, -0.0};
    task_planner.open_gripper(); //reset open gripper
    ROS_INFO("Directing towards Bowl");
    task_planner.move(bowl_location, neutral_orientation, goal_pose_publisher); ros::Duration(0.3).sleep();
    task_planner.grasp_object(); ros::Duration(0.3).sleep();
    ROS_INFO("Directing towards pan");
    task_planner.move(pan_location, neutral_orientation, goal_pose_publisher);
    ROS_INFO("Filling Pan");
    task_planner.move(pan_location, {-3.14156, -0.785, -0.0}, goal_pose_publisher);
    ros::Duration(3).sleep(); //WATCH OUT -> Here we set the "POURING" time
    //place bowl
    ROS_INFO("Going back to idle");
    task_planner.move({0.2, 0.3, 0.1}, {-3.14156, -0.0, -0.0}, goal_pose_publisher); ros::Duration(0.3).sleep();
    task_planner.open_gripper();
    ros::Duration(0.3).sleep();
    //go back to ready position
    task_planner.move({0.4, 0.0, 0.5}, neutral_orientation, goal_pose_publisher);

}

void task3(TaskPlanner& task_planner, double downward_force /*some publisher (or add it to task_planner)*/){
    std_msgs::Int16 control_mode;
    control_mode.data = 0;
    while(task_planner.F_ext.norm() < 5.0){ //wait until human pushes robot arm
        task_planner.control_mode_pub.publish(control_mode);
        ros::Duration(0.2).sleep();
    }
    control_mode.data = 1; //stiffen up end effector (probably needs to be even more stiff)
        //trajectory should look approximately like plot 0.35 * sin(25x) for x = 0.1 to 0.65
    double omega = 0.15 * M_PI; //rad^-1
    double a = 0.5; //m, will be 0 -> no sin behaviour
    //create a trajectory message or simply publish points
    ros::Time start = ros::Time::now();
    int trajectory_size = 0;
    Eigen::VectorXd x = Eigen::VectorXd::LinSpaced(20, 0.1, 0.65);
    Eigen::VectorXd y = 0.35 * (x.array().sin() * 25).matrix();
    for(int k = 0; k < trajectory_size; k++){
        //send force, x, y to right client
        //wait to finish (or send whole trajectory)
        } //for loop
    }


void TaskPlanner::ee_callback(const franka_msgs::FrankaStateConstPtr & msg){
    // for the time being we only need ee positions without orientations
    //ROS_INFO("got new end effector position");
    global_ee_position.x() = msg->O_T_EE[12]; // O_T_EE is a float[16] array in COLUMN MAJOR format!
    global_ee_position.y() = msg->O_T_EE[13];
    global_ee_position.z() = msg->O_T_EE[14];
    F_ext = Eigen::Map<const Eigen::Matrix<double, 6, 1>>((msg->O_F_ext_hat_K).data());
}

int main(int argc, char **argv) {
    //ToDo: Use argv to set robot (panda/fr3)
    ros::init(argc, argv, "demo_planner");
    ros::NodeHandle n;

    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::Rate loop_rate(0.1);
    ros::Rate update_rate(20);

    //create move group_connection
    moveit::planning_interface::MoveGroupInterface move_group("panda_arm"); //or panda_arm
    moveit::planning_interface::MoveGroupInterface gripper_group("panda_hand");
    TaskPlanner task_planner(&move_group, &gripper_group);
    //subscribe to ee_pos
    ros::Publisher equilibrium_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/cartesian_impedance_controller/reference_pose", 1);
    task_planner.control_mode_pub = n.advertise<std_msgs::Int16>("/cartesian_impedance_controller/control_mode", 1);
    ros::Subscriber ee_pose = n.subscribe("/franka_state_controller/franka_states", 10, &TaskPlanner::ee_callback, &task_planner);


    setPlanningParameters(move_group, 7, 20, 0.3, 0.3);
    int grip_action;    //input for flow control
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> force_client(
            "cartesian_impedance_controller/follow_joint_trajectory", true);


    while (ros::ok()) {

        // Prompt user to input three values
        std::cout
                << "Enter 0 (move, nothing else) | 1 (open gripper) | 2 (grasp) | 3 (stop) | 4 (try task 1)| ";
        std::cin >> grip_action;
        franka_gripper::StopAction stop;
        franka_gripper::StopGoalConstPtr stop_goal;
        switch (grip_action) {
            case 0:
                //task_planner.move({0.4, 0.05, 0.3}, {-3.14156, 0, -0.785});
                break;
            case 1:
                task_planner.open_gripper(0.08, 0.2);
                break;
            case 2:
                task_planner.grasp_object(0.2, 0.02, 20, 0.005);
                break;
            case 3:
                task_planner.stop_pub.publish(stop_goal);
                break;
            case 4:
                task1(task_planner,  &equilibrium_pose_pub);
                task2(task_planner, &equilibrium_pose_pub);
                task3(task_planner, 5.0);
                break;

        } //switch case
        //ros::spinOnce();
        loop_rate.sleep();
    } //while ros node
    return 0;
} //main