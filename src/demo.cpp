//
// Created by lucas on 21.09.23.
//
#include "ros/ros.h"
#include <vector>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <TaskPlanner.h>
#include "utility.h"
#include <Eigen/Dense>
#include <std_msgs/Int16.h>


void task1(TaskPlanner &task_planner, ros::Publisher* goal_pose_publisher){

    std::vector<double> object_location = {0.5, 0.0, 0.06};
    std::vector<double> place_location = {0.4, 0.45, 0.06};
    std::vector<double> mid_location = {0.45, 0.2, 0.2};
    std::vector<double> neutral_orientation = {-3.14156, 0.0, -1.571}; //moveit has it at -0.785 while for ee direct it's 0 ?????
    std::vector<double> place_orientation = {-3.14156, 0.0, -0.125}; //moveit has 0 orientation at -0.785 while for ee direct it's 0 ?????
    std::vector<double> object_heights = {0.045, 0.0906, 0.0906, 0.3345}; //bowl spice, spice, bottle
    for (size_t i = 0; i < 4; i++){
        if (i == 3){
            place_location = {0.27, -0.7, 0.16}; //last location (bottle)
        }
        //TODO write function pick & place
        task_planner.open_gripper(); //reset open gripper
        //set place locations and waypoints
        object_location[2] = object_heights[i];
        place_location[2] = object_location[2] + 0.005;
        mid_location[0] = (object_location[0] + place_location[0])/2;
        mid_location[1] = (object_location[1] + place_location[1])/2;
        mid_location[2] = (object_location[2] + place_location[2])/2 + 0.25; //go up a little after grasping the object, and down again to place it
        ROS_INFO("going to next object");
        task_planner.move({object_location[0], object_location[1], object_location[2] + 0.05}, neutral_orientation, goal_pose_publisher, 0.04, "grasp");
        task_planner.move(object_location, neutral_orientation, goal_pose_publisher, 0.015, "grasp");
        ROS_INFO("Move1 done");
        task_planner.grasp_object();
        ROS_INFO("grasp done");
        task_planner.move(mid_location, place_orientation, goal_pose_publisher, 0.04, "continue");
        task_planner.move(place_location, place_orientation,goal_pose_publisher, 0.04, "place");
        ROS_INFO("move2 done");
        task_planner.open_gripper();
        task_planner.move(mid_location, neutral_orientation, goal_pose_publisher, 0.04, "continue");
        ROS_INFO("placed object");
        object_location[0] -= 0.03;
        object_location[1] -= 0.1;
        place_location[0] += 0.04;
        place_location[1] -= 0.09;
    }
    //go back to ready position
    task_planner.move({0.4, 0.0, 0.5}, neutral_orientation, goal_pose_publisher); ros::Duration(0.5).sleep();
}

void task2(TaskPlanner &task_planner, ros::Publisher* goal_pose_publisher){
        std_msgs::Int16 control_mode;
        control_mode.data = 1;
    double elapsed_time = 0.0;
    double start_time = ros::Time::now().toSec();
   while(task_planner.F_ext.norm() < 10.0 && elapsed_time < 15.0){ //wait until human pushes robot arm
       task_planner.control_mode_pub.publish(control_mode);
       double current_time = ros::Time::now().toSec();
       elapsed_time = current_time - start_time;
       ros::Duration(0.5).sleep();
   }
    control_mode.data = 0;
    task_planner.control_mode_pub.publish(control_mode);
   //go to bowl pick it up and throw it into the pan
    std::vector<double> bowl_location = {0.5, 0.0, 0.07};
    std::vector<double> pan_location = {0.05, 0.59, 0.3};
    std::vector<double> neutral_orientation = {-3.14156, 0.0, -0.0};
    task_planner.open_gripper(); //reset open gripper
    ROS_INFO("Directing towards Bowl");
    task_planner.move(bowl_location, neutral_orientation, goal_pose_publisher, 0.015, "grasp"); ros::Duration(0.2).sleep();
    task_planner.grasp_object(); ros::Duration(0.2).sleep();
    ROS_INFO("Directing towards pan");
    task_planner.move(pan_location, neutral_orientation, goal_pose_publisher);
    ROS_INFO("Filling Pan");
    task_planner.move(pan_location, {-3.14156, 1.6, -0.0}, goal_pose_publisher);
    ros::Duration(3).sleep(); //WATCH OUT -> Here we set the "POURING" time
    //place bowl
    task_planner.move({0.2, 0.3, 0.06}, {-3.14156, -0.0, -0.0}, goal_pose_publisher, 0.04, "place"); ros::Duration(0.2).sleep();
    task_planner.open_gripper();
    ros::Duration(0.2).sleep();
    //go back to ready position
    ROS_INFO("Going back to idle");
    task_planner.move({0.4, 0.0, 0.5}, neutral_orientation, goal_pose_publisher);

}

void task3(TaskPlanner& task_planner, double downward_force, ros::Publisher* cleaning_task_publisher){
        // at the moment we don't need downward force
        //create trajectory points
    double omega = 55.0; //rad^-1
    double a = 0.3;
    //create a trajectory message or simply publish points
    //trajectory should look approximately like plot 0.35 * sin(25x) for x = 0.1 to 0.65
    ros::Time start = ros::Time::now();
    int trajectory_size = 40;
    Eigen::VectorXd x = Eigen::VectorXd::LinSpaced(trajectory_size, 0.30, 0.7);
    Eigen::VectorXd y = a * ((x*omega).array().sin()).matrix();
    Eigen::VectorXd z_orientations = (3.14156/3) * ((x*50).array().sin()).matrix();
    //set free float
    std_msgs::Int16 control_mode;
    control_mode.data = 1;
    ROS_INFO("setting idle");
    // time for simulation
    double elapsed_time = 0.0;
    double start_time = ros::Time::now().toSec();
    while(task_planner.F_ext.norm() < 5.0 && elapsed_time < 10.0){ //wait until human pushes robot arm
        task_planner.control_mode_pub.publish(control_mode);
        double current_time = ros::Time::now().toSec();
        elapsed_time = current_time - start_time;
        ros::Duration(0.2).sleep();
    }
    //stiffen up end effector (probably needs to be even more stiff)
    control_mode.data = 0;
    ROS_INFO("re-setting stiffness");
    task_planner.control_mode_pub.publish(control_mode);
    //move to start point
    ROS_INFO_STREAM("Moving to starting position " << x(0,0) << " " << y(0,0));
    task_planner.move({0.5, 0.0, 0.026}, {-3.14156, 0.0, 0.0}, &task_planner.equilibrium_pose_pub, 0.005);
    ROS_INFO("re-setting stiffness");
    task_planner.control_mode_pub.publish(control_mode);
    for(int k = 0; k < trajectory_size; k++){
        ROS_INFO("proceeding to next trajectory point");
        //send force, x, y to right client
        task_planner.move({x(k,0), y(k,0), 0.025}, {-3.14156, 0.0, z_orientations(k, 0)}, cleaning_task_publisher, 0.04); //higher tolerance, no precision needed
        ros::Duration(0.05).sleep();
        //wait to finish (or send whole trajectory)
        } //for loop
    task_planner.move({0.4, 0.0, 0.5}, {-3.14156, 0.0, 0.0}, &task_planner.equilibrium_pose_pub); ros::Duration(0.5).sleep(); // go back to default position
    }

	void press_down_static(TaskPlanner& task_planner, double downward_force, ros::Publisher* cleaning_task_publisher){
        //stiffen up end effector
        std_msgs::Int16 control_mode;
        control_mode.data = 0;
        ROS_INFO("re-setting stiffness");
        task_planner.control_mode_pub.publish(control_mode);
        //move to start point
        task_planner.move({0.5, 0.0, 0.024}, {-3.14156, 0.0, 0.0}, &task_planner.equilibrium_pose_pub, 0.005);
        double elapsed_time = 0.0;
        double start_time = ros::Time::now().toSec();
        while(elapsed_time <= 5.0){
            ROS_INFO("proceeding to next trajectory point");
            //send force, x, y to right client
            task_planner.move({0.5, 0.0, 0.024}, {-3.14156, 0.0, 0.0}, cleaning_task_publisher, 0.005); //higher tolerance, no precision needed
            ros::Duration(0.05).sleep();
            elapsed_time = ros::Time::now().toSec() - start_time;
            //wait to finish (or send whole trajectory)
            } //for loop
        task_planner.move({0.5, 0.0, 0.024}, {-3.14156, 0.0, 0.0}, &task_planner.equilibrium_pose_pub); ros::Duration(0.5).sleep(); // go back to default position
}


int main(int argc, char **argv) {
    //ToDo: Use argv to set robot (panda/fr3)
    ros::init(argc, argv, "demo_planner");
    ros::NodeHandle n;

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::Rate loop_rate(0.5);

    TaskPlanner task_planner;
    //subscribe to ee_pos
    task_planner.equilibrium_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/cartesian_impedance_controller/reference_pose", 1);
    task_planner.control_mode_pub = n.advertise<std_msgs::Int16>("/cartesian_impedance_controller/control_mode", 1);
    ros::Publisher cleaning_task_pub = n.advertise<geometry_msgs::PoseStamped>("/cartesian_impedance_controller/panda_force_action", 1);
    ros::Subscriber ee_pose = n.subscribe("/franka_state_controller/franka_states", 10, &TaskPlanner::ee_callback, &task_planner);


    int grip_action;    //input for flow control
    franka_gripper::StopAction stop;
    franka_gripper::StopGoalConstPtr stop_goal;
    std_msgs::Int16 control_mode_msg;
    while (ros::ok()) {

        // Prompt user to input three values
        std::cout
                << "Enter 0 (move to) | 1 (task1) | 2 (task2) | 3 (task3) | 4 (try task 1-3 in succession)| 5 (free-float) | 6 (reactivate stiffness) | 7 (press down) ";
        std::cin >> grip_action;
		std::vector<double> position(3);
	    std::vector<double> orientation(3);
        switch (grip_action) {
            case 0:
	            std::cout << "Enter Reference Position: ";
		        std::cin >> position[0] >> position[1] >> position[2];
		        std::cout << "Enter Reference Orientation:  (>3.14156 as first entry if default)  ";
		        std::cin >> orientation[0] >> orientation[1] >> orientation[2];
				if (orientation[0] > 3.2)
				{
						orientation = {-3.14156, 0.0, 0.0};
				}
                task_planner.move(position, orientation, &task_planner.equilibrium_pose_pub);
                break;
            case 1:
                task1(task_planner,  &task_planner.equilibrium_pose_pub);
                break;
            case 2:
                task2(task_planner, &task_planner.equilibrium_pose_pub);
                break;
            case 3:
                task3(task_planner, 5.0, &cleaning_task_pub);
                break;
            case 4:
                task1(task_planner,  &task_planner.equilibrium_pose_pub);
                task2(task_planner, &task_planner.equilibrium_pose_pub);
                task3(task_planner, 5.0, &cleaning_task_pub);
                break;
            case 5:
                control_mode_msg.data = 1; // 1 for free float!
                task_planner.control_mode_pub.publish(control_mode_msg);
                break;
            case 6:
                control_mode_msg.data = 0;
                task_planner.control_mode_pub.publish(control_mode_msg);
                break;
				case 7:
                  press_down_static(task_planner, 5.0, &cleaning_task_pub);
                break;
        } //switch case
        std::cout << "current end-effector position is at " << task_planner.global_ee_position.transpose() << std::endl;
        loop_rate.sleep();
    } //while ros node
    return 0;
} //main