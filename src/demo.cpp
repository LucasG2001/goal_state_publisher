//
// Created by lucas on 21.09.23.
//
#include "ros/ros.h"
#include <vector>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <TaskPlanner.h>
#include <Eigen/Dense>//
// Created by lucas on 21.09.23.
//
#include "ros/ros.h"
#include <vector>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <TaskPlanner.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <limits>



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
        mid_location[2] = (object_location[2] + place_location[2])/2 + 0.15; //go up a little after grasping the object, and down again to place it
        ROS_INFO("going to next object");
        task_planner.move(object_location, neutral_orientation, goal_pose_publisher, 0.02, "grasp");
        ros::Duration(0.2).sleep();
        ROS_INFO("Move1 done");
        task_planner.grasp_object();
        ros::Duration(0.2).sleep();
        ROS_INFO("grasp done");
        task_planner.move(mid_location, place_orientation, goal_pose_publisher, 0.07, "continue");
        task_planner.move(place_location, place_orientation,goal_pose_publisher, 0.06, "place"); ros::Duration(0.3).sleep();
        ROS_INFO("move2 done");
        task_planner.open_gripper(); ros::Duration(0.2).sleep();
        task_planner.move(mid_location, neutral_orientation, goal_pose_publisher, 0.07, "continue");
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
    task_planner.move(bowl_location, neutral_orientation, goal_pose_publisher, 0.02, "grasp"); ros::Duration(0.2).sleep();
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
    task_planner.move({0.5, 0.0, 0.033}, {-3.14156, 0.0, 0.0}, &task_planner.equilibrium_pose_pub, 0.02);
    ROS_INFO("re-setting stiffness");
    task_planner.control_mode_pub.publish(control_mode);
    for(int k = 0; k < trajectory_size; k++){
        ROS_INFO("proceeding to next trajectory point");
        //send force, x, y to right client
        task_planner.move({x(k,0), y(k,0), 0.025}, {-3.14156, 0.0, z_orientations(k, 0)}, cleaning_task_publisher, 0.06); //higher tolerance, no precision needed
        ros::Duration(0.05).sleep();
        //wait to finish (or send whole trajectory)
        } //for loop
    task_planner.move({0.4, 0.0, 0.5}, {-3.14156, 0.0, 0.0}, &task_planner.equilibrium_pose_pub); ros::Duration(0.5).sleep(); // go back to default position
    }

    void task4(TaskPlanner &task_planner, ros::Publisher* goal_pose_publisher){


        std::vector<double> object_location = {0.5, 0.0, 0.06};
        std::vector<double> place_location = {0.4, 0.45, 0.06};
        std::vector<double> mid_location = {0.45, 0.2, 0.2};
        std::vector<double> neutral_orientation = {-3.14156, 0.0, -1.571}; //moveit has it at -0.785 while for ee direct it's 0 ?????
        std::vector<double> place_orientation = {-3.14156, 0.0, -0.125}; //moveit has 0 orientation at -0.785 while for ee direct it's 0 ?????
        std::vector<double> object_heights = {0.045, 0.0906, 0.0906, 0.3345}; //bowl spice, spice, bottle

        task_planner.move({0.4, 0.0, 0.5}, neutral_orientation, goal_pose_publisher); ros::Duration(5).sleep();
        ROS_INFO("achieved neutral position");
        ros::Duration(0.2).sleep();
        task_planner.move({0.4, -0.35, 0.5}, neutral_orientation, goal_pose_publisher); ros::Duration(5).sleep();
        task_planner.move({0.4, 0.35, 0.5}, neutral_orientation, goal_pose_publisher); ros::Duration(5).sleep();
        task_planner.move({0.4, 0.0, 0.5}, neutral_orientation, goal_pose_publisher); ros::Duration(5).sleep();
        ROS_INFO("achieved neutral position, y done");
        task_planner.move({0.4, 0.0, 0.2}, neutral_orientation, goal_pose_publisher); ros::Duration(5).sleep();
        task_planner.move({0.4, 0.0, 0.6}, neutral_orientation, goal_pose_publisher); ros::Duration(5).sleep();
        task_planner.move({0.4, 0.0, 0.5}, neutral_orientation, goal_pose_publisher); ros::Duration(5).sleep();
        ROS_INFO("achieved neutral position, z done");
        task_planner.move({0.6, 0.0, 0.5}, neutral_orientation, goal_pose_publisher); ros::Duration(5).sleep();
        task_planner.move({0.2, 0.0, 0.5}, neutral_orientation, goal_pose_publisher); ros::Duration(5).sleep();
        task_planner.move({0.4, 0.0, 0.5}, neutral_orientation, goal_pose_publisher); ros::Duration(5).sleep();
        ROS_INFO("achieved neutral position, x done");
        task_planner.move({0.6, -0.35, 0.2}, neutral_orientation, goal_pose_publisher); ros::Duration(5).sleep();
        task_planner.move({0.2, 0.35, 0.6}, neutral_orientation, goal_pose_publisher); ros::Duration(5).sleep();
        task_planner.move({0.4, 0.0, 0.5}, neutral_orientation, goal_pose_publisher); ros::Duration(5).sleep();
        ROS_INFO("achieved neutral position, combined done");


    }

    void task5(TaskPlanner &task_planner, ros::Publisher* goal_pose_publisher)
    {
        std::vector<double> neutral_orientation = {-3.14156, 0.0, 0.0}; //moveit has it at -0.785 while for ee direct it's 0 ?????
        std::vector<std::vector<double>> object_location{
                            {0.588175,	0.333778,	0.03828},
                            {0.609549,	0.33479,	0.03788},
                            {0.62802,	0.33697,	0.0374},
                            {0.64805,	0.33472,	0.0381}
        };

        std::vector<std::vector<double>> place_location{
                            {0.5275382,	0.078622,	0.055105},
                            {0.528564,	0.14105,	0.056055},
                            {0.557536,	0.077595,	0.05707},
                            {0.55752,	0.14045,	0.05722}
        };
        double offset = 0.025;
        

        for(int i = 0; i < 4; ++i){
            task_planner.open_gripper();
            ROS_INFO("going to next object");
            object_location[i][2] += offset;
            task_planner.move(object_location[i], neutral_orientation, goal_pose_publisher, 0.002, "continue");ros::Duration(0.2).sleep();
            object_location[i][2] -= offset;
            task_planner.move(object_location[i], neutral_orientation, goal_pose_publisher, 0.002, "grasp");ros::Duration(0.2).sleep();
            task_planner.grasp_object();ros::Duration(0.2).sleep();
            ROS_INFO("going to place objct");

            place_location[i][2] += offset;
            task_planner.move(place_location[i], neutral_orientation, goal_pose_publisher, 0.002, "continue");ros::Duration(0.2).sleep();
            place_location[i][2] -= offset;
            task_planner.move(place_location[i], neutral_orientation, goal_pose_publisher, 0.002, "continue");ros::Duration(0.2).sleep();
            task_planner.open_gripper(); ros::Duration(0.2).sleep();        
            place_location[i][2] += offset;
            task_planner.move(place_location[i], neutral_orientation, goal_pose_publisher, 0.002, "continue");ros::Duration(0.2).sleep();
            ROS_INFO("placed object");
        }
    }

    void test_mode(TaskPlanner &task_planner){

        goal_state_publisher::testMsg usr_input;
        usr_input.test = true;

        std::cout << "What joint do you want to test? (0-7)";
        std::cin >> usr_input.joint;
        
        task_planner.pub_test.publish(usr_input);
        
    }

    void friction_selection(TaskPlanner &task_planner, std_msgs::Bool &friction_usr){

        bool userInput;
        std::cout << "Do you want to use friction compensation? (0/1) \n";
        std::cin >> userInput;
    
        if(std::cin.fail()){
            std::cout << "Invalid input \n";
        }

        friction_usr.data = userInput;

        if(friction_usr.data){
            std::cout << "I read true \n";
        }
        else{
            std::cout << "I read false \n";
        }
        task_planner.friction_pub.publish(friction_usr);
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
    task_planner.pub_test = n.advertise<goal_state_publisher::testMsg>("test_topic", 1); 
    task_planner.friction_pub = n.advertise<std_msgs::Bool>("friction_topic", 1);
    ros::Publisher cleaning_task_pub = n.advertise<geometry_msgs::PoseStamped>("/cartesian_impedance_controller/panda_force_action", 1);
    ros::Subscriber ee_pose = n.subscribe("/franka_state_controller/franka_states", 10, &TaskPlanner::ee_callback, &task_planner);


    int grip_action;    //input for flow control
    franka_gripper::StopAction stop;
    franka_gripper::StopGoalConstPtr stop_goal;
    std_msgs::Int16 control_mode_msg;
    std_msgs::Bool friction_usr;
    while (ros::ok()) {

        // Prompt user to input three values
        std::cout
                << "Enter 0 (go home) | 1 (task1) | 2 (task2) | 3 (task3) | 4 (try task 1-3 in succession)| 5 (free-float) | 6 (reactivate stiffness) | 7 (test mode) | 8 (test mode 2) | 9 (demo PCB)";
        std::cin >> grip_action;
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

        if(grip_action != 7){
            goal_state_publisher::testMsg usr_input;
            usr_input.test = false;
            task_planner.pub_test.publish(usr_input);
        }

        switch (grip_action) {
            case 0:
                task_planner.move({0.4, 0.05, 0.3}, {-3.14156, 0, 0}, &task_planner.equilibrium_pose_pub);
                break;
            case 1:
                friction_selection(task_planner, friction_usr);
                task1(task_planner,  &task_planner.equilibrium_pose_pub);
                friction_usr.data = 0;
                task_planner.friction_pub.publish(friction_usr);
                break;
            case 2:
                friction_selection(task_planner, friction_usr);
                task2(task_planner, &task_planner.equilibrium_pose_pub);
                friction_usr.data = 0;
                task_planner.friction_pub.publish(friction_usr);
                break;
            case 3:
                friction_selection(task_planner, friction_usr);
                task3(task_planner, 5.0, &cleaning_task_pub);
                friction_usr.data = 0;
                task_planner.friction_pub.publish(friction_usr);
                break;
            case 4:
                friction_selection(task_planner, friction_usr);
                task1(task_planner,  &task_planner.equilibrium_pose_pub);
                task2(task_planner, &task_planner.equilibrium_pose_pub);
                task3(task_planner, 5.0, &cleaning_task_pub);
                friction_usr.data = 0;
                task_planner.friction_pub.publish(friction_usr);
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
                test_mode(task_planner);
                break;
            case 8:
                friction_selection(task_planner, friction_usr);
                task4(task_planner, &task_planner.equilibrium_pose_pub);
                friction_usr.data = 0;
                task_planner.friction_pub.publish(friction_usr);
                break;
            case 9:
                friction_selection(task_planner, friction_usr);
                task5(task_planner, &task_planner.equilibrium_pose_pub);
                friction_usr.data = 0;
                task_planner.friction_pub.publish(friction_usr);
                break;
        } //switch case
        std::cout << "current end-effector position is at " << task_planner.global_ee_position.transpose() << std::endl;
	    std::cout << "current end-effector orientation is " << task_planner.global_ee_orientation.coeffs().transpose() << std::endl;
        loop_rate.sleep();
    } //while ros node
    return 0;
} //main