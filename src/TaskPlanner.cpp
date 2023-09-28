//
// Created by lucas on 27.09.23.
//
// Default constructor
#include <demo_classes.h>

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

TaskPlanner::TaskPlanner( moveit::planning_interface::MoveGroupInterface* move_group,  moveit::planning_interface::MoveGroupInterface* gripper_group) :
        gripper_grasp_client("franka_gripper/grasp", true),
        gripper_move_client("franka_gripper/move", true),
        gripper_stop_client("franka_gripper/stop", true)
{
    // Initialize ROS node handle
    nh_.reset(new ros::NodeHandle("~"));
    // Set default callback queue sizes
    move_group_ptr = move_group;

    // Wait for the action servers to start up
    gripper_grasp_client.waitForServer();
    gripper_move_client.waitForServer();
    gripper_stop_client.waitForServer();
} // end constructor


void TaskPlanner::multiplan_move(std::vector<double> position, std::vector<double> orientation){
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

void TaskPlanner::moveit_move(std::vector<double> position, std::vector<double> orientation){
    geometry_msgs::Pose target_pose;
    target_pose = createGoalPose(position, orientation);
    bool success = plan_until_successful(this->move_group_ptr, 3);
}

void TaskPlanner::move(std::vector<double> position, std::vector<double> orientation, ros::Publisher* goal_pose_publisher, double tol){
    geometry_msgs::PoseStamped target_pose;
    target_pose.pose = createGoalPose(position, orientation);
    goal_pose_publisher->publish(target_pose);
    Eigen::Map<Eigen::Vector3d> goal_vector(position.data());
    double elapsed_time = 0.0;
    double start_time = ros::Time::now().toSec();
    while((goal_vector-global_ee_position).norm() > tol && elapsed_time < 10.0){
        double current_time = ros::Time::now().toSec();
        elapsed_time = current_time - start_time;
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

void TaskPlanner::ee_callback(const franka_msgs::FrankaStateConstPtr & msg){
    // for the time being we only need ee positions without orientations
    //ROS_INFO("got new end effector position");
    global_ee_position.x() = msg->O_T_EE[12]; // O_T_EE is a float[16] array in COLUMN MAJOR format!
    global_ee_position.y() = msg->O_T_EE[13];
    global_ee_position.z() = msg->O_T_EE[14];
    F_ext = Eigen::Map<const Eigen::Matrix<double, 6, 1>>((msg->O_F_ext_hat_K).data());
}


