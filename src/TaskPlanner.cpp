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

void TaskPlanner::move(std::vector<double> position, std::vector<double> orientation, ros::Publisher* goal_pose_publisher, double tol, bool clear_integrator){
    double trajectory_intervals = 0.12;
    geometry_msgs::PoseStamped target_pose;
    Eigen::Map<Eigen::Vector3d> goal_position(position.data());
    Eigen::Map<Eigen::Vector3d> goal_orientation(position.data());
    Eigen::Matrix<double, 6, 1> waypoint_number;
    waypoint_number.head(3) = ((goal_position-global_ee_position)/trajectory_intervals).array().abs().ceil(); //position with 0.1 spaced waypoints
    ROS_INFO_STREAM(" waypoints are distributed as " << waypoint_number);
    waypoint_number.tail(3).setZero();
    //at the moment only use position
    //waypoint_number.head(3) = (goal_position/0.785).array().abs().ceil(); //orientation with 0.785 spaced waypoints
    Eigen::Vector3d step_size = (goal_position - global_ee_position).array() / waypoint_number.head(3).array();
    ROS_INFO_STREAM("step sizes are " << step_size);
    Eigen::Vector3d start_position = global_ee_position;
    Eigen::Vector3d lower_bound = start_position.cwiseMin(goal_position);
    Eigen::Vector3d upper_bound = start_position.cwiseMax(goal_position);
    ROS_INFO_STREAM("going from " << start_position << " to ");
    ROS_INFO_STREAM(" " << goal_position);
    tol *= 3; //double tolerance for middle waypoints
    double goal_time = 1;
    if (clear_integrator){ target_pose.header.frame_id = "clear_integrator"; }  //send this to clear integrator at waypoint start
    for (int i = 1; i < waypoint_number.maxCoeff() + 1; i++){
        if (i == waypoint_number.maxCoeff()){
            tol *= 0.25;//get back tolerance for last waypoint
            target_pose.header.frame_id = "continue";
            goal_time = 10;
        }
        Eigen::Vector3d reference_pos = (start_position + i * step_size); //just add up correct later
        reference_pos = reference_pos.cwiseMax(lower_bound).cwiseMin(upper_bound);
        std::vector<double> reference_for_msg = {reference_pos(0,0), reference_pos(1,0), reference_pos(2,0)};
        ROS_INFO_STREAM("waypoint " << i << " is " << reference_pos);
        //generate a trajectory with waypoints
        target_pose.pose = createGoalPose(reference_for_msg, orientation);
        goal_pose_publisher->publish(target_pose);
        Eigen::Map<Eigen::Vector3d> goal_vector(position.data());
        double elapsed_time = 0.0;
        double start_time = ros::Time::now().toSec();
        while((goal_vector-global_ee_position).norm() > tol && elapsed_time < goal_time){
            double current_time = ros::Time::now().toSec();
            elapsed_time = current_time - start_time;
            ros::Duration(0.1).sleep();
        }
        //target_pose.header.frame_id = "fill_integrator"; //start filling the integrator again
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
    //move upward a little bit after grasp
    geometry_msgs::PoseStamped target_pose;
    target_pose.pose.position.x = global_ee_position.x();
    target_pose.pose.position.y = global_ee_position.y();
    target_pose.pose.position.z = global_ee_position.z() + 0.1;
    target_pose.pose.orientation.x = global_ee_orientation.x();
    target_pose.pose.orientation.y = global_ee_orientation.y();
    target_pose.pose.orientation.z = global_ee_orientation.z();
    target_pose.pose.orientation.w = global_ee_orientation.w();
    equilibrium_pose_pub.publish(target_pose);
    ros::Duration(2.0).sleep();

}

void TaskPlanner::ee_callback(const franka_msgs::FrankaStateConstPtr & msg){
    // for the time being we only need ee positions without orientations
    //ROS_INFO("got new end effector position");
    global_ee_position.x() = msg->O_T_EE[12]; // O_T_EE is a float[16] array in COLUMN MAJOR format!
    global_ee_position.y() = msg->O_T_EE[13];
    global_ee_position.z() = msg->O_T_EE[14];
    F_ext = Eigen::Map<const Eigen::Matrix<double, 6, 1>>((msg->O_F_ext_hat_K).data());
}


