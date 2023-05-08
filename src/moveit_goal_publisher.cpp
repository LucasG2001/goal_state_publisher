/*
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <random>

int main(int argc, char** argv) {

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(-0.3, 0.3);
  
  ros::init(argc, argv, "moveit_goal_publisher");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface move_group("panda_arm");
  move_group.setPoseReferenceFrame("panda_link0");

  geometry_msgs::PoseStamped goal_state;
  goal_state.header.frame_id = "panda_link0";
  goal_state.pose.position.x = 0.0;
  goal_state.pose.position.y = 0.0;
  goal_state.pose.position.z = 0.0;

  while (ros::ok()) {
     
    goal_state.pose.position.x = dis(gen);
    goal_state.pose.position.y = dis(gen);
    goal_state.pose.position.z = dis(gen);

    move_group.setPoseTarget(goal_state);

    if (!move_group.asyncMove()) {
      ROS_WARN("Failed to send goal state to moveit");
    } else {
      ROS_INFO_STREAM("New goal state sent: " << goal_state.pose.position.x << " "
                                                << goal_state.pose.position.y << " "
                                                << goal_state.pose.position.z);
    }

    ros::Duration(5.0).sleep();
  }

  spinner.stop();
  return 0;
}
*/