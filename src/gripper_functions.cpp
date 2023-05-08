//
// Created by lucas on 21.03.23.
//

#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

/** utility functions to create the collision object and set its orientation **/

moveit_msgs::CollisionObject createBox (std::string id, std::string frame_id, float width, float length, float height){
    moveit_msgs::CollisionObject collision_object;
    ROS_INFO("Creating collision Object");
    collision_object.id = id;
    collision_object.header.frame_id = frame_id;
    // Define the primitive and its dimensions.
    collision_object.primitives.resize(1);
    collision_object.primitives[0].type = collision_object.primitives[0].BOX;
    collision_object.primitives[0].dimensions.resize(3);
    collision_object.primitives[0].dimensions[0] = width;
    collision_object.primitives[0].dimensions[1] = length;
    collision_object.primitives[0].dimensions[2] = height;
    // Define the orientation
    collision_object.primitive_poses.resize(1);
    collision_object.primitive_poses[0].orientation.w = 1.0;

    return collision_object;
}

void setCollisionObjectPose (moveit_msgs::CollisionObject& collision_object, std::vector<float>position, std::vector<float>orientation = {0.0, 0.0, 0.0}){
    ROS_INFO("Setting collision Object Pose");
    tf2::Quaternion orientation_quaternion;
    orientation_quaternion.setRPY(orientation[0], orientation[1], orientation[2]);
    // ... code to populate the collision_object ...
    collision_object.primitive_poses[0].orientation = tf2::toMsg(orientation_quaternion);
    collision_object.primitive_poses[0].position.x = position[0];
    collision_object.primitive_poses[0].position.y = position[1];
    collision_object.primitive_poses[0].position.z = position[2];

}
void openGripper(trajectory_msgs::JointTrajectory& posture)
{
    // BEGIN_SUB_TUTORIAL open_gripper
    /* Add both finger joints of panda robot. */
    posture.joint_names.resize(2);
    posture.joint_names[0] = "panda_finger_joint1";
    posture.joint_names[1] = "panda_finger_joint2";

    /* Set them as open, wide enough for the object to fit. */
    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.04;
    posture.points[0].positions[1] = 0.04;
    posture.points[0].time_from_start = ros::Duration(0.5);
    // END_SUB_TUTORIAL
}

void closedGripper(trajectory_msgs::JointTrajectory& posture)
{
    // BEGIN_SUB_TUTORIAL closed_gripper
    /* Add both finger joints of panda robot. */
    posture.joint_names.resize(2);
    posture.joint_names[0] = "panda_finger_joint1";
    posture.joint_names[1] = "panda_finger_joint2";

    /* Set them as closed. */
    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.0;
    posture.points[0].positions[1] = 0.0;
    posture.points[0].time_from_start = ros::Duration(0.5);
    // END_SUB_TUTORIAL
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group)
{
    // BEGIN_SUB_TUTORIAL pick1
    // Create a vector of grasps to be attempted, currently only creating single grasp.
    // This is essentially useful when using a grasp generator to generate and test multiple grasps.
    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

    // Setting grasp pose
    // ++++++++++++++++++++++
    // This is the pose of panda_link8. |br|
    // Make sure that when you set the grasp_pose, you are setting it to be the pose of the last link in
    // your manipulator which in this case would be `"panda_link8"` You will have to compensate for the
    // transform from `"panda_link8"` to the palm of the end effector.
    grasps[0].grasp_pose.header.frame_id = "panda_link0";
    tf2::Quaternion orientation;
    orientation.setRPY(-tau / 4, -tau / 8, -tau / 4);
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasps[0].grasp_pose.pose.position.x = 0.4;
    grasps[0].grasp_pose.pose.position.y = 0;
    grasps[0].grasp_pose.pose.position.z = 0.5;

    // Setting pre-grasp approach
    // ++++++++++++++++++++++++++
    /* Defined with respect to frame_id */
    grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
    /* Direction is set as positive x axis */
    grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
    grasps[0].pre_grasp_approach.min_distance = 0.095;
    grasps[0].pre_grasp_approach.desired_distance = 0.115;

    // Setting post-grasp retreat
    // ++++++++++++++++++++++++++
    /* Defined with respect to frame_id */
    grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
    /* Direction is set as positive z axis */
    grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
    grasps[0].post_grasp_retreat.min_distance = 0.00;//0.1
    grasps[0].post_grasp_retreat.desired_distance = 0.01;//0.25

    grasps[0].max_contact_force = 15;

    // Setting posture of eef before grasp
    // +++++++++++++++++++++++++++++++++++
    openGripper(grasps[0].pre_grasp_posture);
    // END_SUB_TUTORIAL

    // BEGIN_SUB_TUTORIAL pick2
    // Setting posture of eef during grasp
    // +++++++++++++++++++++++++++++++++++
    closedGripper(grasps[0].grasp_posture);
    // END_SUB_TUTORIAL

    // BEGIN_SUB_TUTORIAL pick3
    // Set support surface as table1.
    move_group.setSupportSurfaceName("table1");
    // Call pick to pick up the object using the grasps given
    move_group.pick("object", grasps);
    // END_SUB_TUTORIAL
}

void place(moveit::planning_interface::MoveGroupInterface& group)
{
    // BEGIN_SUB_TUTORIAL place
    // location in verbose mode." This is a known issue. |br|
    // |br|
    // Ideally, you would create a vector of place locations to be attempted although in this example, we only create
    // a single place location.
    std::vector<moveit_msgs::PlaceLocation> place_location;
    place_location.resize(1);

    // Setting place location pose
    // +++++++++++++++++++++++++++
    place_location[0].place_pose.header.frame_id = "panda_link0";
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, tau / 4);  // A quarter turn about the z-axis
    place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

    /* For place location, we set the value to the exact location of the center of the object. */
    place_location[0].place_pose.pose.position.x = 0;
    place_location[0].place_pose.pose.position.y = 0.5;
    place_location[0].place_pose.pose.position.z = 0.5;

    // Setting pre-place approach
    // ++++++++++++++++++++++++++
    /* Defined with respect to frame_id */
    place_location[0].pre_place_approach.direction.header.frame_id = "panda_link0";
    /* Direction is set as negative z axis */
    place_location[0].pre_place_approach.direction.vector.z = -1.0;
    place_location[0].pre_place_approach.min_distance = 0.095;
    place_location[0].pre_place_approach.desired_distance = 0.115;

    // Setting post-grasp retreat
    // ++++++++++++++++++++++++++
    /* Defined with respect to frame_id */
    place_location[0].post_place_retreat.direction.header.frame_id = "panda_link0";
    /* Direction is set as negative y axis */
    place_location[0].post_place_retreat.direction.vector.y = -1.0;
    place_location[0].post_place_retreat.min_distance = 0.1;
    place_location[0].post_place_retreat.desired_distance = 0.25;

    // Setting posture of eef after placing object
    // +++++++++++++++++++++++++++++++++++++++++++
    /* Similar to the pick case */
    openGripper(place_location[0].post_place_posture);

    // Set support surface as table2.
    group.setSupportSurfaceName("table2");
    // Call place to place the object using the place locations given.
    group.place("object", place_location);
    // END_SUB_TUTORIAL
}


std::vector<std::string> addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface) {
    // BEGIN_SUB_TUTORIAL table1
    //
    // Creating Environment
    // ^^^^^^^^^^^^^^^^^^^^
    // Create vector to hold 3 collision objects.
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    std::vector<std::string> object_ids;
    collision_objects.resize(3);

    // Add the first table where the cube will originally be kept.
    collision_objects[0] = createBox("table1", "panda_link0", 0.2,  0.4,  0.4);
    setCollisionObjectPose(collision_objects[0], {0.5, 0, 0.2}, {0.0, 0.0, 0.0});
    object_ids.push_back(collision_objects[0].id);
    collision_objects[0].operation = collision_objects[0].ADD;

    // Add the second table where we will be placing the cube.
    collision_objects[1] = createBox("table2", "panda_link0", 0.4,  0.2,  0.4);
    setCollisionObjectPose(collision_objects[1], {0.0, 0.5, 0.2}, {0.0, 0.0, 0.0});
    object_ids.push_back(collision_objects[1].id);
    collision_objects[1].operation = collision_objects[1].ADD;

    // Define the object that we will be manipulating
    collision_objects[2] = createBox("object", "panda_link0", 0.02,  0.02,  0.2);
    setCollisionObjectPose(collision_objects[2], {0.5, 0, 0.5}, {0.0, 0.0, 0.0});
    object_ids.push_back(collision_objects[2].id);
    collision_objects[2].operation = collision_objects[2].ADD;

    planning_scene_interface.applyCollisionObjects(collision_objects);

    return object_ids;
}

