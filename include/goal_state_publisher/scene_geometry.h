//
// Created by lucas on 14.09.23.
//

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <goal_state_publisher/bounding_box.h>
#include <tf2_msgs/TFMessage.h>
#include <moveit_msgs/PlanningScene.h>
#include <franka_gripper/GraspActionResult.h>
#include <franka_gripper/MoveActionResult.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

#ifndef GOAL_STATE_PUBLISHER_SCENE_GEOMETRY_H
#define GOAL_STATE_PUBLISHER_SCENE_GEOMETRY_H


class SceneGeometry {
public:
    // Constructor
    SceneGeometry(int size, double stiffness = 1.0, double Q=0.02) : size_(size), stiffness_(stiffness), Q_(Q){
        // Initialize the bounding boxes vector with 'size' default-constructed bounding boxes
        boundingBoxes_.resize(size_);
        // Initialize the transformations vector with 'size' default-constructed Eigen::Affine3d
        transforms_.resize(size_, Eigen::Affine3d::Identity());
        picked_object = &boundingBoxes_[0];
    }
    void resize_scene(size_t new_size){
        size_ = new_size;
        boundingBoxes_.resize(size_);
        // Initialize the transformations vector with 'size' default-constructed Eigen::Affine3d
        transforms_.resize(size_);
    }

    // Getters for bounding boxes and transformations, stiffness an size
    std::vector<BoundingBox> getBoundingBoxes() { return boundingBoxes_; }
    std::vector<Eigen::Affine3d> getTransforms() { return transforms_; }
    BoundingBox* picked_object; //pointer to currently grasped bounding box
    std::vector<double> grasp_diff = {0.0, 0.0, 0.0};
    int grasped_index;
    size_t getSize() const { return size_; }
    double get_stiffness() const { return stiffness_; }
    ros::Publisher planning_scene_publisher;
    ros::Subscriber goal_pose_subscriber;
    bool has_grasped = false;


    //callbacks
    void planning_scene_callback(const moveit_msgs::PlanningSceneConstPtr & msg);
    void bbox_callback(const moveit_msgs::PlanningSceneWorldConstPtr& msg);
    void transform_callback(const tf2_msgs::TFMessageConstPtr& msg);
    Eigen::Vector3d compute_force(const Eigen::Vector3d& global_ee_position, double exponent);
    static double get_nearest_point_coordinate(Eigen::Vector3d ee_pos, std::vector<double> x_extensions, int dimension);
    void graspFeedbackCallback(const franka_gripper::GraspActionResultConstPtr & result);
    void moveFeedbackCallback(const franka_gripper::MoveActionResultConstPtr & result);
    void pick_and_place_callback(const geometry_msgs::PoseStampedConstPtr & target_pose);

private:
    std::vector<BoundingBox> boundingBoxes_;
    std::vector<Eigen::Affine3d> transforms_;
    size_t size_;
    double stiffness_;
    double Q_; //critical radius (outside of this radius w.r.t the nearest point no force field will be active)
};

#endif //GOAL_STATE_PUBLISHER_SCENE_GEOMETRY_H
