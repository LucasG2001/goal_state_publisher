//
// Created by lucas on 14.09.23.
//

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <goal_state_publisher/bounding_box.h>
#include <tf2_msgs/TFMessage.h>

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
    size_t getSize() const { return size_; }
    double get_stiffness() const { return stiffness_; }

    //callbacks

    void bbox_callback(const moveit_msgs::PlanningSceneWorldConstPtr& msg);
    void transform_callback(const tf2_msgs::TFMessageConstPtr& msg);
    Eigen::Vector3d compute_force(const Eigen::Vector3d& global_ee_position);
    static double get_nearest_point_coordinate(Eigen::Vector3d ee_pos, std::vector<double> x_extensions, int dimension);

private:
    std::vector<BoundingBox> boundingBoxes_;
    std::vector<Eigen::Affine3d> transforms_;
    size_t size_;
    double stiffness_;
    double Q_; //critical radius (outside of this radius w.r.t the nearest point no force field will be active)
};

#endif //GOAL_STATE_PUBLISHER_SCENE_GEOMETRY_H
