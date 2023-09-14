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
    SceneGeometry(int size) : size_(size) {
        // Initialize the bounding boxes vector with 'size' default-constructed bounding boxes
        boundingBoxes_.resize(size_);
        // Initialize the transformations vector with 'size' default-constructed Eigen::Affine3d
        transforms_.resize(size_);
    }
    void resize_scene(int new_size){
        size_ = new_size;
        boundingBoxes_.resize(size_);
        // Initialize the transformations vector with 'size' default-constructed Eigen::Affine3d
        transforms_.resize(size_);
    }

    // Getters for bounding boxes and transformations
    const std::vector<BoundingBox>& getBoundingBoxes() const { return boundingBoxes_; }
    std::vector<BoundingBox>& getBoundingBoxes() { return boundingBoxes_; }

    const std::vector<Eigen::Affine3d>& getTransforms() const { return transforms_; }
    std::vector<Eigen::Affine3d>& getTransforms() { return transforms_; }

    int getSize() const { return size_; }

    //callbacks

    void bbox_callback(const moveit_msgs::PlanningSceneWorldConstPtr& msg);
    void transform_callback(const tf2_msgs::TFMessageConstPtr& msg);

private:
    std::vector<BoundingBox> boundingBoxes_;
    std::vector<Eigen::Affine3d> transforms_;
    int size_;
};

#endif //GOAL_STATE_PUBLISHER_SCENE_GEOMETRY_H
