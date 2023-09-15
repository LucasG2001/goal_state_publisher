#include <Eigen/Dense>
#include <geometry_msgs/Transform.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <random>
#include <chrono>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32MultiArray.h>
#include <moveit_msgs/PlanningSceneWorld.h>
#include <goal_state_publisher/scene_geometry.h>
#include <tf2_msgs/TFMessage.h>

#define DIMENSION_X 0
#define DIMENSION_Y 1
#define DIMENSION_Z 2

Eigen::Vector3d global_EE_position;

void ee_callback(const geometry_msgs::Pose::ConstPtr& msg){
    // for the time being we only need ee positions without orientations
    ROS_INFO("got new end effector position");
    global_EE_position.x() = msg->position.x;
    global_EE_position.y() = msg->position.y;
    global_EE_position.z() = msg->position.z;
}

void SceneGeometry::bbox_callback(const moveit_msgs::PlanningSceneWorldConstPtr& msg){
    ROS_INFO("got new objects for force field planning scene");
    size_t size = msg->collision_objects.size();
    std::cout << "New planning scene has size of " << size << std::endl;
    this->resize_scene(size);
    double x_bound;
    double y_bound;
    double z_bound;
    for (int k = 0; k < size; k++){
       x_bound = msg->collision_objects[k].primitives[0].dimensions[0] * 0.5;
       y_bound = msg->collision_objects[k].primitives[0].dimensions[1] * 0.5;
       z_bound = msg->collision_objects[k].primitives[0].dimensions[2] * 0.5;

       this->boundingBoxes_[k].setXBounds({-x_bound, x_bound});
       this->boundingBoxes_[k].setYBounds({-y_bound, y_bound});
       this->boundingBoxes_[k].setZBounds({-z_bound, z_bound});
    }
}

void SceneGeometry::transform_callback(const tf2_msgs::TFMessageConstPtr& msg){
    ROS_INFO("got the transforms for EE");
    size_t size = msg->transforms.size();
    this->transforms_.resize(size);
    Eigen::Vector3d position;

    Eigen::Quaterniond orientation;
    for (int k = 0; k < size; k++){
         position.x() = msg->transforms[k].transform.translation.x;
         position.y() = msg->transforms[k].transform.translation.y;
         position.z() = msg->transforms[k].transform.translation.z;
         orientation.coeffs() << msg->transforms[k].transform.rotation.x, msg->transforms[k].transform.rotation.y,
                msg->transforms[k].transform.rotation.z, msg->transforms[k].transform.rotation.z;

        Eigen::Affine3d ee_transform; //this should transform the end effector into the same reference frame as the aligned bounding box
        ee_transform = Eigen::Translation3d(position)* orientation.toRotationMatrix();

        this->transforms_[k] = ee_transform;

    }
}

Eigen::Vector3d SceneGeometry::compute_force(const Eigen::Vector3d& global_ee_position){
    Eigen::Vector3d F_res;
    F_res << 0, 0, 0;
    for (int i=0; i<size_; i++){
        //get bbox bounds
        std::vector<double>  x_bound = boundingBoxes_[i].getXBounds();
        std::vector<double>  y_bound = boundingBoxes_[i].getYBounds();
        std::vector<double>  z_bound = boundingBoxes_[i].getZBounds();
        //transform ee-position into globally aligned coordinate frame (4.2)
        Eigen::Affine3d transform = transforms_[i];
        Eigen::Vector3d local_ee_position = transform * global_ee_position;
        ROS_INFO("transformed end effector position");
        //compute nearest point on BBOX (4.3)
        Eigen::Vector3d projected_point; //this is the nearest point in the corresponding bounding box
        projected_point.x() = get_nearest_point_coordinate(local_ee_position, x_bound, DIMENSION_X);
        projected_point.y() = get_nearest_point_coordinate(local_ee_position, y_bound, DIMENSION_Y);
        projected_point.z() = get_nearest_point_coordinate(local_ee_position, z_bound, DIMENSION_Z);
        ROS_INFO("projected point on box");
        //compute repulsive Force and add up to the total
        Eigen::Vector3d effective_distance = local_ee_position - projected_point;
        double D = effective_distance.norm();
        if (D <= Q_){
            Eigen::Vector3d nabla_D = effective_distance * (1/D); //unit distance vector
            F_res += -stiffness_ * (1/D - 1/Q_) * (1/std::pow(D, 2)) * nabla_D;
        }
        else{ F_res += 0 * F_res; }
        ROS_INFO_STREAM(F_res);
    }
    return F_res;
}
double SceneGeometry::get_nearest_point_coordinate(Eigen::Vector3d ee_pos, std::vector<double> x_extensions, int dimension){
    double x = ee_pos(dimension, 0);
    double coordinate;

    if (x < x_extensions[1] && x > x_extensions[0]){
        coordinate = x;
    }
    else if (x > x_extensions[1]){
        coordinate = x_extensions[1];
    }
    else if(x < x_extensions[0]){
        coordinate = x_extensions[0];
    }

    return coordinate;
}



int main(int argc, char **argv) {
    //node functionality
    ros::init(argc, argv, "force_field");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(4);
    spinner.start();
    SceneGeometry aligned_geometry(1, 2.0, 0.02);
    ROS_INFO("set up node");
    // Create a ros::Rate object to control the loop rate
    ros::Rate loop_rate(0.3333);
    //subscribers
    //1) subscribe to ee-pos
    ros::Subscriber ee_pose = n.subscribe("ee_pose", 10, ee_callback);
    //2) and transforms and bbox_subscriber
    ros::Subscriber bbox_subscriber = n.subscribe("/force_bboxes", 10, &SceneGeometry::bbox_callback, &aligned_geometry);
    ros::Subscriber transforms = n.subscribe("/ee_transforms", 10, &SceneGeometry::transform_callback, &aligned_geometry);
    // force field publisher (3)
    ros::Publisher force_publisher = n.advertise<geometry_msgs::Vector3>("resulting_force", 10);
    ROS_INFO("set up publishers and subscribers");
    geometry_msgs:: Vector3 resulting_force_msg;


    Eigen::Vector3d ee_pos;
    Eigen::Vector3d projected_point;
    Eigen::Vector3d F_res;
    F_res << 0, 0, 0;
    ee_pos << 0.3, 0.1, 0.5;
    while (ros::ok()){
        size_t size = aligned_geometry.getSize();
        std::cout << "Starting time measurement in loop. " <<  "size of loop is " << size << std::endl;
        auto start = std::chrono::high_resolution_clock ::now();
        //get bbox bounds
        F_res = aligned_geometry.compute_force(ee_pos);
        ROS_INFO_STREAM(F_res);
        resulting_force_msg.x = F_res.x();
        resulting_force_msg.y = F_res.y();
        resulting_force_msg.z = F_res.z();

        force_publisher.publish(resulting_force_msg);

        auto stop = std::chrono::high_resolution_clock::now();
        auto time = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        std::cout << "Time taken by update (whole loop): " << time.count() << " microseconds" << std::endl;
        loop_rate.sleep();
    }


    ros::waitForShutdown();

}