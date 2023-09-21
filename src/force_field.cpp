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
#include <franka_msgs/FrankaState.h>
#include <moveit_msgs/PlanningScene.h>

#define DIMENSION_X 0
#define DIMENSION_Y 1
#define DIMENSION_Z 2

Eigen::Vector3d global_EE_position;

//ToDo (OPTIONAL) add subscriber for moveit planning scene or similar to simulate robot movement for work at home
void ee_callback(const franka_msgs::FrankaStateConstPtr & msg){
    // for the time being we only need ee positions without orientations
    //ROS_INFO("got new end effector position");
    global_EE_position.x() = msg->O_T_EE[12]; // O_T_EE is a float[16] array in COLUMN MAJOR format!
    global_EE_position.y() = msg->O_T_EE[13];
    global_EE_position.z() = msg->O_T_EE[14];
}

void SceneGeometry::bbox_callback(const moveit_msgs::PlanningSceneWorldConstPtr& msg){
    /*
     * This callback handles the reception of the initial planning scene directly after segmenting
     */
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

void SceneGeometry::planning_scene_callback(const moveit_msgs::PlanningSceneConstPtr & msg){
    /*
     * This callback handles the reception of planning scene updates under /move_group/monitored_planning_scene
     * it is also somewhat redundant with the "bbox callback"
     */
    if(msg->is_diff == true){ return; }
    size_t size = msg->world.collision_objects.size();
    this->resize_scene(size);
    double x_bound;
    double y_bound;
    double z_bound;
    for (int k = 0; k < size; k++){
        x_bound = msg->world.collision_objects[k].primitives[0].dimensions[0] * 0.5;
        y_bound = msg->world.collision_objects[k].primitives[0].dimensions[1] * 0.5;
        z_bound = msg->world.collision_objects[k].primitives[0].dimensions[2] * 0.5;

        this->boundingBoxes_[k].setXBounds({-x_bound, x_bound});
        this->boundingBoxes_[k].setYBounds({-y_bound, y_bound});
        this->boundingBoxes_[k].setZBounds({-z_bound, z_bound});

        //state transforms according to box translation
        Eigen::Vector3d translation;
        Eigen::Quaterniond rotation;
        //note that we need the inverse transform of the object's pose so to say
        translation.x() = msg->world.collision_objects[k].pose.position.x;
        translation.y() = msg->world.collision_objects[k].pose.position.y;
        translation.z() = msg->world.collision_objects[k].pose.position.z;
        rotation.coeffs() << msg->world.collision_objects[k].pose.orientation.x, msg->world.collision_objects[k].pose.orientation.y,
                msg->world.collision_objects[k].pose.orientation.z, msg->world.collision_objects[k].pose.orientation.w;

        Eigen::MatrixXd R_matrix = rotation.toRotationMatrix();
        Eigen::Affine3d ee_transform; //this should transform the end effector into the same reference frame as the aligned bounding box
        //ee_transform = Eigen::Translation3d(R_matrix.transpose() * translation) * R_matrix.transpose();
        ee_transform = Eigen::Translation3d(-1 * R_matrix.transpose() * translation) * R_matrix.transpose();
        this->transforms_[k] = ee_transform;
        std::cout << "resulting translation is " << ee_transform.translation().array() << std::endl;
        std::cout << "resulting translation is " << ee_transform.rotation().array() << std::endl;
        /*
         * Check if we don't need to transform the bounding boxes coming in from this message
         * UPDATE: We don't need to I think. They are assumed at 0 0 0 with their dimension only.
         * it suffices to transform the end-effector
         * CHECK also if the correct transformation (and when) is the inverse or just R,t. I think it should be R,t always
         * as we set by definition our boxes to be at 0 0 0
         * */
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
                msg->transforms[k].transform.rotation.z, msg->transforms[k].transform.rotation.w;

        Eigen::Affine3d ee_transform; //this should transform the end effector into the same reference frame as the aligned bounding box
        ee_transform = Eigen::Translation3d(position)* orientation.toRotationMatrix();

        this->transforms_[k] = ee_transform;

    }
}

Eigen::Vector3d SceneGeometry::compute_force(const Eigen::Vector3d& global_ee_position, double exponent){
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
        ROS_INFO_STREAM("transformed end effector position to " << local_ee_position);
        //compute nearest point on BBOX (4.3)
        Eigen::Vector3d projected_point; //this is the nearest point in the corresponding bounding box
        projected_point.x() = get_nearest_point_coordinate(local_ee_position, x_bound, DIMENSION_X);
        projected_point.y() = get_nearest_point_coordinate(local_ee_position, y_bound, DIMENSION_Y);
        projected_point.z() = get_nearest_point_coordinate(local_ee_position, z_bound, DIMENSION_Z);
        //compute repulsive Force and add up to the total
        Eigen::Vector3d effective_distance = local_ee_position - projected_point;
        ROS_INFO_STREAM("effective distance is " << (transforms_[i].rotation().transpose() * effective_distance).transpose());
        double D = effective_distance.norm();
        if (D <= Q_){
            Eigen::Vector3d nabla_D = effective_distance * (1/D); //unit distance vector
            //F_res += stiffness_ * (1/D - 1/Q_) * (1/std::pow(D, 2)) * nabla_D;
            F_res += transforms_[i].rotation().transpose() *
                    (stiffness_ * std::pow((1/D - 1/Q_), exponent) * nabla_D); //transform back to global frame global_F = T_lg^-1 * l_F
        }
        else{ F_res += 0 * F_res; }
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
    double stiffness = 1;  // Default value
    double Q = 0.2;         // Default value
    double publishing_frequency = 50;
    double exponent = 0.8;
    if (argc >= 4) {
        try {
            stiffness = std::stod(argv[1]);
            Q = std::stod(argv[2]);
            exponent = std::stod(argv[3]);
            publishing_frequency = std::stod(argv[4]);

        } catch (const std::exception& e) {
            std::cerr << "Error parsing arguments: " << e.what() << std::endl;
            return 1;  // Return an error code
        }
    }

    //node functionality
    ros::init(argc, argv, "force_field");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(4);
    spinner.start();
    SceneGeometry aligned_geometry(1, stiffness, Q);
    ROS_INFO("set up node");
    global_EE_position << 0.2, 0.0, 0.5;
    // Create a ros::Rate object to control the loop rate
    ros::Rate loop_rate(publishing_frequency);
    //subscribers
    //1) subscribe to ee-pos
    ros::Subscriber ee_pose = n.subscribe("/franka_state_controller/franka_states", 10, ee_callback);
    //2) and transforms and bbox_subscriber
    ros::Subscriber bbox_subscriber = n.subscribe("/force_bboxes", 10, &SceneGeometry::bbox_callback, &aligned_geometry);
    ros::Subscriber planning_scene_subscriber = n.subscribe("/move_group/monitored_planning_scene", 10, &SceneGeometry::planning_scene_callback, &aligned_geometry);
    ros::Subscriber transforms = n.subscribe("/ee_transforms", 10, &SceneGeometry::transform_callback, &aligned_geometry);
    // force field publisher (3)
    ros::Publisher force_publisher = n.advertise<geometry_msgs::Vector3>("/resulting_force", 10);
    ROS_INFO("set up publishers and subscribers");
    geometry_msgs:: Vector3 resulting_force_msg;


    Eigen::Vector3d ee_pos;
    Eigen::Vector3d projected_point;
    Eigen::Vector3d F_res;
    F_res << 0, 0, 0;
    while (ros::ok()){
        size_t size = aligned_geometry.getSize();
        std::cout << "Starting time measurement in loop. " <<  "size of loop is " << size << std::endl;
        auto start = std::chrono::high_resolution_clock ::now();
        //get bbox bounds
        F_res = aligned_geometry.compute_force(global_EE_position, exponent);
        ROS_INFO_STREAM("Resultant Force is " << F_res.transpose() << " N");
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