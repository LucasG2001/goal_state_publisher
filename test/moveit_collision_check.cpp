#include <ros/ros.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/collision_detection/collision_common.h>

void planningSceneCallback(const moveit_msgs::PlanningSceneConstPtr& scene_msg, planning_scene::PlanningScenePtr& planning_scene) {
	// Set the received planning scene message to the planning scene pointer

	//moveit_msgs::AllowedCollisionMatrix allowed_collision_matrix;
	//moveit_msgs::PlanningScene temp = *scene_msg;
	//planning_scene->getAllowedCollisionMatrix().getMessage(allowed_collision_matrix);
	//temp.allowed_collision_matrix = allowed_collision_matrix;
	//planning_scene->setPlanningSceneMsg(*scene_msg);
	planning_scene->processPlanningSceneWorldMsg(scene_msg->world);


	// Additional collision checking logic...
}
int main(int argc, char** argv) {
	ros::init(argc, argv, "collision_checking_subscribe");
	ros::NodeHandle node_handle;
	// Create a PlanningSceneMonitor
	planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
	// Start scene monitor
	planning_scene_monitor->startSceneMonitor();
	planning_scene_monitor->startWorldGeometryMonitor();
	planning_scene_monitor->startStateMonitor();
	ROS_INFO("started planning scene monitor");

	// Subscribe to the planning scene topic
	ros::Subscriber planning_scene_subscriber = node_handle.subscribe<moveit_msgs::PlanningScene>(
			"/move_group/monitored_planning_scene", 1,
			boost::bind(planningSceneCallback, _1, planning_scene_monitor->getPlanningScene()));

	// Publisher for the modified planning scene
	ros::Publisher planning_scene_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("/test_planning_scene", 1);

	ROS_INFO("set planning scene");
	while(ros::ok){
		// Check for collisions in the received scene
		// Get the collision objects
		const std::vector<std::string>& collision_object_ids = planning_scene_monitor->getPlanningScene()->getWorld()->getObjectIds();
		// Print the number of collision objects
		ROS_INFO_STREAM("Number of collision objects: " << collision_object_ids.size());

		collision_detection::CollisionRequest collision_request;
		collision_request.contacts = true;
		collision_detection::CollisionResult collision_result;
		planning_scene_monitor->getPlanningScene()->checkCollision(collision_request, collision_result);
		std::cout << collision_result.contact_count << "\n";
		if (collision_result.collision) {
			ROS_WARN("Collision detected in the received scene");

			// Get the names of the colliding objects
			std::vector<std::string> objects_to_remove;
			const collision_detection::CollisionResult::ContactMap& contacts = collision_result.contacts;
			ROS_INFO_STREAM("reached for loop");
			for (const auto& contact : contacts) { // there seem to be no contacts (size is 0)
				ROS_INFO_STREAM("loop iteration");
				const std::string& object1 = contact.first.first;
				const std::string& object2 = contact.first.second;
				ROS_INFO_STREAM("Collision between: " << object1 << " and " << object2);
				// Add colliding objects to the removal list
				objects_to_remove.push_back(object1);
				objects_to_remove.push_back(object2);
			}

			// Remove the colliding objects from the planning scene
			moveit_msgs::CollisionObject remove_object;
			remove_object.operation = moveit_msgs::CollisionObject::REMOVE;
			ROS_INFO_STREAM("removing objects for loop");
			for (const auto& object : objects_to_remove) {
				ROS_INFO_STREAM("loop iteration removing");
				remove_object.id = object;
				planning_scene_monitor->getPlanningScene()->processCollisionObjectMsg(remove_object);
			}
		} else {
			ROS_INFO("No collision in the received scene");
		}

		moveit_msgs::PlanningScene modified_scene_msg;
		planning_scene_monitor->getPlanningScene()->getPlanningSceneMsg(modified_scene_msg);
		planning_scene_publisher.publish(modified_scene_msg);


		ros::spinOnce();
		ros::Rate(5).sleep();
	}


	return 0;
}
