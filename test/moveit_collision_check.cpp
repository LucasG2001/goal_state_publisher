#include <ros/ros.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/collision_detection/collision_common.h>

void planningSceneCallback(const moveit_msgs::PlanningSceneConstPtr& scene_msg, planning_scene::PlanningScenePtr& planning_scene) {
	// Set the received planning scene message to the planning scene pointer
	auto start = std::chrono::steady_clock::now();

	// Your code snippet here
	planning_scene->processPlanningSceneWorldMsg(scene_msg->world);
	const std::vector<std::string>& collision_object_ids = planning_scene->getWorld()->getObjectIds();
	planning_scene->setPlanningSceneMsg(*scene_msg);
	// Print the number of collision objects
	ROS_INFO_STREAM("Number of collision objects: " << collision_object_ids.size());
	ROS_INFO_STREAM("Number of collision objects received: " << scene_msg->world.collision_objects.size());

	collision_detection::CollisionRequest collision_request;
	collision_request.contacts = true;
	collision_request.max_contacts = 100;
	collision_detection::CollisionResult collision_result;
	planning_scene->checkCollision(collision_request, collision_result);
	if (collision_result.collision) {
		//ROS_WARN("Collision detected in the received scene");

		// Get the names of the colliding objects
		std::vector<std::string> objects_to_remove;
		const collision_detection::CollisionResult::ContactMap& contacts = collision_result.contacts;
		for (const auto& contact : contacts) { // there seem to be no contacts (size is 0)
			ROS_INFO_STREAM("loop iteration");
			const std::string& object1 = contact.first.first;
			const std::string& object2 = contact.first.second;
			//TODO: Loop is left when it wants to delete a panda link -> how do we except this?
			ROS_INFO_STREAM("Collision between: " << object1 << " and " << object2);
			// Add colliding objects to the removal list
			if (object1.find("panda") == std::string::npos){
					objects_to_remove.push_back(object1);
			}
			if (object2.find("panda") == std::string::npos){
				objects_to_remove.push_back(object2);
			}
		}
		// Remove the colliding objects from the planning scene
		moveit_msgs::CollisionObject remove_object;
		remove_object.operation = moveit_msgs::CollisionObject::REMOVE;
		for (const auto& object : objects_to_remove) {
			//ROS_INFO_STREAM("loop iteration removing");
			remove_object.id = object;
			planning_scene->processCollisionObjectMsg(remove_object);
		}
	} else {
		ROS_INFO("No collision in the received scene");
	}

	auto end = std::chrono::steady_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
	std::cout << "Callback took time: " << duration << " microseconds" << std::endl;

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
			"/segmented_planning_scene", 1,
			boost::bind(planningSceneCallback, _1, planning_scene_monitor->getPlanningScene()));

	// Publisher for the modified planning scene
	ros::Publisher planning_scene_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("/move_group/monitored_planning_scene", 1);

	ROS_INFO("set planning scene");
	moveit_msgs::PlanningScene modified_scene_msg;
	while(ros::ok){
		// Check for collisions in the received scene
		// Get the collision objects

		planning_scene_monitor->getPlanningScene()->getPlanningSceneMsg(modified_scene_msg);
		modified_scene_msg.is_diff = false;
		planning_scene_publisher.publish(modified_scene_msg);

		ros::spinOnce();
		ros::Rate(2).sleep();
	}


	return 0;
}
