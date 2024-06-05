#ifndef ATOMIC_TASKS_H
#define ATOMIC_TASKS_H

#include "ActionPrimitive.h"
#include <TaskPlanner.h>
#include <ros/ros.h>

class GetMe : public ActionPrimitive {
public:
	// Constructor
	GetMe();
	// Additional methods or overrides specific to GetMeTask
	void performAction(TaskPlanner &task_planner, ros::Publisher &goal_publisher, ros::Publisher &impedance_publisher,
	                   ros::Publisher &is_task_finished_publisher) override;

};

class FollowMe : public ActionPrimitive {
public:
	// Constructor
	FollowMe();
	// Additional methods or overrides specific to FollowMeTask
	void performAction(TaskPlanner &task_planner, ros::Publisher &goal_publisher, ros::Publisher &impedance_publisher,
	                   ros::Publisher &is_task_finished_publisher) override;
	//offset for the "leash"
	Eigen::Matrix<double, 6, 1> fixed_offset;
	Eigen::Matrix<double, 6, 1> hand_pose;
};

class HoldThis : public ActionPrimitive {
public:
	// Constructor
	HoldThis();
	// Additional methods or overrides specific to HoldThisTask
	void performAction(TaskPlanner &task_planner, ros::Publisher &goal_publisher, ros::Publisher &impedance_publisher,
	                   ros::Publisher &is_task_finished_publisher) override;

	bool free_float;
	bool is_constrained;  // tells us if we free float but with direction constraints
};

class TakeThis : public ActionPrimitive {
public:
	// Constructor
	TakeThis();
	// Additional methods or overrides specific to TakeThisTask
	void performAction(TaskPlanner &task_planner, ros::Publisher &goal_publisher, ros::Publisher &impedance_publisher,
	                   ros::Publisher &is_task_finished_publisher) override;
};

class AvoidMe : public ActionPrimitive {
public:
	// Constructor
	AvoidMe();
	// Additional methods or overrides specific to AvoidMeTask
	void performAction(TaskPlanner &task_planner, ros::Publisher &goal_publisher, ros::Publisher &impedance_publisher,
	                   ros::Publisher &is_task_finished_publisher) override;
};

#endif // ATOMIC_TASKS_H
