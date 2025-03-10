#ifndef ACTION_PRIMITIVE_H
#define ACTION_PRIMITIVE_H


#include <Eigen/Dense>
#include <ros/ros.h>
#include <TaskPlanner.h>
#include <custom_msgs/ImpedanceParameterMsg.h>

struct ImpedanceMatrices {
	Eigen::Matrix<double, 6, 6> spring_stiffness;
	Eigen::Matrix<double, 6, 6> damping;
	Eigen::Matrix<double, 6, 6> inertia;
	Eigen::Matrix<double, 6, 6> repulsion_stiffness;
	Eigen::Matrix<double, 6, 6> repulsion_damping;

	ImpedanceMatrices operator*(double scalar) const {
		ImpedanceMatrices result;
		result.spring_stiffness = spring_stiffness * scalar;
		result.damping = damping * scalar;
		result.inertia = inertia * scalar;
		result.repulsion_stiffness = repulsion_stiffness * scalar;
		result.repulsion_damping = repulsion_damping * scalar;
		return result;
	}
};

class ActionPrimitive {
public:
    // Constructor
    ActionPrimitive();
	Eigen::Vector3d pick_offset; //offset for object pickup

    // Pure virtual function - to be implemented by derived classes
    virtual void
    performAction(TaskPlanner &task_planner, ros::Publisher &goal_publisher, ros::Publisher &impedance_publisher,
                  ros::Publisher &is_task_finished_publisher) = 0;

    // Setters
    void setStartPose(const Eigen::Matrix<double, 6, 1>& start_pose);
    void setGoalPose(const Eigen::Matrix<double, 6, 1>& goal_pose);
    void setObjectPose(const Eigen::Matrix<double, 6, 1>& object_pose);
    void setGrasp(bool grasp);
    void setParameters(const Eigen::Matrix<double, 6, 6>& spring_stiffness,
                       const Eigen::Matrix<double, 6, 6>& damping,
                       const Eigen::Matrix<double, 6, 6>& inertia,
                       const Eigen::Matrix<double, 6, 6>& repulsion_stiffness,
                       const Eigen::Matrix<double, 6, 6>& repulsion_damping);
	ImpedanceMatrices getImpedanceParams();

    // Getters
    Eigen::Matrix<double, 6, 1> getStartPose() const;
    Eigen::Matrix<double, 6, 1> getGoalPose() const;
    Eigen::Matrix<double, 6, 1> getObjectPose() const;
	bool hasGrasped = false;
    Eigen::Matrix<double, 6, 6> getSpringStiffness() const;
    Eigen::Matrix<double, 6, 6> getDamping() const;
    Eigen::Matrix<double, 6, 6> getInertia() const;
	//ToDo: make matrix sizes of IMpedance parameters consistent over all nodes
	Eigen::Matrix<double, 6, 6> getRepulsionStiffness() const;
    Eigen::Matrix<double, 6, 6> getRepulsionDamping() const;
	void construct_impedance_message(const ImpedanceMatrices &impedance_matrices);

	custom_msgs::ImpedanceParameterMsg compliance_update;

protected:
    Eigen::Matrix<double, 6, 1> start_pose_;
    Eigen::Matrix<double, 6, 1> goal_pose_;
    Eigen::Matrix<double, 6, 1> object_pose_;
    bool grasp_;
    ImpedanceMatrices impedance_params;
};

#endif //ACTION_PRIMITIVE_H
