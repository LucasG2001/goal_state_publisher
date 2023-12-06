#ifndef ACTION_PRIMITIVE_H
#define ACTION_PRIMITIVE_H
#endif //ACTION_PRIMITIVE_H

#include <Eigen/Dense>

class ActionPrimitive {
public:
    // Constructor
    ActionPrimitive();

    // Pure virtual function - to be implemented by derived classes
    virtual void performAction() = 0;

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

    // Getters
    Eigen::Matrix<double, 6, 1> getStartPose() const;
    Eigen::Matrix<double, 6, 1> getGoalPose() const;
    Eigen::Matrix<double, 6, 1> getObjectPose() const;
    bool getGrasp() const;
    Eigen::Matrix<double, 6, 6> getSpringStiffness() const;
    Eigen::Matrix<double, 6, 6> getDamping() const;
    Eigen::Matrix<double, 6, 6> getInertia() const;
    Eigen::Matrix<double, 6, 6> getRepulsionStiffness() const;
    Eigen::Matrix<double, 6, 6> getRepulsionDamping() const;

protected:
    Eigen::Matrix<double, 6, 1> start_pose_;
    Eigen::Matrix<double, 6, 1> goal_pose_;
    Eigen::Matrix<double, 6, 1> object_pose_;
    bool grasp_;
    Eigen::Matrix<double, 6, 6> spring_stiffness_;
    Eigen::Matrix<double, 6, 6> damping_;
    Eigen::Matrix<double, 6, 6> inertia_;
    Eigen::Matrix<double, 6, 6> repulsion_stiffness_;
    Eigen::Matrix<double, 6, 6> repulsion_damping_;
};


