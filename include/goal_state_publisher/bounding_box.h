//
// Created by lucas on 14.09.23.
//
#include <iostream>
#include <tuple>

#ifndef GOAL_STATE_PUBLISHER_BOUNDING_BOX_H
#define GOAL_STATE_PUBLISHER_BOUNDING_BOX_H

#endif //GOAL_STATE_PUBLISHER_BOUNDING_BOX_H

class BoundingBox {
public:
    // Constructors
    /*
    BoundingBox(const std::tuple<double, double>& x_bounds,
                const std::tuple<double, double>& y_bounds,
                const std::tuple<double, double>& z_bounds)
            : x_bounds_(x_bounds), y_bounds_(y_bounds), z_bounds_(z_bounds) {}
    */
    // Getters for x_bounds, y_bounds, and z_bounds
    const std::tuple<double, double>& getXBounds() const { return x_bounds_; }
    const std::tuple<double, double>& getYBounds() const { return y_bounds_; }
    const std::tuple<double, double>& getZBounds() const { return z_bounds_; }

    // Setters for x_bounds, y_bounds, and z_bounds
    void setXBounds(const std::tuple<double, double>& x_bounds) { x_bounds_ = x_bounds; }
    void setYBounds(const std::tuple<double, double>& y_bounds) { y_bounds_ = y_bounds; }
    void setZBounds(const std::tuple<double, double>& z_bounds) { z_bounds_ = z_bounds; }

private:
    std::tuple<double, double> x_bounds_;
    std::tuple<double, double> y_bounds_;
    std::tuple<double, double> z_bounds_;
};