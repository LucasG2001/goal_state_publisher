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
    // Default Constructor
    BoundingBox() {
        // Initialize x_bounds_, y_bounds_, and z_bounds_ with unit vectors
        x_bounds_ = {0.0, 1.0};
        y_bounds_ = {0.0, 1.0};
        z_bounds_ = {0.0, 1.0};
    }
    // Getters for x_bounds, y_bounds, and z_bounds
    const std::vector<double>& getXBounds() const { return x_bounds_; }
    const std::vector<double>& getYBounds() const { return y_bounds_; }
    const std::vector<double>& getZBounds() const { return z_bounds_; }

    // Setters for x_bounds, y_bounds, and z_bounds
    void setXBounds(const std::vector<double> & x_bounds) { x_bounds_ = x_bounds; }
    void setYBounds(const std::vector<double> & y_bounds) { y_bounds_ = y_bounds; }
    void setZBounds(const std::vector<double> & z_bounds) { z_bounds_ = z_bounds; }

private:
    std::vector<double> x_bounds_;
    std::vector<double> y_bounds_;
    std::vector<double> z_bounds_;
};