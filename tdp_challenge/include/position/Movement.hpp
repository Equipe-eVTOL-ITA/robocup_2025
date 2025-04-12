#pragma once

#include <cmath>


class Movement{
public:
    // Constructor for GoTo
    Movement(std::string type, Eigen::Vector3d position)
        : type_(std::move(type)), position_(position), total_angle_(0.0), rotation_direction_(1) {}

    // Constructor for Rotation
    Movement(std::string type, double total_angle, double rotation_direction)
        : type_(std::move(type)), position_(Eigen::Vector3d(0.0, 0.0, 0.0)), total_angle_(total_angle), rotation_direction_(rotation_direction) {}

    // Constructor for Finished
    Movement(std::string type)
        : type_(std::move(type)), position_(Eigen::Vector3d(0.0, 0.0, 0.0)), total_angle_(0.0), rotation_direction_(1) {}

        
    void setFinished() {
        is_finished_ = true;
    }

    bool isFinished() const {
        return is_finished_;
    }

    std::string getType() const {
        return type_;
    }

    Eigen::Vector3d getPosition() const {
        return position_;
    }

    double getTotalAngle() const {
        return total_angle_*M_PI/180.0;
    }

    double getRotationDirection() const {
        return rotation_direction_;
    }
    
private:
    std::string type_;
    Eigen::Vector3d position_;
    double total_angle_;
    int rotation_direction_;
    bool is_finished_{false};
};

Movement* getNextMovement(std::vector<Movement>* movements) {
    for (auto& movement : *movements) {
        if (!movement.isFinished()) {
            return &movement;
        }
    }
    return nullptr;
}