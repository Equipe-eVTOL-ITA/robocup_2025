#include <Eigen/Eigen>
#include <opencv2/highgui.hpp>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include <chrono>
#include <thread>
#include <cmath>

class RotateState : public fsm::State {
public:
    RotateState(): fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        
        drone = blackboard.get<Drone>("drone");
        if (!drone) return;
        drone->log("STATE: RotateState");

        angular_velocity = 0.2; // rad/s

        movement = getNextMovement(blackboard.get<std::vector<Movement>>("movements"));
        double total_angle = movement->getTotalAngle();
        rotation_direction = movement->getRotationDirection();

        drone->log("Rotating " + std::to_string(rotation_direction * total_angle * 180 / M_PI) + " degrees.");

        rotation_time = total_angle / angular_velocity;
        start_time = std::chrono::steady_clock::now();
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;

        auto now = std::chrono::steady_clock::now();
        double elapsed_time = std::chrono::duration_cast<std::chrono::duration<double>>(now - start_time).count();

        if (elapsed_time >= rotation_time) {
            captureAndSaveImages();
            return "FINISHED ROTATION";
        }

        drone->setLocalVelocity(0, 0, 0, rotation_direction * angular_velocity);

        return "";
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void) blackboard;
        movement->setFinished();
    }

private:
    Drone* drone;
    double total_angle, rotation_time, angular_velocity;
    int rotation_direction;
    std::chrono::steady_clock::time_point start_time;
    Movement *movement;

    void captureAndSaveImages() {
        auto angledImg = drone->getAngledImage();
        auto verticalImg = drone->getVerticalImage();

        // Generate a timestamp.
        auto now = std::chrono::system_clock::now();
        auto timeNow = std::chrono::system_clock::to_time_t(now);
        char timestamp[32];
        std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", std::localtime(&timeNow));

        // Create bucket_detections folder.
        std::filesystem::create_directory("bucket_detections");

        // Save images in the "bucket_detections" folder with descriptive names.
        std::string angledFilename = std::string("bucket_detections/angled_") + timestamp + ".png";
        std::string verticalFilename = std::string("bucket_detections/vertical_") + timestamp + ".png";
        cv::imwrite(angledFilename, angledImg->image);
        cv::imwrite(verticalFilename, verticalImg->image);
    }
};
