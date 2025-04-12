#include <Eigen/Eigen>
#include <opencv2/highgui.hpp>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "ArenaPoint.hpp"
#include <chrono>
#include <thread>
#include <filesystem>
#include <ctime>

class PrecisionAlignState : public fsm::State {
public:
    PrecisionAlignState() : fsm::State(), drone(nullptr) {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone = blackboard.get<Drone>("drone");
        if (!drone)
            return;
        drone->log("STATE: PrecisionAlignState");

        yaw = drone->getOrientation()[2];

        int waypoints_visited = *blackboard.get<int>("waypoints_visited");
        next_action = ((waypoints_visited == 1 || waypoints_visited == 6 || waypoints_visited == 7)
                       ? "ROTATE NOW"
                       : "NEXT BUCKET");
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        pos = drone->getLocalPosition();


        if (!aligned){
            // Align the drone to the target position.
            aligned = true;
            drone->log("Aligning to target position.");
            start_time = std::chrono::steady_clock::now();
        }

        else{
            auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - start_time).count();

            if (elapsed_time > 1) {
                captureAndSaveImages();
                drone->log("Images captured and saved.");
                return next_action;
            }
        }

        drone->setLocalPosition(pos[0], pos[1], pos[2], yaw);
        return "";
    }

private:
    Eigen::Vector3d pos;
    Drone* drone;
    std::string next_action;
    bool aligned = false;
    std::chrono::steady_clock::time_point start_time;
    float yaw;

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