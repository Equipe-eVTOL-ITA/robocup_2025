#include <Eigen/Eigen>
#include <opencv2/highgui.hpp>
#include <chrono>
#include <thread>
#include <filesystem>
#include <ctime>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "Movement.hpp"
#include "PidController.hpp"
#include "Detection.hpp"

class PrecisionAlignState : public fsm::State {
public:
    PrecisionAlignState() : fsm::State(), drone(nullptr),
                            x_pid(0.9, 0.0, 0.05, 0.5),
                            y_pid(0.9, 0.0, 0.05, 0.5),
                            z_pid(0.9, 0.0, 0.05, 0.5),
                            yaw_pid(0.9, 0.0, 0.05, 0.5) {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone = blackboard.get<Drone>("drone");
        if (!drone)
            return;
        drone->log("STATE: PrecisionAlignState");

        Movement *movement = getNextMovement(blackboard.get<std::vector<Movement>>("movements"));

        if ((movement->getType() == "Rotate" && movement->getRotationDirection() == -1)  || movement->getType() == "Finished"){
            skip_align = true;
        }

        std::string type = movement->getType();
        if (type == "GoTo") {
            next_action = "GOTO NOW";
        }
        else if (type == "Rotate") {
            next_action = "ROTATE NOW";
        }
        else if (type == "Finished") {
            next_action = "FINISHED CHALLENGE";
        }
        else {
            drone->log("Invalid movement type.");
            return;
        }
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;

        if (skip_align){
            skip_align = false;
            return next_action;
        }

        float x_rate = 0.0, y_rate = 0.0, z_rate = 0.0, yaw_rate = 0.0;
        float vertical_distance = 1.0, angled_distance = 1.0;

        Detection verticalDetection(drone->getVerticalBboxes());
        Detection angledDetection(drone->getAngledBboxes());

        if (verticalDetection.isThereDetection()){
            DronePX4::BoundingBox vertical_bbox = verticalDetection.getClosestBbox();
            vertical_distance = verticalDetection.getMinDistance();

            x_rate = x_pid.compute(vertical_bbox.center_y);
            y_rate = y_pid.compute(vertical_bbox.center_x);
        }

        if (angledDetection.isThereDetection()){
            DronePX4::BoundingBox angled_bbox = angledDetection.getClosestBbox();
            angled_distance = angledDetection.getMinDistance();

            z_rate = z_pid.compute(angled_bbox.center_y);
            yaw_rate = yaw_pid.compute(angled_bbox.center_x);
        }

        if (vertical_distance < 0.03 && angled_distance < 0.03){
            this->captureAndSaveImages();
            return next_action;
        }

        // Control drone based on PID
        float yaw = drone->getOrientation()[2];
        float frd_x_rate = x_rate * cos(yaw) - y_rate * sin(yaw);
        float frd_y_rate = x_rate * sin(yaw) + y_rate * cos(yaw);
        drone->setLocalVelocity(frd_x_rate, -frd_y_rate, -z_rate, -yaw_rate);

        return "";
    }

private:
    Drone* drone;
    std::string next_action;
    int waypoints_visited;
    bool skip_align = false;

    PidController x_pid, y_pid, z_pid, yaw_pid;

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