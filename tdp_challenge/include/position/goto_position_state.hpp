#include <Eigen/Eigen>
#include <opencv2/highgui.hpp>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "Movement.hpp"

class GoToPositionState : public fsm::State {
public:
    GoToPositionState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {

        drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;
        drone->log("STATE: GoToPositionState");

        movement = getNextMovement(blackboard.get<std::vector<Movement>>("movements"));

        goal = movement->getPosition();
        yaw = drone->getOrientation()[2];
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        pos = drone->getLocalPosition();

        if ((pos-goal).norm() < 0.08) {
            drone->log("Waypoint visited.");
            return "ARRIVED AT POINT";
        }
        
        Eigen::Vector3d diff = goal - pos;
        Eigen::Vector3d little_goal = pos + (diff.norm() > max_velocity ? diff.normalized() * max_velocity : diff);

        drone->setLocalPosition(little_goal[0], little_goal[1], little_goal[2], yaw);

        return "";
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        movement->setFinished();
    }

private:
    Eigen::Vector3d pos;
    Drone* drone;
    Eigen::Vector3d goal;
    const double max_velocity = 0.8; // m/s
    float yaw;
    Movement* movement;
};