#include <Eigen/Eigen>
#include <opencv2/highgui.hpp>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "ArenaPoint.hpp"

class GoToPositionState : public fsm::State {
public:
    GoToPositionState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {

        drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;
        drone->log("STATE: GoToPositionState");

        std::vector<ArenaPoint>* waypoints = blackboard.get<std::vector<ArenaPoint>>("waypoints");

        yaw = drone->getOrientation()[2];

        int waypoints_visited = 0;
        for (auto& point : *waypoints) {
            waypoints_visited ++;
            if (!point.is_visited) {
                goal_ptr = &point;
                break;
            }
        }
        blackboard.set<int>("waypoints_visited", waypoints_visited);

    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        pos = drone->getLocalPosition();

        if ((pos-goal_ptr->coordinates).norm() < 0.08) {
            goal_ptr->is_visited = true;
            drone->log("Waypoint visited.");
            return "ARRIVED AT POINT";
        }
        
        Eigen::Vector3d goal = goal_ptr->coordinates;
        Eigen::Vector3d goal_diff = goal - pos;
        if (goal_diff.norm() > max_velocity) {
            goal_diff = goal_diff.normalized() * max_velocity;
        }
        goal = goal_diff + pos;

        drone->setLocalPosition(goal[0], goal[1], goal[2], yaw);

        return "";
    }

private:
    Eigen::Vector3d pos;
    Drone* drone;
    ArenaPoint* goal_ptr;
    const double max_velocity = 0.8; // m/s
    float yaw;
};