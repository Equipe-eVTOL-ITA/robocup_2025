#include "position/initial_takeoff_state.hpp"
#include "position/landing_state.hpp"
#include "position/return_home_state.hpp"
#include "position/precision_align_state.hpp"
#include "position/rotate_state.hpp"
#include "position/goto_position_state.hpp"
#include "position/ArenaPoint.hpp"
#include <rclcpp/rclcpp.hpp>


#include <memory>
#include <iostream>


class PositionFSM : public fsm::FSM {
public:
    PositionFSM() : fsm::FSM({"ERROR", "FINISHED"}) {

        this->blackboard_set<Drone>("drone", new Drone());
        Drone* drone = blackboard_get<Drone>("drone");

        const Eigen::Vector3d fictual_home = Eigen::Vector3d({1.2, -1.0, -0.6});
        drone->setHomePosition(fictual_home);

        const Eigen::Vector3d home_pos = drone->getLocalPosition(); 
        const Eigen::Vector3d orientation = drone->getOrientation();

        this->blackboard_set<Eigen::Vector3d>("home_position", home_pos);
        this->blackboard_set<bool>("finished_bases", false);
        this->blackboard_set<float>("initial_yaw", orientation[2]);

        // ARENA POINTS
        std::vector<ArenaPoint> waypoints;
        float takeoff_height = -2.3;
        waypoints.push_back({Eigen::Vector3d({1.0, -7.0, takeoff_height})});
        waypoints.push_back({Eigen::Vector3d({3.0, -7.0, takeoff_height})});
        waypoints.push_back({Eigen::Vector3d({3.0, -1.0, takeoff_height})});
        waypoints.push_back({Eigen::Vector3d({5.0, -1.0, takeoff_height})});
        waypoints.push_back({Eigen::Vector3d({5.0, -7.0, takeoff_height})});
        waypoints.push_back({Eigen::Vector3d({6.0, -7.0, takeoff_height})});
        waypoints.push_back({Eigen::Vector3d({6.0, -1.0, takeoff_height})});
        this->blackboard_set<std::vector<ArenaPoint>>("waypoints", waypoints);
        this->blackboard_set<float>("takeoff_height", takeoff_height);


        this->add_state("INITIAL TAKEOFF", std::make_unique<InitialTakeoffState>());
        this->add_state("GO TO POSITION", std::make_unique<GoToPositionState>());
        this->add_state("PRECISION ALIGN", std::make_unique<PrecisionAlignState>());
        this->add_state("LANDING", std::make_unique<LandingState>());
        this->add_state("ROTATE", std::make_unique<RotateState>());
        this->add_state("RETURN HOME", std::make_unique<ReturnHomeState>());

        // Initial Takeoff transitions
        this->add_transitions("INITIAL TAKEOFF", {{"INITIAL TAKEOFF COMPLETED", "GO TO POSITION"},{"SEG FAULT", "ERROR"}});

        // Go To Position transitions
        this->add_transitions("GO TO POSITION", {{"ARRIVED AT POINT", "PRECISION ALIGN"},{"SEG FAULT", "ERROR"}});

        // Precision Align transitions
        this->add_transitions("PRECISION ALIGN", {
                                                    {"ALIGNED TO FIRST BUCKET", "ROTATE"},
                                                    {"ALIGNED TO LAST BUCKET", "GO TO POSITION"},
                                                    {"FINISHED BUCKETS", "RETURN HOME"},
                                                    {"SEG FAULT", "ERROR"}});

        // Rotate transitions
        this->add_transitions("ROTATE", {{"FINISHED ROTATION", "PRECISION ALIGN"},{"SEG FAULT", "ERROR"}});

        // Return Home transitions
        this->add_transitions("RETURN HOME", {{"AT HOME", "LANDING"},{"SEG FAULT", "ERROR"}});

        // Landing transitions
        this->add_transitions("LANDING", {{"LANDED", "FINISHED"},{"SEG FAULT", "ERROR"}});
        
    }
};

class NodeFSM : public rclcpp::Node {
public:
    NodeFSM() : rclcpp::Node("position_node"), my_fsm() {
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),  // Run at approximately 20 Hz
            std::bind(&NodeFSM::executeFSM, this));
    }

    void executeFSM() {
        if (rclcpp::ok() && !my_fsm.is_finished()) {
            my_fsm.execute();
        } else {
            rclcpp::shutdown();
        }
    }

private:
    PositionFSM my_fsm;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, const char *argv[]) {
    rclcpp::init(argc, argv);

    auto my_node = std::make_shared<NodeFSM>();
    rclcpp::spin(my_node);

    return 0;
}
