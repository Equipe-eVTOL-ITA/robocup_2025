#include "position/initial_takeoff_state.hpp"
#include "position/landing_state.hpp"
#include "position/precision_align_state.hpp"
#include "position/rotate_state.hpp"
#include "position/goto_position_state.hpp"
#include "position/Movement.hpp"
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <iostream>


class PositionFSM : public fsm::FSM {
public:
    PositionFSM() : fsm::FSM({"ERROR", "FINISHED"}) {

        this->blackboard_set<Drone>("drone", new Drone());

        // MOVEMENTS
        std::vector<Movement> movements;
        float distance_s = 3.0;
        movements.push_back(Movement("GoTo", Eigen::Vector3d({distance_s, 0.0, -distance_s})));
        movements.push_back(Movement("Rotate", 360, -1));
        movements.push_back(Movement("Rotate", 360, 1));
        movements.push_back(Movement("GoTo", Eigen::Vector3d({distance_s, 0.0, -2*distance_s})));
        movements.push_back(Movement("GoTo", Eigen::Vector3d({distance_s, 0.0, -distance_s})));
        movements.push_back(Movement("GoTo", Eigen::Vector3d({2*distance_s, 0.0, -distance_s})));
        movements.push_back(Movement("GoTo", Eigen::Vector3d({distance_s, 0.0, -distance_s})));
        movements.push_back(Movement("GoTo", Eigen::Vector3d({2*distance_s, 0.0, -distance_s})));
        movements.push_back(Movement("Rotate", 180, 1));
        movements.push_back(Movement("GoTo", Eigen::Vector3d({0.0, 0.0, -distance_s})));
        movements.push_back(Movement("Rotate", 180, -1));
        movements.push_back(Movement("Finished"));

        this->blackboard_set<std::vector<Movement>>("movements", movements);
        this->blackboard_set<float>("takeoff_height", -distance_s);


        this->add_state("INITIAL TAKEOFF", std::make_unique<InitialTakeoffState>());
        this->add_state("GO TO POSITION", std::make_unique<GoToPositionState>());
        this->add_state("PRECISION ALIGN", std::make_unique<PrecisionAlignState>());
        this->add_state("LANDING", std::make_unique<LandingState>());
        this->add_state("ROTATE", std::make_unique<RotateState>());

        // Initial Takeoff transitions
        this->add_transitions("INITIAL TAKEOFF", {{"INITIAL TAKEOFF COMPLETED", "GO TO POSITION"},{"SEG FAULT", "ERROR"}});

        // Go To Position transitions
        this->add_transitions("GO TO POSITION", {{"ARRIVED AT POINT", "PRECISION ALIGN"},{"SEG FAULT", "ERROR"}});

        // Precision Align transitions
        this->add_transitions("PRECISION ALIGN", {
                                                    {"ROTATE NOW", "ROTATE"},
                                                    {"GOTO NOW", "GO TO POSITION"},
                                                    {"FINISHED CHALLENGE", "LANDING"},
                                                    {"SEG FAULT", "ERROR"}});

        // Rotate transitions
        this->add_transitions("ROTATE", {
                                            {"FINISHED ROTATION", "PRECISION ALIGN"},
                                            {"SEG FAULT", "ERROR"}});

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
