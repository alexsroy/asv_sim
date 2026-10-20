#pragma once
// Target ship: a kinematic vessel that follows a scripted course.
// Uses the sim's conventions: x = north [m], y = east [m],
// psi = heading [rad] (0 = north, clockwise positive), u = surge speed [m/s].

#include <cstddef>
#include <iosfwd>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "asv/frame_sink.hpp"
#include "asv/vessel.hpp"

namespace asv {

struct TargetParams {
    double max_speed = 10.0;      // [m/s]
    double max_accel = 0.5;       // [m/s^2]
    double max_decel = 0.8;       // [m/s^2]
    double max_turn_rate = 0.05;  // [rad/s]. Min turn radius = u / max_turn_rate.
};

// --- Script legs -----------------------------------------------------------

// Steer toward (x north, y east) at `speed`; done once inside `radius`.
// Keep radius above speed / max_turn_rate or the ship can orbit the waypoint.
struct GoTo {
    double x = 0.0;  // north [m]
    double y = 0.0;  // east [m]
    double speed = 0.0;
    double radius = 10.0;
};

// Hold a fixed heading at `speed` for `duration` seconds.
struct HoldHeading {
    double psi = 0.0;  // [rad]
    double speed = 0.0;
    double duration = 0.0;
};

// Slow to a stop and sit for `duration` seconds.
struct Hold {
    double duration = 0.0;
};

using Leg = std::variant<GoTo, HoldHeading, Hold>;

struct Script {
    std::vector<Leg> legs;
    bool loop = false;                  // restart from the first leg after the last
    std::optional<VesselState> start;   // from a "start" line; caller may use it
};

// Text format, one command per line, '#' starts a comment:
//   start <x_north> <y_east> <heading_deg> [speed]   (optional, at most once)
//   goto <x_north> <y_east> <speed> [radius]
//   heading <deg> <speed> <duration>
//   hold <duration>
//   loop
// Throws std::runtime_error with the line number on bad input.
Script parse_script(std::istream& in);
Script parse_script_file(const std::string& path);

// --- Ship ------------------------------------------------------------------

class TargetShip {
public:
    TargetShip(int vessel_id, VesselState initial, TargetParams params, Script script);

    // Advance by dt seconds (use the same fixed dt as the main sim).
    void step(double dt);

    // Snapshot for any FrameSink. psi_ref / u_cmd are the script's current
    // commands; delta and cross_track are 0 (not modelled for targets);
    // segment is the active script leg.
    Frame make_frame() const;

    int vessel_id() const { return vessel_id_; }
    const VesselState& state() const { return state_; }
    double time() const { return t_; }
    const TargetParams& params() const { return params_; }
    bool finished() const { return finished_; }
    std::size_t current_leg() const { return leg_index_; }

private:
    void advance_leg();
    void command_for_current_leg(double& psi_cmd, double& u_cmd);

    int vessel_id_;
    VesselState state_;
    TargetParams params_;
    Script script_;
    std::size_t leg_index_ = 0;
    double leg_elapsed_ = 0.0;
    double t_ = 0.0;
    double psi_ref_ = 0.0;
    double u_ref_ = 0.0;
    bool finished_ = false;
};

}  // namespace asv