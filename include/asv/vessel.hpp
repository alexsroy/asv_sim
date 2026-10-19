#pragma once

namespace asv {

struct VesselState {
    double x = 0.0;    // north [m]
    double y = 0.0;    // east [m]
    double psi = 0.0;  // heading [rad], 0 = north, clockwise positive
    double r = 0.0;    // yaw rate [rad/s]
    double u = 0.0;    // surge speed [m/s]
};

struct VesselParams {
    double K = 0.2;    // rudder gain [1/s]
    double T = 5.0;    // yaw time constant [s]
    double Tu = 4.0;   // surge time constant [s]
};

struct VesselInput {
    double delta = 0.0;  // rudder angle [rad]
    double u_cmd = 0.0;  // commanded speed [m/s]
};

// Time derivative of the state. Pure function, no hidden state.
VesselState derivative(const VesselState& s, const VesselInput& in,
                       const VesselParams& p);

}  // namespace asv