#pragma once
// Closest point of approach (CPA) and time to CPA (TCPA) between two vessels
// moving in straight lines at constant velocity.
//
// Frame: x = north [m], y = east [m]; velocities are (north, east) [m/s].

#include "asv/vessel.hpp"

namespace asv {

// Position and velocity of a vessel. This is what a tracker will output, and
// what a VesselState reduces to (see kinematics()).
struct Kinematics {
    double x = 0.0;   // north [m]
    double y = 0.0;   // east [m]
    double vx = 0.0;  // north velocity [m/s]
    double vy = 0.0;  // east velocity [m/s]
};

// Velocity from heading and surge speed: v = u * (cos psi, sin psi).
// Ignores sideslip, so it is "speed through water along the heading".
Kinematics kinematics(const VesselState& s);

struct CpaResult {
    double range = 0.0;  // current separation [m]

    // Time until closest approach [s]. Negative means the closest approach was
    // in the past, i.e. the vessels are already opening. Zero when there is no
    // relative motion (same velocity) or the vessels are exactly at CPA now.
    double tcpa = 0.0;

    // Separation at the closest approach from now on [m]. When tcpa <= 0 the
    // closest point from now on is now, so dcpa == range.
    double dcpa = 0.0;

    // True if the separation is shrinking right now (tcpa > 0).
    bool closing = false;
};

// CPA of `target` relative to `own`. Symmetric: swapping the arguments gives
// the same tcpa and dcpa.
CpaResult compute_cpa(const Kinematics& own, const Kinematics& target);
CpaResult compute_cpa(const VesselState& own, const VesselState& target);

// True when the target will pass inside dcpa_limit [m] within tcpa_limit [s].
// A vessel already inside the limit and opening still counts as a risk, since
// it is currently too close. A TCPA in the past never exceeds the time limit.
bool collision_risk(const CpaResult& cpa, double dcpa_limit, double tcpa_limit);

}  // namespace asv