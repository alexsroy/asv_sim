#pragma once
// COLREGS encounter classifier from own ship's point of view.
//
// Uses relative bearing and heading difference between own ship and a target's
// kinematics (typically a Kalman track estimate). Classifies only when CPA
// indicates collision risk; otherwise returns Encounter::None.
//
// Decision order matches COLREGS priority: overtaking (Rule 13), then head-on
// (Rule 14), then crossing give-way / stand-on (Rule 15).

#include "asv/cpa.hpp"
#include "asv/math_constants.hpp"
#include "asv/vessel.hpp"

namespace asv {

enum class Encounter {
    None,             // no collision risk / opening / too slow to classify
    HeadOn,           // Rule 14
    CrossingGiveWay,  // Rule 15 — other on starboard
    CrossingStandOn,  // Rule 15 — other on port
    Overtaking,       // Rule 13 — own is overtaking (give-way)
    BeingOvertaken,   // Rule 13 — other approaching from abaft own beam
};

struct EncounterParams {
    double dcpa_limit = 30.0;   // [m] — must match collision_risk used in the viewer
    double tcpa_limit = 60.0;   // [s]

    // |relative bearing| above this → stern sector (22.5° abaft the beam).
    double stern_sector = deg2rad(112.5);

    // Head-on: target roughly ahead and courses nearly reciprocal.
    double head_on_bearing = deg2rad(15.0);
    double head_on_heading = deg2rad(22.5);

    // Below this speed the target course is unreliable → None.
    double min_speed = 0.1;  // [m/s]
};

struct EncounterResult {
    Encounter type = Encounter::None;
    double relative_bearing = 0.0;  // β [rad], starboard positive
    double heading_diff = 0.0;      // wrap_pi(ψ_t - ψ_o) [rad]
    CpaResult cpa;
};

// Human-readable label for HUD / logs (stable ASCII, no spaces).
const char* encounter_name(Encounter e);

EncounterResult classify_encounter(const VesselState& own, const Kinematics& target,
                                   EncounterParams params = {});

}  // namespace asv
