#pragma once
// Discrete COLREGS heading planner.
//
// Samples candidate headings around the LOS desired heading and scores each:
//   cost = w_cpa * cpa_term + w_dev * deviation^2 + w_rule * rule_term
//
// CPA term: penalises predicted closest approach under an instantaneous heading
//           change at constant surge speed.
// Deviation: squared heading error from the guidance (LOS) command.
// Rule bias: COLREGS-preferred turn direction / stand-on behaviour from the
//            encounter classifier (starboard for head-on / give-way; hold
//            course for stand-on / being overtaken).

#include "asv/cpa.hpp"
#include "asv/encounter.hpp"
#include "asv/math_constants.hpp"
#include "asv/vessel.hpp"

namespace asv {

struct HeadingPlannerParams {
    // Search ± half_span around psi_desired (inclusive endpoints).
    double half_span = deg2rad(90.0);
    int num_candidates = 37;  // every 5° over ±90°

    double w_cpa = 10.0;
    double w_dev = 1.0;
    double w_rule = 4.0;

    // Same safety envelope as collision_risk / encounter classifier.
    double dcpa_limit = 30.0;  // [m]
    double tcpa_limit = 60.0;  // [s]

    // Extra cost weight on port turns when the rule prefers starboard.
    double port_penalty = 2.0;
};

struct CandidateScore {
    double psi = 0.0;
    double cost = 0.0;
    double cpa_cost = 0.0;
    double dev_cost = 0.0;
    double rule_cost = 0.0;
    CpaResult cpa;
};

struct HeadingPlan {
    double psi_cmd = 0.0;       // selected heading [rad]
    CandidateScore best;
    Encounter encounter = Encounter::None;
    bool active = false;  // false → returned psi_desired without search
};

// Score candidates for one target. If the encounter is None (no risk), returns
// psi_desired immediately with active = false.
HeadingPlan plan_heading(const VesselState& own, double psi_desired,
                         const Kinematics& target, Encounter encounter,
                         HeadingPlannerParams params = {});

}  // namespace asv
