#include "asv/heading_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "asv/heading_controller.hpp"

namespace asv {

namespace {

Kinematics own_on_heading(const VesselState& own, double psi) {
    Kinematics k;
    k.x = own.x;
    k.y = own.y;
    k.vx = own.u * std::cos(psi);
    k.vy = own.u * std::sin(psi);
    return k;
}

// Lower is safer. Hard penalty inside the CPA envelope; soft preference for
// larger margins outside it.
double cpa_term(const CpaResult& cpa, double dcpa_limit, double tcpa_limit) {
    if (collision_risk(cpa, dcpa_limit, tcpa_limit)) {
        const double gap = std::max(dcpa_limit - cpa.dcpa, 0.0);
        return 1.0 + (gap / dcpa_limit) * (gap / dcpa_limit);
    }
    // Mild preference for larger CPA once already safe.
    return dcpa_limit / (dcpa_limit + std::max(cpa.dcpa, 0.0));
}

// Positive turn = starboard (clockwise heading frame).
double rule_term(Encounter encounter, double turn, double psi_cand,
                 double psi_desired, double port_penalty) {
    const double abs_turn = std::fabs(turn);
    const double hold = std::fabs(wrap_pi(psi_cand - psi_desired));

    switch (encounter) {
        case Encounter::HeadOn:
        case Encounter::CrossingGiveWay:
            // Prefer starboard; penalise port alterations.
            if (turn < 0.0) return port_penalty * abs_turn;
            return 0.15 * abs_turn;  // small cost so CPA can still widen the turn
        case Encounter::Overtaking:
            // Give-way: mild starboard bias, otherwise free to clear.
            if (turn < 0.0) return 0.5 * port_penalty * abs_turn;
            return 0.1 * abs_turn;
        case Encounter::CrossingStandOn:
        case Encounter::BeingOvertaken:
            // Stand-on: stay on the guidance heading unless CPA forces escape.
            return hold;
        case Encounter::None:
            return 0.0;
    }
    return 0.0;
}

}  // namespace

HeadingPlan plan_heading(const VesselState& own, double psi_desired,
                         const Kinematics& target, Encounter encounter,
                         HeadingPlannerParams params) {
    HeadingPlan plan;
    plan.encounter = encounter;
    plan.psi_cmd = psi_desired;

    if (encounter == Encounter::None || params.num_candidates < 1) {
        plan.best.psi = psi_desired;
        plan.best.cpa = compute_cpa(own_on_heading(own, psi_desired), target);
        return plan;
    }

    plan.active = true;
    const int n = std::max(params.num_candidates, 1);
    double best_cost = std::numeric_limits<double>::infinity();

    for (int i = 0; i < n; ++i) {
        const double frac = (n == 1) ? 0.5 : static_cast<double>(i) / (n - 1);
        const double offset = -params.half_span + frac * (2.0 * params.half_span);
        const double psi = wrap_pi(psi_desired + offset);

        const CpaResult cpa = compute_cpa(own_on_heading(own, psi), target);
        const double turn = wrap_pi(psi - own.psi);

        CandidateScore s;
        s.psi = psi;
        s.cpa = cpa;
        s.cpa_cost = cpa_term(cpa, params.dcpa_limit, params.tcpa_limit);
        s.dev_cost = wrap_pi(psi - psi_desired);
        s.dev_cost *= s.dev_cost;
        s.rule_cost =
            rule_term(encounter, turn, psi, psi_desired, params.port_penalty);
        s.cost = params.w_cpa * s.cpa_cost + params.w_dev * s.dev_cost +
                 params.w_rule * s.rule_cost;

        if (s.cost < best_cost) {
            best_cost = s.cost;
            plan.best = s;
            plan.psi_cmd = psi;
        }
    }
    return plan;
}

}  // namespace asv
