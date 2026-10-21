#include "asv/encounter.hpp"

#include <cmath>

#include "asv/heading_controller.hpp"
#include "asv/math_constants.hpp"

namespace asv {

namespace {

bool in_stern_sector(double relative_bearing, double stern_sector) {
    return std::fabs(relative_bearing) > stern_sector;
}

}  // namespace

const char* encounter_name(Encounter e) {
    switch (e) {
        case Encounter::None:
            return "None";
        case Encounter::HeadOn:
            return "HeadOn";
        case Encounter::CrossingGiveWay:
            return "CrossingGiveWay";
        case Encounter::CrossingStandOn:
            return "CrossingStandOn";
        case Encounter::Overtaking:
            return "Overtaking";
        case Encounter::BeingOvertaken:
            return "BeingOvertaken";
    }
    return "None";
}

EncounterResult classify_encounter(const VesselState& own, const Kinematics& target,
                                   EncounterParams params) {
    EncounterResult out;
    out.cpa = compute_cpa(kinematics(own), target);

    const double px = target.x - own.x;
    const double py = target.y - own.y;
    const double abs_bearing = std::atan2(py, px);
    out.relative_bearing = wrap_pi(abs_bearing - own.psi);

    const double target_speed = std::hypot(target.vx, target.vy);
    if (target_speed < params.min_speed) {
        return out;  // type remains None
    }

    const double psi_t = std::atan2(target.vy, target.vx);
    out.heading_diff = wrap_pi(psi_t - own.psi);

    if (!collision_risk(out.cpa, params.dcpa_limit, params.tcpa_limit)) {
        return out;
    }

    const double own_speed = own.u;
    // Aspect: relative bearing of own ship as seen from the target.
    const double aspect = wrap_pi(std::atan2(-py, -px) - psi_t);

    // Rule 13 — overtaking takes priority over head-on / crossing.
    if (in_stern_sector(aspect, params.stern_sector) && own_speed > target_speed) {
        out.type = Encounter::Overtaking;
        return out;
    }
    if (in_stern_sector(out.relative_bearing, params.stern_sector) &&
        target_speed > own_speed) {
        out.type = Encounter::BeingOvertaken;
        return out;
    }

    // Rule 14 — nearly reciprocal courses, target roughly ahead.
    const double reciprocal_err = std::fabs(wrap_pi(out.heading_diff - kPi));
    if (std::fabs(out.relative_bearing) < params.head_on_bearing &&
        reciprocal_err < params.head_on_heading) {
        out.type = Encounter::HeadOn;
        return out;
    }

    // Rule 15 — crossing: starboard → give-way, port → stand-on.
    out.type = (out.relative_bearing > 0.0) ? Encounter::CrossingGiveWay
                                            : Encounter::CrossingStandOn;
    return out;
}

}  // namespace asv
