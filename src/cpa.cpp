#include "asv/cpa.hpp"

#include <algorithm>
#include <cmath>

namespace asv {

namespace {
// Below this squared relative speed (~3e-5 m/s) the vessels are treated as
// having no relative motion; dividing by it would blow up.
constexpr double kMinRelSpeedSq = 1.0e-9;
}  // namespace

Kinematics kinematics(const VesselState& s) {
    Kinematics k;
    k.x = s.x;
    k.y = s.y;
    k.vx = s.u * std::cos(s.psi);
    k.vy = s.u * std::sin(s.psi);
    return k;
}

// Work in the target's frame relative to own ship: p = relative position,
// v = relative velocity. Separation at time t is |p + v t|, which is smallest
// where d/dt |p + v t|^2 = 0, i.e. t = -(p . v) / (v . v).
CpaResult compute_cpa(const Kinematics& own, const Kinematics& target) {
    const double px = target.x - own.x;
    const double py = target.y - own.y;
    const double vx = target.vx - own.vx;
    const double vy = target.vy - own.vy;

    CpaResult r;
    r.range = std::hypot(px, py);

    const double v2 = vx * vx + vy * vy;
    if (v2 < kMinRelSpeedSq) {
        // Same velocity: separation never changes.
        r.tcpa = 0.0;
        r.dcpa = r.range;
        r.closing = false;
        return r;
    }

    r.tcpa = -(px * vx + py * vy) / v2;
    const double t = std::max(r.tcpa, 0.0);  // only the future matters for dcpa
    r.dcpa = std::hypot(px + vx * t, py + vy * t);
    r.closing = r.tcpa > 0.0;
    return r;
}

CpaResult compute_cpa(const VesselState& own, const VesselState& target) {
    return compute_cpa(kinematics(own), kinematics(target));
}

bool collision_risk(const CpaResult& cpa, double dcpa_limit, double tcpa_limit) {
    return cpa.dcpa < dcpa_limit && cpa.tcpa < tcpa_limit;
}

}  // namespace asv