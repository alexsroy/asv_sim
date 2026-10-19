#include "asv/heading_controller.hpp"

#include <algorithm>
#include <cmath>

namespace asv {

double wrap_pi(double angle) {
    constexpr double kTwoPi = 6.283185307179586;
    return std::remainder(angle, kTwoPi);
}

HeadingController::HeadingController(PidParams params) : p_(params) {}

double HeadingController::update(double psi_ref, double psi, double r,
                                 double dt) {
    const double e = wrap_pi(psi_ref - psi);

    const double integral_candidate = integral_ + e * dt;
    const double unsat = p_.kp * e + p_.ki * integral_candidate - p_.kd * r;
    const double sat = std::clamp(unsat, -p_.delta_max, p_.delta_max);

    // Anti-windup: only keep the new integral if we're not saturated,
    // or if the error is working to pull us back out of saturation.
    const bool saturated = (sat != unsat);
    const bool pushing_deeper = ((unsat > 0.0) == (e > 0.0));
    if (!saturated || !pushing_deeper) {
        integral_ = integral_candidate;
    }

    return sat;
}

void HeadingController::reset() { integral_ = 0.0; }

}  // namespace asv