#include "asv/radar_sensor.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace asv {

RadarSensor::RadarSensor(RadarParams params, std::uint32_t seed)
    : p_(params), rng_(seed) {}

void RadarSensor::update(double t, const VesselState& own,
                         const std::vector<TruthTarget>& targets) {
    // Catch up if a step spans several scan periods; scans fire on schedule
    // (next_scan_ += period) so timing doesn't drift with the step size.
    while (t >= next_scan_) {
        scan(t, own, targets);
        next_scan_ += p_.scan_period;
    }
}

// Convert a (range, bearing) measurement to a world-frame contact. The
// covariance is the polar noise propagated through x = r cos b, y = r sin b:
// elongated across the line of sight, tight along it.
Contact RadarSensor::make_contact(double t, const VesselState& own,
                                  double range, double bearing,
                                  int truth_id) const {
    const double c = std::cos(bearing);
    const double s = std::sin(bearing);
    const double vr = p_.sigma_range * p_.sigma_range;
    const double vb = range * range * p_.sigma_bearing * p_.sigma_bearing;

    Contact ct;
    ct.kind = SensorKind::Radar;
    ct.t_meas = t;
    ct.t_avail = t + p_.latency;
    ct.id = -1;
    ct.x = own.x + range * c;  // north
    ct.y = own.y + range * s;  // east
    ct.cov_xx = vr * c * c + vb * s * s;
    ct.cov_yy = vr * s * s + vb * c * c;
    ct.cov_xy = (vr - vb) * s * c;
    ct.truth_id = truth_id;
    return ct;
}

void RadarSensor::scan(double t, const VesselState& own,
                       const std::vector<TruthTarget>& targets) {
    for (const auto& tg : targets) {
        const double dn = tg.state.x - own.x;
        const double de = tg.state.y - own.y;
        const double range = std::hypot(dn, de);
        if (range > p_.max_range) continue;
        if (!rng_.chance(p_.p_detect)) continue;  // missed detection

        const double bearing = std::atan2(de, dn);  // 0 = north, clockwise
        const double r_meas = range + rng_.normal(0.0, p_.sigma_range);
        const double b_meas = bearing + rng_.normal(0.0, p_.sigma_bearing);
        queue_.push(make_contact(t, own, std::max(r_meas, 0.0), b_meas, tg.id));
    }

    // False alarms: uniform over the radar disc, reported with no noise added
    // (they are not measurements of anything).
    const int n_clutter = rng_.poisson(p_.clutter_mean);
    for (int i = 0; i < n_clutter; ++i) {
        constexpr double kTwoPi = 6.283185307179586;
        const double r = p_.max_range * std::sqrt(rng_.uniform());
        const double b = kTwoPi * rng_.uniform();
        queue_.push(make_contact(t, own, r, b, /*truth_id=*/-1));
    }
}

}  // namespace asv