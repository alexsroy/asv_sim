#include "asv/ais_sensor.hpp"

#include <algorithm>
#include <cmath>

namespace asv {

AisSensor::AisSensor(AisParams params, std::uint32_t seed)
    : p_(params), rng_(seed) {}

void AisSensor::update(double t, const VesselState& own,
                       const std::vector<TruthTarget>& targets) {
    for (const auto& tg : targets) {
        // First sight of a vessel: its first report is due now.
        Link& link = links_.try_emplace(tg.id, Link{t, -1.0}).first->second;

        while (t >= link.next_report) {
            const double t_report = link.next_report;
            link.next_report += p_.report_period;

            // Out of reception range, or in a dropout: nothing is received.
            const double range = std::hypot(tg.state.x - own.x, tg.state.y - own.y);
            if (range > p_.max_range) continue;
            if (t_report < link.silent_until) continue;

            // A new dropout starts here: this report is lost, and so are the
            // following ones until the burst ends.
            if (rng_.chance(p_.p_drop)) {
                link.silent_until = t_report + p_.drop_duration;
                continue;
            }

            const double var = p_.sigma_pos * p_.sigma_pos;
            Contact c;
            c.kind = SensorKind::Ais;
            c.t_meas = t;
            c.t_avail = t + p_.latency + rng_.uniform(0.0, p_.latency_jitter);
            c.id = tg.id;
            c.x = tg.state.x + rng_.normal(0.0, p_.sigma_pos);
            c.y = tg.state.y + rng_.normal(0.0, p_.sigma_pos);
            c.cov_xx = var;
            c.cov_xy = 0.0;
            c.cov_yy = var;
            c.has_velocity = true;
            c.sog = std::max(tg.state.u + rng_.normal(0.0, p_.sigma_sog), 0.0);
            c.cog = tg.state.psi + rng_.normal(0.0, p_.sigma_cog);
            c.sigma_sog = p_.sigma_sog;
            c.sigma_cog = p_.sigma_cog;
            c.truth_id = tg.id;
            queue_.push(c);
        }
    }
}

}  // namespace asv