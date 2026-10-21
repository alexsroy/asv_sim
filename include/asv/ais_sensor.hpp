#pragma once

#include <cstdint>
#include <map>
#include <vector>

#include "asv/contact.hpp"
#include "asv/rng.hpp"
#include "asv/vessel.hpp"

namespace asv {

struct AisParams {
    double report_period = 3.0;    // each vessel transmits this often [s]
    double max_range = 2000.0;     // reception range [m]
    double sigma_pos = 2.0;        // GPS position noise, per axis [m]
    double sigma_sog = 0.1;        // speed noise [m/s]
    double sigma_cog = 0.02;       // course noise [rad]
    double p_drop = 0.05;          // chance a report starts a dropout
    double drop_duration = 10.0;   // a dropout silences that vessel this long [s]
    double latency = 1.0;          // base delay, report -> available [s]
    double latency_jitter = 1.0;   // extra uniform random delay in [0, jitter] [s]
};

// AIS: every vessel transmits its own identity, position, speed and course on a
// fixed period. Reports can drop out in bursts (a dropout silences that vessel
// for drop_duration) and arrive late with random jitter, so they can reach the
// tracker out of order. AIS contacts carry the vessel id.
class AisSensor {
public:
    explicit AisSensor(AisParams params = {}, std::uint32_t seed = 2);

    void update(double t, const VesselState& own,
                const std::vector<TruthTarget>& targets);

    std::vector<Contact> poll(double t) { return queue_.poll(t); }

    const AisParams& params() const { return p_; }

private:
    struct Link {
        double next_report = 0.0;
        double silent_until = -1.0;
    };

    AisParams p_;
    Rng rng_;
    ContactQueue queue_;
    std::map<int, Link> links_;
};

}  // namespace asv