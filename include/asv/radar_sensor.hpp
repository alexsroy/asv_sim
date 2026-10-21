#pragma once

#include <cstdint>
#include <vector>

#include "asv/contact.hpp"
#include "asv/rng.hpp"
#include "asv/vessel.hpp"

namespace asv {

// Defaults are scaled to the sim's few-hundred-metre world, not a real marine
// radar. Scale ranges and sigmas up to model a real set.
struct RadarParams {
    double scan_period = 2.0;     // time between sweeps [s]
    double max_range = 500.0;     // [m]
    double sigma_range = 3.0;     // range noise [m]
    double sigma_bearing = 0.01;  // bearing noise [rad] (~0.6 deg)
    double p_detect = 0.9;        // chance a target in range is seen each sweep
    double clutter_mean = 0.3;    // mean false returns per sweep (Poisson)
    double latency = 0.3;         // measurement -> available [s]
};

// Rotating radar: once per scan_period it measures range and bearing to every
// target within range (subject to p_detect), adds Gaussian noise, adds random
// clutter, and delivers the results after `latency`. Radar contacts are
// anonymous (id = -1).
class RadarSensor {
public:
    explicit RadarSensor(RadarParams params = {}, std::uint32_t seed = 1);

    // Call once per sim step with the current time, own-ship state and truth.
    // Own ship is assumed to know its own position exactly.
    void update(double t, const VesselState& own,
                const std::vector<TruthTarget>& targets);

    // Contacts whose latency has elapsed by time t (removed from the queue).
    std::vector<Contact> poll(double t) { return queue_.poll(t); }

    const RadarParams& params() const { return p_; }

private:
    void scan(double t, const VesselState& own,
              const std::vector<TruthTarget>& targets);
    Contact make_contact(double t, const VesselState& own, double range,
                         double bearing, int truth_id) const;

    RadarParams p_;
    Rng rng_;
    ContactQueue queue_;
    double next_scan_ = 0.0;
};

}  // namespace asv