#pragma once
// What the perception side sees: noisy, delayed measurements of other vessels.

#include <map>
#include <ostream>
#include <vector>

#include "asv/vessel.hpp"

namespace asv {

enum class SensorKind { Radar, Ais };

// One measurement of one (possibly nonexistent) vessel.
// Positions use the sim's frame: x = north [m], y = east [m].
struct Contact {
    SensorKind kind = SensorKind::Radar;

    double t_meas = 0.0;   // when the measurement was taken [s]
    double t_avail = 0.0;  // when it reaches the tracker (t_meas + latency) [s]

    // AIS carries the vessel's identity; radar is anonymous (-1).
    int id = -1;

    // Measured position and its covariance (symmetric 2x2: xx, xy, yy) [m^2].
    double x = 0.0;
    double y = 0.0;
    double cov_xx = 0.0;
    double cov_xy = 0.0;
    double cov_yy = 0.0;

    // AIS also reports velocity (speed and course over ground).
    bool has_velocity = false;
    double sog = 0.0;        // [m/s]
    double cog = 0.0;        // [rad], 0 = north, clockwise
    double sigma_sog = 0.0;  // [m/s]
    double sigma_cog = 0.0;  // [rad]

    // Ground truth, for scoring only: which vessel really produced this
    // measurement (-1 = clutter / false alarm). The tracker must not read it.
    int truth_id = -1;
};

// A real vessel as the sensors see it (ground truth, noise-free).
struct TruthTarget {
    int id = 0;
    VesselState state;
};

// Holds contacts until their latency has elapsed, released in t_avail order.
class ContactQueue {
public:
    void push(const Contact& c) { q_.emplace(c.t_avail, c); }

    // All contacts with t_avail <= t, oldest first. They are removed.
    std::vector<Contact> poll(double t) {
        std::vector<Contact> out;
        auto end = q_.upper_bound(t);
        for (auto it = q_.begin(); it != end; ++it) out.push_back(it->second);
        q_.erase(q_.begin(), end);
        return out;
    }

    std::size_t pending() const { return q_.size(); }

private:
    std::multimap<double, Contact> q_;
};

// Logs contacts as they are delivered. Columns:
//   t (delivery time), kind, id, truth_id, t_meas, x, y, cov_xx, cov_xy, cov_yy,
//   has_velocity, sog, cog
class ContactCsvWriter {
public:
    explicit ContactCsvWriter(std::ostream& os) : os_(os) {
        os_ << "t,kind,id,truth_id,t_meas,x,y,cov_xx,cov_xy,cov_yy,"
               "has_velocity,sog,cog\n";
    }

    void write(double t_now, const Contact& c) {
        os_ << t_now << ',' << (c.kind == SensorKind::Radar ? "radar" : "ais")
            << ',' << c.id << ',' << c.truth_id << ',' << c.t_meas << ',' << c.x
            << ',' << c.y << ',' << c.cov_xx << ',' << c.cov_xy << ','
            << c.cov_yy << ',' << (c.has_velocity ? 1 : 0) << ',' << c.sog << ','
            << c.cog << '\n';
    }

private:
    std::ostream& os_;
};

}  // namespace asv