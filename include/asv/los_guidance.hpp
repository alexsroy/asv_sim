#pragma once

#include <cstddef>
#include <vector>

namespace asv {

struct Waypoint {
    double x;  // north [m]
    double y;  // east [m]
};

class LosGuidance {
public:
    // Needs at least 2 waypoints, lookahead > 0, accept_radius > 0.
    explicit LosGuidance(std::vector<Waypoint> waypoints,
                         double lookahead = 20.0,
                         double accept_radius = 10.0);

    // Returns the desired heading [rad] for the current position and
    // advances to the next segment when a waypoint is reached.
    double update(double x, double y);

    bool finished() const { return finished_; }
    std::size_t segment() const { return seg_; }

    // Signed cross-track error [m] for the active segment (+ = starboard).
    double cross_track_error(double x, double y) const;

private:
    double segment_bearing() const;

    std::vector<Waypoint> wps_;
    double lookahead_;
    double accept_radius_;
    std::size_t seg_ = 0;
    bool finished_ = false;
};

}  // namespace asv