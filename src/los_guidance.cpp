#include "asv/los_guidance.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

#include "asv/heading_controller.hpp"  // wrap_pi

namespace asv {

LosGuidance::LosGuidance(std::vector<Waypoint> waypoints, double lookahead,
                         double accept_radius)
    : wps_(std::move(waypoints)),
      lookahead_(lookahead),
      accept_radius_(accept_radius) {
    if (wps_.size() < 2) {
        throw std::invalid_argument("LosGuidance needs at least 2 waypoints");
    }
    if (lookahead_ <= 0.0 || accept_radius_ <= 0.0) {
        throw std::invalid_argument(
            "lookahead and accept_radius must be positive");
    }
}

double LosGuidance::segment_bearing() const {
    const Waypoint& a = wps_[seg_];
    const Waypoint& b = wps_[seg_ + 1];
    return std::atan2(b.y - a.y, b.x - a.x);  // east over north
}

double LosGuidance::cross_track_error(double x, double y) const {
    const Waypoint& a = wps_[seg_];
    const double alpha = segment_bearing();
    return -(x - a.x) * std::sin(alpha) + (y - a.y) * std::cos(alpha);
}

double LosGuidance::update(double x, double y) {
    // Advance segments while inside the acceptance circle of the next waypoint.
    while (!finished_) {
        const Waypoint& next = wps_[seg_ + 1];
        const double dist = std::hypot(next.x - x, next.y - y);
        if (dist > accept_radius_) break;
        if (seg_ + 2 < wps_.size()) {
            ++seg_;
        } else {
            finished_ = true;
        }
    }

    const double e = cross_track_error(x, y);
    return wrap_pi(segment_bearing() - std::atan2(e, lookahead_));
}

}  // namespace asv