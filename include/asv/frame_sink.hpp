#pragma once

#include <cstddef>

#include "asv/vessel.hpp"

namespace asv {

// Snapshot of one simulation step, handed to every sink.
struct Frame {
    double t = 0.0;               // sim time [s]
    int vessel_id = 0;
    VesselState state;            // x, y, psi, r, u
    double psi_ref = 0.0;         // desired heading from guidance [rad]
    double delta = 0.0;           // rudder command [rad]
    double u_cmd = 0.0;           // speed command [m/s]
    double cross_track = 0.0;     // cross-track error [m], + = starboard
    std::size_t segment = 0;      // active route segment
};

class FrameSink {
public:
    virtual ~FrameSink() = default;
    virtual void on_frame(const Frame& frame) = 0;
    virtual void flush() {}
};

}  // namespace asv