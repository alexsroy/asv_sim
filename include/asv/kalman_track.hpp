#pragma once
// Constant-velocity Kalman filter for one tracked vessel.
//
// State: [x, y, vx, vy] = north/east position [m] and north/east velocity
// [m/s]. Motion model: constant velocity with white-noise acceleration, so
// manoeuvres are absorbed as process noise (sigma_accel).
//
// Measurements are Contacts: position always (with the contact's covariance),
// plus velocity when the contact carries it (AIS).
//
// Out-of-order measurements are handled by replay. Radar arrives ~0.3 s late
// and AIS 1-2 s late, so an older AIS report routinely arrives AFTER a newer
// radar one. The track keeps a short window of recent measurements sorted by
// measurement time; a late arrival is inserted in order and the window is
// re-run from a saved anchor state, so the result is the same as if everything
// had arrived in order.

#include <vector>

#include "asv/contact.hpp"
#include "asv/cpa.hpp"
#include "asv/mat.hpp"

namespace asv {

struct KalmanParams {
    // Unmodelled acceleration [m/s^2, 1-sigma]. Larger tracks manoeuvres
    // faster but is noisier. Should be about the target's typical manoeuvre.
    double sigma_accel = 0.4;

    // Velocity uncertainty when a track starts from a position-only contact.
    double sigma_vel_init = 5.0;  // [m/s]

    // How far behind the newest measurement a late one is still accepted [s].
    // Must exceed the largest sensor latency difference.
    double history_window = 5.0;
};

struct KalmanState {
    Vec4 x;           // [x, y, vx, vy]
    Mat4 P;           // covariance
    double t = 0.0;   // time the state refers to [s]
};

struct TrackEstimate {
    double t = 0.0;
    Kinematics kin;   // position and velocity (north, east)
    Mat4 P;           // covariance of [x, y, vx, vy]
};

class KalmanTrack {
public:
    // Start a track from its first contact.
    explicit KalmanTrack(const Contact& first, KalmanParams params = {});

    // Fold in a contact. Returns false (and ignores it) if it is older than
    // the history window allows.
    bool update(const Contact& c);

    // Estimate at the time of the newest measurement.
    TrackEstimate estimate() const;

    // Estimate extrapolated to time t (constant-velocity prediction, with the
    // covariance grown accordingly). Works for t before the newest measurement
    // too, as an approximation, which is what gating needs for late contacts.
    TrackEstimate estimate_at(double t) const;

    // Squared Mahalanobis distance between the contact's position and the
    // track's prediction at the contact's measurement time, using the innovation
    // covariance. Compare against a chi-square threshold with 2 degrees of
    // freedom (9.21 = 99%, 5.99 = 95%). This is the gate for association.
    double gate_distance2(const Contact& c) const;

    double time() const { return cur_.t; }
    int updates() const { return updates_; }
    int rejected_old() const { return rejected_old_; }

private:
    KalmanState apply(const KalmanState& s, const Contact& c) const;
    KalmanState propagate(const KalmanState& s, double t) const;
    void replay();
    void prune();

    KalmanParams p_;
    KalmanState anchor_;            // state just before the first buffered measurement
    std::vector<Contact> buf_;      // measurements after the anchor, sorted by t_meas
    KalmanState cur_;               // anchor_ with every buffered measurement applied
    int updates_ = 0;
    int rejected_old_ = 0;
};

}  // namespace asv