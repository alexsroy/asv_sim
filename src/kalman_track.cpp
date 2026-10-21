#include "asv/kalman_track.hpp"

#include <algorithm>
#include <cmath>

namespace asv {

namespace {

Mat2 position_cov(const Contact& c) {
    Mat2 r;
    r(0, 0) = c.cov_xx;
    r(0, 1) = r(1, 0) = c.cov_xy;
    r(1, 1) = c.cov_yy;
    return r;
}

// Velocity measurement from AIS speed/course over ground, with its covariance
// propagated through v = sog * (cos cog, sin cog).
void velocity_measurement(const Contact& c, Vec2& v, Mat2& r) {
    const double cs = std::cos(c.cog);
    const double sn = std::sin(c.cog);
    v(0, 0) = c.sog * cs;  // north
    v(1, 0) = c.sog * sn;  // east

    Mat2 j;  // d(vx, vy) / d(sog, cog)
    j(0, 0) = cs;  j(0, 1) = -c.sog * sn;
    j(1, 0) = sn;  j(1, 1) = c.sog * cs;
    Mat2 noise;
    noise(0, 0) = c.sigma_sog * c.sigma_sog;
    noise(1, 1) = c.sigma_cog * c.sigma_cog;
    r = j * noise * transpose(j);
}

// Constant-velocity transition for a step of dt (dt may be negative).
Mat4 transition(double dt) {
    Mat4 f = Mat4::identity();
    f(0, 2) = dt;
    f(1, 3) = dt;
    return f;
}

// White-noise-acceleration process noise. It depends on |dt| so that stepping
// "backwards" (only used for gating) still gives a valid covariance.
Mat4 process_noise(double dt, double sigma_accel) {
    const double a = std::fabs(dt);
    const double q = sigma_accel * sigma_accel;
    Mat4 m;
    for (int axis = 0; axis < 2; ++axis) {
        m(axis, axis) = q * a * a * a / 3.0;
        m(axis, axis + 2) = m(axis + 2, axis) = q * a * a / 2.0;
        m(axis + 2, axis + 2) = q * a;
    }
    return m;
}

// Standard Kalman measurement update, Joseph form (keeps P symmetric and
// positive definite despite rounding).
template <int M>
void kf_update(KalmanState& s, const Mat<M, 4>& h, const Mat<M, 1>& z,
               const Mat<M, M>& r) {
    const Mat<M, 1> y = z - h * s.x;                  // innovation
    const Mat<M, M> S = h * s.P * transpose(h) + r;   // innovation covariance
    const Mat<4, M> k = s.P * transpose(h) * inverse(S);
    s.x = s.x + k * y;
    const Mat4 ikh = Mat4::identity() - k * h;
    s.P = ikh * s.P * transpose(ikh) + k * r * transpose(k);
}

}  // namespace

KalmanTrack::KalmanTrack(const Contact& first, KalmanParams params) : p_(params) {
    KalmanState s;
    s.t = first.t_meas;
    s.x(0, 0) = first.x;
    s.x(1, 0) = first.y;

    s.P(0, 0) = first.cov_xx;
    s.P(0, 1) = s.P(1, 0) = first.cov_xy;
    s.P(1, 1) = first.cov_yy;

    if (first.has_velocity) {
        Vec2 v;
        Mat2 rv;
        velocity_measurement(first, v, rv);
        s.x(2, 0) = v(0, 0);
        s.x(3, 0) = v(1, 0);
        s.P(2, 2) = rv(0, 0);
        s.P(2, 3) = s.P(3, 2) = rv(0, 1);
        s.P(3, 3) = rv(1, 1);
    } else {
        // Velocity unknown: start at rest with a wide prior.
        const double vv = p_.sigma_vel_init * p_.sigma_vel_init;
        s.P(2, 2) = vv;
        s.P(3, 3) = vv;
    }
    anchor_ = cur_ = s;
}

KalmanState KalmanTrack::propagate(const KalmanState& s, double t) const {
    const double dt = t - s.t;
    const Mat4 f = transition(dt);
    KalmanState out;
    out.x = f * s.x;
    out.P = f * s.P * transpose(f) + process_noise(dt, p_.sigma_accel);
    out.t = t;
    return out;
}

KalmanState KalmanTrack::apply(const KalmanState& s, const Contact& c) const {
    KalmanState out = propagate(s, c.t_meas);

    Vec2 zp;
    zp(0, 0) = c.x;
    zp(1, 0) = c.y;
    const Mat2 rp = position_cov(c);

    if (!c.has_velocity) {
        Mat<2, 4> h;
        h(0, 0) = 1.0;
        h(1, 1) = 1.0;
        kf_update<2>(out, h, zp, rp);
    } else {
        // Position and velocity together; the two measurements are independent.
        Vec2 zv;
        Mat2 rv;
        velocity_measurement(c, zv, rv);

        Vec4 z;
        z(0, 0) = zp(0, 0);  z(1, 0) = zp(1, 0);
        z(2, 0) = zv(0, 0);  z(3, 0) = zv(1, 0);

        Mat4 r;
        r(0, 0) = rp(0, 0);  r(0, 1) = rp(0, 1);
        r(1, 0) = rp(1, 0);  r(1, 1) = rp(1, 1);
        r(2, 2) = rv(0, 0);  r(2, 3) = rv(0, 1);
        r(3, 2) = rv(1, 0);  r(3, 3) = rv(1, 1);

        kf_update<4>(out, Mat4::identity(), z, r);
    }
    return out;
}

bool KalmanTrack::update(const Contact& c) {
    // Older than the anchor: its place in the sequence has already been folded
    // into the saved state and cannot be reopened.
    if (c.t_meas < anchor_.t) {
        ++rejected_old_;
        return false;
    }

    auto it = std::upper_bound(
        buf_.begin(), buf_.end(), c.t_meas,
        [](double t, const Contact& m) { return t < m.t_meas; });
    const bool newest = (it == buf_.end());
    buf_.insert(it, c);

    if (newest) {
        cur_ = apply(cur_, c);  // normal case: one incremental update
    } else {
        replay();               // late arrival: re-run the window in order
    }
    prune();
    ++updates_;
    return true;
}

void KalmanTrack::replay() {
    cur_ = anchor_;
    for (const Contact& m : buf_) cur_ = apply(cur_, m);
}

// Fold measurements that have fallen out of the window into the anchor so the
// buffer stays short.
void KalmanTrack::prune() {
    std::size_t n = 0;
    while (n < buf_.size() && buf_[n].t_meas < cur_.t - p_.history_window) {
        anchor_ = apply(anchor_, buf_[n]);
        ++n;
    }
    if (n > 0) buf_.erase(buf_.begin(), buf_.begin() + static_cast<std::ptrdiff_t>(n));
}

TrackEstimate KalmanTrack::estimate() const { return estimate_at(cur_.t); }

TrackEstimate KalmanTrack::estimate_at(double t) const {
    const KalmanState s = (t == cur_.t) ? cur_ : propagate(cur_, t);
    TrackEstimate e;
    e.t = s.t;
    e.kin = Kinematics{s.x(0, 0), s.x(1, 0), s.x(2, 0), s.x(3, 0)};
    e.P = s.P;
    return e;
}

double KalmanTrack::gate_distance2(const Contact& c) const {
    const KalmanState pred = propagate(cur_, c.t_meas);

    Vec2 y;
    y(0, 0) = c.x - pred.x(0, 0);
    y(1, 0) = c.y - pred.x(1, 0);

    Mat2 s = position_cov(c);
    s(0, 0) += pred.P(0, 0);
    s(0, 1) += pred.P(0, 1);
    s(1, 0) += pred.P(1, 0);
    s(1, 1) += pred.P(1, 1);

    const Mat<1, 1> d2 = transpose(y) * inverse(s) * y;
    return d2(0, 0);
}

}  // namespace asv