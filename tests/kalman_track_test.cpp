#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <vector>

#include "asv/ais_sensor.hpp"
#include "asv/contact.hpp"
#include "asv/kalman_track.hpp"
#include "asv/mat.hpp"
#include "asv/radar_sensor.hpp"
#include "asv/rng.hpp"
#include "asv/target_ship.hpp"

using namespace asv;

namespace {

constexpr double kPi = 3.14159265358979323846;  // M_PI is not portable to MSVC

// A position-only contact (radar-like) with isotropic variance.
Contact pos_contact(double t, double x, double y, double var) {
    Contact c;
    c.kind = SensorKind::Radar;
    c.t_meas = t;
    c.t_avail = t;
    c.x = x;
    c.y = y;
    c.cov_xx = var;
    c.cov_yy = var;
    return c;
}

// An AIS-like contact: position plus velocity.
Contact ais_contact(double t, double x, double y, double var, double sog, double cog) {
    Contact c = pos_contact(t, x, y, var);
    c.kind = SensorKind::Ais;
    c.id = 1;
    c.has_velocity = true;
    c.sog = sog;
    c.cog = cog;
    c.sigma_sog = 0.1;
    c.sigma_cog = 0.02;
    return c;
}

double pos_var_trace(const TrackEstimate& e) { return e.P(0, 0) + e.P(1, 1); }

}  // namespace

// --- Mat -----------------------------------------------------------------------

TEST(Mat, MultiplyTransposeAndInverse) {
    Mat<2, 3> a;
    a(0, 0) = 1; a(0, 1) = 2; a(0, 2) = 3;
    a(1, 0) = 4; a(1, 1) = 5; a(1, 2) = 6;
    auto at = transpose(a);
    auto p = a * at;  // 2x2
    EXPECT_DOUBLE_EQ(p(0, 0), 14.0);
    EXPECT_DOUBLE_EQ(p(0, 1), 32.0);
    EXPECT_DOUBLE_EQ(p(1, 1), 77.0);

    auto inv = inverse(p);
    auto id = p * inv;
    EXPECT_NEAR(id(0, 0), 1.0, 1e-12);
    EXPECT_NEAR(id(0, 1), 0.0, 1e-12);
    EXPECT_NEAR(id(1, 1), 1.0, 1e-12);
}

TEST(Mat, FourByFourInverseNeedsPivoting) {
    Mat4 m;  // zero on the first diagonal entry forces a row swap
    m(0, 1) = 2; m(0, 3) = 1;
    m(1, 0) = 3; m(1, 2) = 1;
    m(2, 1) = 1; m(2, 2) = 4;
    m(3, 0) = 1; m(3, 3) = 5;
    auto id = m * inverse(m);
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            EXPECT_NEAR(id(i, j), i == j ? 1.0 : 0.0, 1e-12);
}

TEST(Mat, SingularInverseThrows) {
    Mat2 m;
    m(0, 0) = 1; m(0, 1) = 2;
    m(1, 0) = 2; m(1, 1) = 4;
    EXPECT_THROW(inverse(m), std::runtime_error);
}

// --- Initialisation ------------------------------------------------------------

TEST(KalmanTrack, InitFromPositionOnlyStartsAtRestWithWideVelocityPrior) {
    KalmanParams p;
    KalmanTrack tr(pos_contact(5.0, 100.0, 50.0, 9.0), p);
    auto e = tr.estimate();
    EXPECT_DOUBLE_EQ(e.t, 5.0);
    EXPECT_DOUBLE_EQ(e.kin.x, 100.0);
    EXPECT_DOUBLE_EQ(e.kin.y, 50.0);
    EXPECT_DOUBLE_EQ(e.kin.vx, 0.0);
    EXPECT_NEAR(e.P(0, 0), 9.0, 1e-12);
    EXPECT_NEAR(e.P(2, 2), p.sigma_vel_init * p.sigma_vel_init, 1e-12);
}

TEST(KalmanTrack, InitFromAisUsesItsVelocity) {
    KalmanTrack tr(ais_contact(0.0, 0.0, 0.0, 4.0, 4.0, kPi / 2.0));  // 4 m/s east
    auto e = tr.estimate();
    EXPECT_NEAR(e.kin.vx, 0.0, 1e-9);
    EXPECT_NEAR(e.kin.vy, 4.0, 1e-9);
    EXPECT_LT(e.P(2, 2), 1.0);  // velocity is known well, not the wide prior
}

// --- Filtering -----------------------------------------------------------------

TEST(KalmanTrack, ConvergesOnConstantVelocityTarget) {
    Rng rng(1);
    const double sigma = 3.0;
    const double vx = -3.0, vy = 1.0;
    KalmanTrack tr(pos_contact(0.0, rng.normal(0, sigma), rng.normal(0, sigma), sigma * sigma));
    for (int k = 1; k <= 60; ++k) {
        const double t = 2.0 * k;
        tr.update(pos_contact(t, vx * t + rng.normal(0, sigma), vy * t + rng.normal(0, sigma),
                              sigma * sigma));
    }
    auto e = tr.estimate();
    const double t = e.t;
    const double pos_err = std::hypot(e.kin.x - vx * t, e.kin.y - vy * t);
    const double vel_err = std::hypot(e.kin.vx - vx, e.kin.vy - vy);
    EXPECT_LT(pos_err, 6.0);
    EXPECT_LT(vel_err, 1.5);
    // The error should sit inside the filter's own claimed uncertainty (4 sigma
    // on the 2-D norm). With sigma_accel = 0.4 the filter is deliberately a bit
    // unsure of velocity (~0.8 m/s) so that it can follow manoeuvres.
    EXPECT_LT(pos_err, 4.0 * std::sqrt(e.P(0, 0) + e.P(1, 1)));
    EXPECT_LT(vel_err, 4.0 * std::sqrt(e.P(2, 2) + e.P(3, 3)));
    EXPECT_LT(std::sqrt(e.P(2, 2)), 1.0);  // and not absurdly unsure
}

TEST(KalmanTrack, FilteredPositionBeatsRawMeasurements) {
    Rng rng(2);
    const double sigma = 3.0;
    const double vx = 2.0, vy = -1.0;
    KalmanTrack tr(pos_contact(0.0, 0.0, 0.0, sigma * sigma));
    double raw_sq = 0.0, filt_sq = 0.0;
    int n = 0;
    for (int k = 1; k <= 300; ++k) {
        const double t = 1.0 * k;
        const double mx = vx * t + rng.normal(0, sigma);
        const double my = vy * t + rng.normal(0, sigma);
        tr.update(pos_contact(t, mx, my, sigma * sigma));
        if (k > 30) {  // after the filter has settled
            auto e = tr.estimate();
            raw_sq += (mx - vx * t) * (mx - vx * t) + (my - vy * t) * (my - vy * t);
            filt_sq += (e.kin.x - vx * t) * (e.kin.x - vx * t) +
                       (e.kin.y - vy * t) * (e.kin.y - vy * t);
            ++n;
        }
    }
    EXPECT_LT(std::sqrt(filt_sq / n), 0.8 * std::sqrt(raw_sq / n));
}

TEST(KalmanTrack, AisVelocitySpeedsUpVelocityConvergence) {
    // Same noisy position measurements; one track also gets AIS velocity.
    Rng rng(3);
    const double var = 9.0;
    const double vx = 3.0, vy = 0.0;
    Contact first = pos_contact(0.0, 0.0, 0.0, var);
    KalmanTrack pos_only(first);
    Contact first_ais = ais_contact(0.0, 0.0, 0.0, var, 3.0, 0.0);
    KalmanTrack with_vel(first_ais);
    for (int k = 1; k <= 3; ++k) {
        const double t = 3.0 * k;
        const double mx = vx * t + rng.normal(0, 3.0), my = vy * t + rng.normal(0, 3.0);
        pos_only.update(pos_contact(t, mx, my, var));
        with_vel.update(ais_contact(t, mx, my, var, 3.0 + rng.normal(0, 0.1), rng.normal(0, 0.02)));
    }
    auto a = pos_only.estimate(), b = with_vel.estimate();
    EXPECT_LT(std::hypot(b.kin.vx - vx, b.kin.vy - vy),
              std::hypot(a.kin.vx - vx, a.kin.vy - vy));
    EXPECT_LT(b.P(2, 2), a.P(2, 2));
}

TEST(KalmanTrack, CovarianceStaysSymmetricAndPositive) {
    Rng rng(4);
    KalmanTrack tr(pos_contact(0.0, 0.0, 0.0, 9.0));
    for (int k = 1; k <= 500; ++k)
        tr.update(pos_contact(k * 0.5, rng.normal(0, 3), rng.normal(0, 3), 9.0));
    auto e = tr.estimate();
    for (int i = 0; i < 4; ++i) {
        EXPECT_GT(e.P(i, i), 0.0);
        for (int j = 0; j < 4; ++j) EXPECT_NEAR(e.P(i, j), e.P(j, i), 1e-9);
    }
    EXPECT_GT(e.P(0, 0) * e.P(1, 1) - e.P(0, 1) * e.P(0, 1), 0.0);
}

// --- Dropouts ------------------------------------------------------------------

TEST(KalmanTrack, UncertaintyGrowsDuringDropoutAndShrinksOnRecovery) {
    Rng rng(5);
    const double var = 9.0;
    KalmanTrack tr(pos_contact(0.0, 0.0, 0.0, var));
    for (int k = 1; k <= 30; ++k)
        tr.update(pos_contact(2.0 * k, 2.0 * k * 3.0 + rng.normal(0, 3), rng.normal(0, 3), var));
    const double before = pos_var_trace(tr.estimate());
    const double coasting = pos_var_trace(tr.estimate_at(tr.time() + 20.0));
    EXPECT_GT(coasting, before);  // no data: uncertainty grows

    const double t_back = tr.time() + 20.0;
    tr.update(pos_contact(t_back, 3.0 * t_back + rng.normal(0, 3), rng.normal(0, 3), var));
    EXPECT_LT(pos_var_trace(tr.estimate()), coasting);  // data again: it tightens
}

TEST(KalmanTrack, CoastsThroughDropoutWithHonestUncertainty) {
    const double var = 9.0;
    const double vx = 3.0, vy = -2.0;
    Rng rng(6);
    KalmanTrack tr(pos_contact(0.0, 0.0, 0.0, var));
    for (int k = 1; k <= 40; ++k) {
        const double t = 1.0 * k;
        tr.update(pos_contact(t, vx * t + rng.normal(0, 3), vy * t + rng.normal(0, 3), var));
    }
    // 15 s with no measurements.
    const double t = tr.time() + 15.0;
    auto e = tr.estimate_at(t);
    const double err = std::hypot(e.kin.x - vx * t, e.kin.y - vy * t);
    EXPECT_LT(err, 4.0 * std::sqrt(pos_var_trace(e)));  // error within the stated uncertainty
    EXPECT_LT(err, 25.0);                               // and it is actually still close
}

// --- Out-of-order measurements ---------------------------------------------------

TEST(KalmanTrack, OutOfOrderArrivalsGiveTheSameResultAsInOrder) {
    Rng rng(7);
    const double var = 9.0;
    std::vector<Contact> ms;
    for (int k = 0; k <= 20; ++k) {
        const double t = 1.0 * k;
        ms.push_back(pos_contact(t, 3.0 * t + rng.normal(0, 3), -1.0 * t + rng.normal(0, 3), var));
    }
    // Make some of them AIS (with velocity) so replay covers both update types.
    for (int k : {4, 9, 15}) {
        ms[k] = ais_contact(ms[k].t_meas, ms[k].x, ms[k].y, var, std::hypot(3.0, 1.0),
                            std::atan2(-1.0, 3.0));
    }

    KalmanTrack in_order(ms[0]);
    for (std::size_t k = 1; k < ms.size(); ++k) in_order.update(ms[k]);

    // Scramble the arrival order: swap neighbouring pairs and delay a few by
    // up to 3 s, so late measurements keep arriving after newer ones.
    std::vector<Contact> shuffled(ms.begin() + 1, ms.end());
    for (std::size_t i = 0; i + 1 < shuffled.size(); i += 2) std::swap(shuffled[i], shuffled[i + 1]);
    for (std::size_t i = 0; i + 3 < shuffled.size(); i += 5) {
        Contact late = shuffled[i];
        shuffled.erase(shuffled.begin() + static_cast<std::ptrdiff_t>(i));
        shuffled.insert(shuffled.begin() + static_cast<std::ptrdiff_t>(i + 3), late);
    }

    KalmanTrack scrambled(ms[0]);
    for (const Contact& c : shuffled) EXPECT_TRUE(scrambled.update(c));

    auto a = in_order.estimate(), b = scrambled.estimate();
    EXPECT_DOUBLE_EQ(a.t, b.t);
    EXPECT_NEAR(a.kin.x, b.kin.x, 1e-9);
    EXPECT_NEAR(a.kin.y, b.kin.y, 1e-9);
    EXPECT_NEAR(a.kin.vx, b.kin.vx, 1e-9);
    EXPECT_NEAR(a.kin.vy, b.kin.vy, 1e-9);
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) EXPECT_NEAR(a.P(i, j), b.P(i, j), 1e-9);
}

TEST(KalmanTrack, RejectsMeasurementsOlderThanTheWindow) {
    KalmanParams p;
    p.history_window = 5.0;
    KalmanTrack tr(pos_contact(0.0, 0.0, 0.0, 9.0), p);
    for (int k = 1; k <= 100; ++k) tr.update(pos_contact(1.0 * k, 0.0, 0.0, 9.0));
    EXPECT_FALSE(tr.update(pos_contact(50.0, 0.0, 0.0, 9.0)));
    EXPECT_EQ(tr.rejected_old(), 1);
    EXPECT_TRUE(tr.update(pos_contact(98.0, 0.0, 0.0, 9.0)));  // inside the window
}

// --- Gating --------------------------------------------------------------------

TEST(KalmanTrack, GateAcceptsNearbyAndRejectsFarContacts) {
    Rng rng(8);
    const double var = 9.0;
    KalmanTrack tr(pos_contact(0.0, 0.0, 0.0, var));
    for (int k = 1; k <= 30; ++k)
        tr.update(pos_contact(1.0 * k, 3.0 * k + rng.normal(0, 3), rng.normal(0, 3), var));
    const double t = tr.time() + 1.0;
    const Contact near = pos_contact(t, 3.0 * t, 0.0, var);
    const Contact far = pos_contact(t, 3.0 * t + 200.0, 150.0, var);
    EXPECT_LT(tr.gate_distance2(near), 5.99);   // inside the 95% gate
    EXPECT_GT(tr.gate_distance2(far), 9.21);    // outside the 99% gate
}

// --- Statistical consistency --------------------------------------------------

// If the world really follows the filter's model (white-noise acceleration
// with the same sigma, Gaussian measurement noise), the normalised estimation
// error squared (NEES) averages to the state dimension, 4. Much higher means
// the covariance is over-confident; much lower, too cautious.
TEST(KalmanTrack, NeesIsConsistentWithTheModel) {
    KalmanParams kp;
    kp.sigma_accel = 0.4;
    kp.sigma_vel_init = 5.0;
    const double sigma = 3.0;
    const double dt = 1.0;
    const int steps = 60, runs = 400;
    Rng rng(99);

    // Cholesky factor of one axis's [pos, vel] process noise for a step dt.
    const double q = kp.sigma_accel * kp.sigma_accel;
    const double qa = q * dt * dt * dt / 3.0, qb = q * dt * dt / 2.0, qd = q * dt;
    const double l11 = std::sqrt(qa), l21 = qb / l11, l22 = std::sqrt(qd - l21 * l21);

    double nees_sum = 0.0;
    for (int run = 0; run < runs; ++run) {
        // True state at t = 0: position at the origin, velocity drawn from the
        // filter's prior. The first measurement is the true position plus noise.
        double pos[2] = {0.0, 0.0};
        double vel[2] = {rng.normal(0, kp.sigma_vel_init), rng.normal(0, kp.sigma_vel_init)};
        KalmanTrack tr(pos_contact(0.0, rng.normal(0, sigma), rng.normal(0, sigma), sigma * sigma), kp);

        for (int k = 1; k <= steps; ++k) {
            for (int a = 0; a < 2; ++a) {
                const double z1 = rng.normal(), z2 = rng.normal();
                pos[a] += vel[a] * dt + l11 * z1;
                vel[a] += l21 * z1 + l22 * z2;
            }
            tr.update(pos_contact(k * dt, pos[0] + rng.normal(0, sigma),
                                  pos[1] + rng.normal(0, sigma), sigma * sigma));
        }

        auto e = tr.estimate();
        Vec4 err;
        err(0, 0) = e.kin.x - pos[0];
        err(1, 0) = e.kin.y - pos[1];
        err(2, 0) = e.kin.vx - vel[0];
        err(3, 0) = e.kin.vy - vel[1];
        nees_sum += (transpose(err) * inverse(e.P) * err)(0, 0);
    }
    const double mean_nees = nees_sum / runs;
    EXPECT_GT(mean_nees, 3.4);
    EXPECT_LT(mean_nees, 4.7);
}

// --- End to end: real sensors, clutter, dropouts, a manoeuvring target --------

// Own ship sits still at the origin. The target shuttles back and forth across
// 440 m of water making 180-degree turns at each end. Radar adds clutter and
// misses detections; AIS drops out in 10 s bursts. A single track, gated on the
// radar contacts (AIS trusted by id), must stay on the target throughout.
TEST(KalmanTrackEndToEnd, StaysOnAManoeuvringTargetThroughNoiseClutterAndDropouts) {
    Script script;
    script.loop = true;
    script.legs.emplace_back(GoTo{100.0, -120.0, 3.0, 15.0});
    script.legs.emplace_back(GoTo{100.0, 320.0, 3.0, 15.0});
    VesselState start;
    start.x = 100.0;
    start.y = 320.0;
    start.psi = 1.5 * kPi;  // heading west
    start.u = 3.0;
    TargetParams tp;
    tp.max_speed = 6.0;
    tp.max_turn_rate = 0.1;
    TargetShip target(1, start, tp, script);

    RadarSensor radar(RadarParams{}, 1);
    AisSensor ais(AisParams{}, 2);
    const VesselState own;  // at the origin, stationary

    std::optional<KalmanTrack> track;
    const double dt = 0.1, t_end = 600.0, gate = 9.21;
    double max_pos_err = 0.0, vel_sq = 0.0;
    int vel_n = 0;
    int radar_used = 0, radar_gated_out = 0;

    for (int i = 0; i * dt <= t_end; ++i) {
        const double t = i * dt;
        const std::vector<TruthTarget> truth{{1, target.state()}};
        radar.update(t, own, truth);
        ais.update(t, own, truth);

        std::vector<Contact> batch = radar.poll(t);
        for (const Contact& c : ais.poll(t)) batch.push_back(c);
        for (const Contact& c : batch) {
            if (!track) {
                if (c.id != -1) track.emplace(c);  // start on an identified contact
            } else if (c.id != -1) {
                track->update(c);
            } else if (track->gate_distance2(c) < gate) {
                track->update(c);
                ++radar_used;
            } else {
                ++radar_gated_out;
            }
        }

        if (track && t > 30.0) {
            const auto e = track->estimate_at(t);
            const auto& s = target.state();
            const double perr = std::hypot(e.kin.x - s.x, e.kin.y - s.y);
            max_pos_err = std::max(max_pos_err, perr);
            const double evx = e.kin.vx - s.u * std::cos(s.psi);
            const double evy = e.kin.vy - s.u * std::sin(s.psi);
            vel_sq += evx * evx + evy * evy;
            ++vel_n;
        }
        target.step(dt);
    }

    ASSERT_TRUE(track.has_value());
    EXPECT_LT(max_pos_err, 20.0);                        // never loses the target
    EXPECT_LT(std::sqrt(vel_sq / vel_n), 1.0);           // velocity estimate is sane
    EXPECT_GT(radar_used, 100);                          // radar really contributed
    EXPECT_GT(radar_gated_out, 0);                       // and the gate rejected some clutter
}