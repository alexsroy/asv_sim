#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>

#include "asv/cpa.hpp"
#include "asv/rng.hpp"

using namespace asv;

namespace {

constexpr double kPi = 3.14159265358979323846;  // M_PI is not portable to MSVC

Kinematics K(double x, double y, double vx, double vy) { return Kinematics{x, y, vx, vy}; }

}  // namespace

// --- Hand-worked cases ---------------------------------------------------------

TEST(Cpa, HeadOnCollisionCourse) {
    // Own heads north at 5, target is 1000 m north heading south at 5.
    auto r = compute_cpa(K(0, 0, 5, 0), K(1000, 0, -5, 0));
    EXPECT_NEAR(r.range, 1000.0, 1e-9);
    EXPECT_NEAR(r.tcpa, 100.0, 1e-9);  // closing at 10 m/s over 1000 m
    EXPECT_NEAR(r.dcpa, 0.0, 1e-9);
    EXPECT_TRUE(r.closing);
}

TEST(Cpa, CrossingPassesAtOffset) {
    // Own stationary. Target 1000 m north and 100 m east, moving south at 10.
    auto r = compute_cpa(K(0, 0, 0, 0), K(1000, 100, -10, 0));
    EXPECT_NEAR(r.tcpa, 100.0, 1e-9);
    EXPECT_NEAR(r.dcpa, 100.0, 1e-9);  // passes 100 m to the east
    EXPECT_TRUE(r.closing);
    EXPECT_NEAR(r.range, std::hypot(1000.0, 100.0), 1e-9);
}

TEST(Cpa, OpeningHasNegativeTcpaAndDcpaEqualsRange) {
    // Target 1000 m north, moving further north at 5; own stationary.
    auto r = compute_cpa(K(0, 0, 0, 0), K(1000, 0, 5, 0));
    EXPECT_NEAR(r.tcpa, -200.0, 1e-9);  // closest approach was 200 s ago
    EXPECT_NEAR(r.dcpa, 1000.0, 1e-9);  // from now on, closest is now
    EXPECT_FALSE(r.closing);
}

TEST(Cpa, NoRelativeMotion) {
    // Same velocity: separation is constant.
    auto r = compute_cpa(K(0, 0, 3, 1), K(500, 200, 3, 1));
    EXPECT_DOUBLE_EQ(r.tcpa, 0.0);
    EXPECT_NEAR(r.dcpa, r.range, 1e-12);
    EXPECT_FALSE(r.closing);
}

TEST(Cpa, BothStationary) {
    auto r = compute_cpa(K(0, 0, 0, 0), K(30, 40, 0, 0));
    EXPECT_NEAR(r.range, 50.0, 1e-12);
    EXPECT_DOUBLE_EQ(r.tcpa, 0.0);
    EXPECT_NEAR(r.dcpa, 50.0, 1e-12);
    EXPECT_FALSE(r.closing);
}

TEST(Cpa, ExactlyAtClosestPointNow) {
    // Target abeam (100 m east) moving north: separation is momentarily minimal.
    auto r = compute_cpa(K(0, 0, 0, 0), K(0, 100, 5, 0));
    EXPECT_NEAR(r.tcpa, 0.0, 1e-9);
    EXPECT_NEAR(r.dcpa, 100.0, 1e-9);
    EXPECT_FALSE(r.closing);
}

TEST(Cpa, OvertakingFromBehind) {
    // Target is 200 m behind (south) on the same track, 2 m/s faster.
    auto r = compute_cpa(K(0, 0, 4, 0), K(-200, 0, 6, 0));
    EXPECT_NEAR(r.tcpa, 100.0, 1e-9);
    EXPECT_NEAR(r.dcpa, 0.0, 1e-9);
    EXPECT_TRUE(r.closing);
}

// --- Properties ----------------------------------------------------------------

TEST(Cpa, IsSymmetricInOwnAndTarget) {
    const Kinematics a = K(10, -20, 3, 4), b = K(500, 300, -2, 1);
    auto ab = compute_cpa(a, b);
    auto ba = compute_cpa(b, a);
    EXPECT_NEAR(ab.tcpa, ba.tcpa, 1e-9);
    EXPECT_NEAR(ab.dcpa, ba.dcpa, 1e-9);
    EXPECT_NEAR(ab.range, ba.range, 1e-9);
}

TEST(Cpa, UnchangedByCommonVelocity) {
    // Adding the same drift (current/wind) to both vessels changes nothing.
    const Kinematics a = K(0, 0, 2, 1), b = K(600, 150, -3, 0.5);
    auto base = compute_cpa(a, b);
    Kinematics a2 = a, b2 = b;
    a2.vx += 7.0; a2.vy -= 3.0;
    b2.vx += 7.0; b2.vy -= 3.0;
    auto drift = compute_cpa(a2, b2);
    EXPECT_NEAR(base.tcpa, drift.tcpa, 1e-9);
    EXPECT_NEAR(base.dcpa, drift.dcpa, 1e-9);
}

TEST(Cpa, DcpaIsPositionAtTcpa) {
    // Moving both vessels forward by tcpa should give separation == dcpa.
    const Kinematics a = K(0, 0, 3, 2), b = K(800, -200, -4, 1);
    auto r = compute_cpa(a, b);
    ASSERT_GT(r.tcpa, 0.0);
    const double dx = (b.x + b.vx * r.tcpa) - (a.x + a.vx * r.tcpa);
    const double dy = (b.y + b.vy * r.tcpa) - (a.y + a.vy * r.tcpa);
    EXPECT_NEAR(std::hypot(dx, dy), r.dcpa, 1e-9);
}

TEST(Cpa, MatchesBruteForceOnRandomScenarios) {
    Rng rng(123);
    const double horizon = 600.0;
    const double step = 0.01;
    int checked = 0;
    for (int n = 0; n < 200; ++n) {
        const Kinematics a = K(rng.uniform(-500, 500), rng.uniform(-500, 500),
                               rng.uniform(-8, 8), rng.uniform(-8, 8));
        const Kinematics b = K(rng.uniform(-1000, 1000), rng.uniform(-1000, 1000),
                               rng.uniform(-8, 8), rng.uniform(-8, 8));
        const auto r = compute_cpa(a, b);

        // Brute force: smallest separation sampled over t in [0, horizon].
        double best_d = std::numeric_limits<double>::max();
        double best_t = 0.0;
        for (double t = 0.0; t <= horizon; t += step) {
            const double d = std::hypot((b.x + b.vx * t) - (a.x + a.vx * t),
                                        (b.y + b.vy * t) - (a.y + a.vy * t));
            if (d < best_d) { best_d = d; best_t = t; }
        }

        if (r.tcpa > horizon - 1.0) continue;  // minimum is beyond the window
        EXPECT_NEAR(r.dcpa, best_d, 0.05);
        EXPECT_NEAR(std::max(r.tcpa, 0.0), best_t, 1.0);
        ++checked;
    }
    EXPECT_GT(checked, 100);  // the comparison actually ran on most scenarios
}

// --- VesselState overload (heading convention) ---------------------------------

TEST(Cpa, VesselStateHeadingNorthAndSouthIsHeadOn) {
    VesselState own, tgt;
    own.u = 5.0;  own.psi = 0.0;  // north
    tgt.x = 1000.0;  tgt.u = 5.0;  tgt.psi = kPi;  // south
    auto r = compute_cpa(own, tgt);
    EXPECT_NEAR(r.tcpa, 100.0, 1e-9);
    EXPECT_NEAR(r.dcpa, 0.0, 1e-9);
}

TEST(Cpa, VesselStateHeadingEastMovesTowardPositiveY) {
    VesselState own, tgt;
    own.u = 3.0;  own.psi = kPi / 2.0;  // east
    tgt.y = 300.0;                      // stationary, 300 m east
    auto r = compute_cpa(own, tgt);
    EXPECT_NEAR(r.tcpa, 100.0, 1e-9);
    EXPECT_NEAR(r.dcpa, 0.0, 1e-9);
}

TEST(Cpa, KinematicsFromVesselState) {
    VesselState s;
    s.x = 1.0; s.y = 2.0; s.u = 10.0; s.psi = kPi / 2.0;
    auto k = kinematics(s);
    EXPECT_NEAR(k.vx, 0.0, 1e-12);   // no northward motion heading east
    EXPECT_NEAR(k.vy, 10.0, 1e-12);
    EXPECT_DOUBLE_EQ(k.x, 1.0);
    EXPECT_DOUBLE_EQ(k.y, 2.0);
}

// --- collision_risk -------------------------------------------------------------

TEST(CollisionRisk, FlagsCloseSoonPasses) {
    auto r = compute_cpa(K(0, 0, 0, 0), K(1000, 20, -10, 0));  // dcpa 20 m, tcpa 100 s
    EXPECT_TRUE(collision_risk(r, 50.0, 200.0));
}

TEST(CollisionRisk, IgnoresWideMisses) {
    auto r = compute_cpa(K(0, 0, 0, 0), K(1000, 300, -10, 0));  // dcpa 300 m
    EXPECT_FALSE(collision_risk(r, 50.0, 200.0));
}

TEST(CollisionRisk, IgnoresPassesTooFarInTheFuture) {
    auto r = compute_cpa(K(0, 0, 0, 0), K(1000, 20, -10, 0));  // tcpa 100 s
    EXPECT_FALSE(collision_risk(r, 50.0, 60.0));
}

TEST(CollisionRisk, DivergingFarAwayIsSafe) {
    auto r = compute_cpa(K(0, 0, 0, 0), K(1000, 0, 5, 0));
    EXPECT_FALSE(collision_risk(r, 50.0, 200.0));
}

TEST(CollisionRisk, InsideLimitCountsEvenIfOpening) {
    auto r = compute_cpa(K(0, 0, 0, 0), K(20, 0, 1, 0));  // 20 m away, moving apart
    EXPECT_TRUE(collision_risk(r, 50.0, 200.0));
}