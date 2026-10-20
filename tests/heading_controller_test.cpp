#include <gtest/gtest.h>

#include "asv/heading_controller.hpp"
#include "asv/math_constants.hpp"
#include "asv/vessel.hpp"

using namespace asv;

TEST(WrapPi, WrapsIntoRange) {
    EXPECT_NEAR(wrap_pi(1.5 * kPi), -0.5 * kPi, 1e-12);
    EXPECT_NEAR(wrap_pi(-1.5 * kPi), 0.5 * kPi, 1e-12);
    EXPECT_NEAR(wrap_pi(0.3), 0.3, 1e-12);
}

TEST(HeadingController, TurnsTheShortWay) {
    HeadingController c;
    // Target 350 deg, currently 10 deg: shortest turn is to port
    double delta = c.update(deg2rad(350.0), deg2rad(10.0), 0.0, 0.1);
    EXPECT_LT(delta, 0.0);
}

TEST(HeadingController, OutputIsClampedToRudderLimit) {
    PidParams p;
    p.delta_max = 0.5;
    HeadingController c(p);
    EXPECT_DOUBLE_EQ(c.update(3.0, 0.0, 0.0, 0.1), 0.5);
    EXPECT_DOUBLE_EQ(c.update(-3.0, 0.0, 0.0, 0.1), -0.5);
}

TEST(HeadingController, IntegralAccumulatesOnConstantError) {
    PidParams p;
    p.kp = 0.0;
    p.ki = 1.0;
    p.kd = 0.0;
    HeadingController c(p);
    double out = 0.0;
    for (int i = 0; i < 10; ++i) out = c.update(0.1, 0.0, 0.0, 0.1);
    EXPECT_NEAR(out, 0.1, 1e-12);  // ki * e * (10 * dt)
}

TEST(HeadingController, AntiWindupLetsItRecoverQuickly) {
    PidParams p;
    p.kp = 1.0;
    p.ki = 1.0;
    p.kd = 0.0;
    p.delta_max = 0.5;
    HeadingController c(p);

    // Long period of large error with the rudder pinned
    for (int i = 0; i < 1000; ++i) c.update(1.0, 0.0, 0.0, 0.1);

    // Error flips to a small negative value: output should go negative
    // right away, not stay stuck at +0.5 from a wound-up integral.
    double out = c.update(0.0, 0.1, 0.0, 0.1);
    EXPECT_LT(out, 0.0);
}

TEST(HeadingController, ClosedLoopReachesTargetHeading) {
    VesselParams vp;
    HeadingController ctrl;
    VesselState s;
    s.u = 2.0;

    const double psi_ref = deg2rad(90.0);
    const double dt = 0.1;
    for (int i = 0; i < 1000; ++i) {  // 100 seconds
        VesselInput in;
        in.u_cmd = 2.0;
        in.delta = ctrl.update(psi_ref, s.psi, s.r, dt);
        s = step_rk4(s, in, vp, dt);
    }
    EXPECT_NEAR(wrap_pi(psi_ref - s.psi), 0.0, 0.02);
}