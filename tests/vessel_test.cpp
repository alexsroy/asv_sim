#include <gtest/gtest.h>

#include <cmath>

#include "asv/vessel.hpp"

using namespace asv;

TEST(VesselDerivative, HeadingNorthMovesNorth) {
    VesselState s;
    s.u = 2.0;
    s.psi = 0.0;
    auto d = derivative(s, {}, {});
    EXPECT_NEAR(d.x, 2.0, 1e-12);
    EXPECT_NEAR(d.y, 0.0, 1e-12);
}

TEST(VesselDerivative, HeadingEastMovesEast) {
    VesselState s;
    s.u = 2.0;
    s.psi = M_PI / 2.0;
    auto d = derivative(s, {}, {});
    EXPECT_NEAR(d.x, 0.0, 1e-12);
    EXPECT_NEAR(d.y, 2.0, 1e-12);
}

TEST(VesselDerivative, YawRateSettlesAtKTimesDelta) {
    VesselParams p;
    VesselInput in;
    in.delta = 0.1;
    VesselState s;
    s.r = p.K * in.delta;  // steady-state yaw rate
    auto d = derivative(s, in, p);
    EXPECT_NEAR(d.r, 0.0, 1e-12);
}

TEST(VesselDerivative, SurgeAcceleratesTowardCommand) {
    VesselState s;
    s.u = 0.0;
    VesselInput in;
    in.u_cmd = 3.0;
    auto d = derivative(s, in, {});
    EXPECT_GT(d.u, 0.0);
}