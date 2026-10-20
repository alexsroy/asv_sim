#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <vector>

#include "asv/heading_controller.hpp"
#include "asv/los_guidance.hpp"
#include "asv/vessel.hpp"
#include "asv/math_constants.hpp"

using namespace asv;

TEST(LosGuidance, HeadsAlongNorthboundSegment) {
    LosGuidance g({{0, 0}, {100, 0}}, 20.0, 5.0);
    EXPECT_NEAR(g.update(10, 0), 0.0, 1e-12);
}

TEST(LosGuidance, HeadsAlongEastboundSegment) {
    LosGuidance g({{0, 0}, {0, 100}}, 20.0, 5.0);
    EXPECT_NEAR(g.update(0, 10), kPi / 2.0, 1e-12);
}

TEST(LosGuidance, SteersBackTowardPath) {
    // Path runs north. East of it is starboard, so steer to port (negative).
    LosGuidance g({{0, 0}, {100, 0}}, 20.0, 5.0);
    EXPECT_NEAR(g.update(10, 5), -std::atan2(5.0, 20.0), 1e-12);
    EXPECT_NEAR(g.update(10, -5), std::atan2(5.0, 20.0), 1e-12);
}

TEST(LosGuidance, LargeCrossTrackApproachesPerpendicular) {
    LosGuidance g({{0, 0}, {100, 0}}, 20.0, 5.0);
    EXPECT_NEAR(g.update(50, 1e6), -kPi / 2.0, 1e-3);
}

TEST(LosGuidance, SwitchesToNextSegmentInsideAcceptanceRadius) {
    LosGuidance g({{0, 0}, {100, 0}, {100, 100}}, 20.0, 10.0);
    g.update(80, 0);  // 20 m from waypoint 1: still on segment 0
    EXPECT_EQ(g.segment(), 0u);
    g.update(95, 0);  // 5 m from waypoint 1: switch
    EXPECT_EQ(g.segment(), 1u);
    EXPECT_FALSE(g.finished());
}

TEST(LosGuidance, FinishesAtLastWaypoint) {
    LosGuidance g({{0, 0}, {100, 0}}, 20.0, 10.0);
    g.update(50, 0);
    EXPECT_FALSE(g.finished());
    g.update(95, 0);
    EXPECT_TRUE(g.finished());
}

TEST(LosGuidance, RejectsBadConfiguration) {
    std::vector<Waypoint> one{Waypoint{0.0, 0.0}};
    EXPECT_THROW(LosGuidance{one}, std::invalid_argument);
    EXPECT_THROW(LosGuidance(std::vector<Waypoint>{{0, 0}, {10, 0}}, 0.0),
                 std::invalid_argument);
}

TEST(LosGuidance, ClosedLoopFollowsRoute) {
    LosGuidance g({{0, 0}, {200, 0}, {200, 200}}, 20.0, 10.0);
    HeadingController ctrl;
    VesselParams vp;
    VesselState s;
    s.u = 2.0;

    const double dt = 0.1;
    for (int i = 0; i < 6000; ++i) {  // up to 600 s
        const double psi_d = g.update(s.x, s.y);
        if (g.finished()) break;
        VesselInput in;
        in.u_cmd = 2.0;
        in.delta = ctrl.update(psi_d, s.psi, s.r, dt);
        s = step_rk4(s, in, vp, dt);
    }

    EXPECT_TRUE(g.finished());
    EXPECT_LT(std::hypot(s.x - 200.0, s.y - 200.0), 10.0 + 1e-9);
}