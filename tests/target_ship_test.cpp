#include <gtest/gtest.h>

#include <cmath>
#include <sstream>

#include "asv/target_ship.hpp"

using namespace asv;

namespace {

constexpr double kDt = 0.1;
constexpr double kPi = 3.14159265358979323846;  // M_PI is not portable to MSVC

TargetParams fast_params() {
    TargetParams p;
    p.max_speed = 5.0;
    p.max_accel = 2.0;
    p.max_decel = 2.0;
    p.max_turn_rate = 0.5;
    return p;
}

void run(TargetShip& s, double seconds) {
    for (double t = 0.0; t < seconds; t += kDt) s.step(kDt);
}

}  // namespace

TEST(TargetShip, EmptyScriptIsFinishedAndStationary) {
    TargetShip s(1, VesselState{}, fast_params(), Script{});
    EXPECT_TRUE(s.finished());
    run(s, 5.0);
    EXPECT_DOUBLE_EQ(s.state().x, 0.0);
    EXPECT_DOUBLE_EQ(s.state().y, 0.0);
}

TEST(TargetShip, HeadingNorthIncreasesX) {
    Script sc;
    sc.legs.emplace_back(HoldHeading{0.0, 4.0, 20.0});
    TargetShip s(1, VesselState{}, fast_params(), sc);
    run(s, 10.0);
    EXPECT_GT(s.state().x, 20.0);
    EXPECT_NEAR(s.state().y, 0.0, 1e-6);
}

TEST(TargetShip, HeadingEastIncreasesY) {
    Script sc;
    sc.legs.emplace_back(HoldHeading{kPi / 2.0, 4.0, 20.0});
    VesselState init;
    init.psi = kPi / 2.0;
    TargetShip s(1, init, fast_params(), sc);
    run(s, 10.0);
    EXPECT_GT(s.state().y, 20.0);
    EXPECT_NEAR(s.state().x, 0.0, 1e-6);
}

TEST(TargetShip, ReachesWaypointsInOrderAndFinishes) {
    Script sc;
    sc.legs.emplace_back(GoTo{100.0, 0.0, 4.0, 10.0});    // 100 m north
    sc.legs.emplace_back(GoTo{100.0, 100.0, 4.0, 10.0});  // then 100 m east
    TargetShip s(1, VesselState{}, fast_params(), sc);
    run(s, 120.0);
    EXPECT_TRUE(s.finished());
    EXPECT_LT(std::hypot(s.state().x - 100.0, s.state().y - 100.0), 15.0);
}

TEST(TargetShip, RespectsSpeedAndTurnRateLimits) {
    Script sc;
    sc.legs.emplace_back(GoTo{0.0, -500.0, 99.0, 10.0});  // west, absurd speed
    TargetParams p = fast_params();
    TargetShip s(1, VesselState{}, p, sc);
    double prev_psi = s.state().psi;
    for (int i = 0; i < 300; ++i) {
        s.step(kDt);
        EXPECT_LE(s.state().u, p.max_speed + 1e-9);
        EXPECT_LE(std::fabs(s.state().r), p.max_turn_rate + 1e-9);
        double d = std::fabs(std::remainder(s.state().psi - prev_psi, 2.0 * kPi));
        EXPECT_LE(d, p.max_turn_rate * kDt + 1e-9);
        prev_psi = s.state().psi;
    }
}

TEST(TargetShip, HoldStopsTheShip) {
    Script sc;
    sc.legs.emplace_back(HoldHeading{0.0, 4.0, 10.0});
    sc.legs.emplace_back(Hold{30.0});
    TargetShip s(1, VesselState{}, fast_params(), sc);
    run(s, 30.0);
    EXPECT_NEAR(s.state().u, 0.0, 1e-9);
}

TEST(TargetShip, LoopRestartsScript) {
    Script sc;
    sc.loop = true;
    sc.legs.emplace_back(HoldHeading{0.0, 2.0, 5.0});
    sc.legs.emplace_back(HoldHeading{kPi, 2.0, 5.0});
    TargetShip s(1, VesselState{}, fast_params(), sc);
    run(s, 60.0);
    EXPECT_FALSE(s.finished());
}

TEST(TargetShip, ZeroDurationLoopDoesNotHang) {
    Script sc;
    sc.loop = true;
    sc.legs.emplace_back(Hold{0.0});
    TargetShip s(1, VesselState{}, fast_params(), sc);
    s.step(kDt);  // must return
    SUCCEED();
}

TEST(TargetShip, MakeFrameReflectsStateAndCommands) {
    Script sc;
    sc.legs.emplace_back(HoldHeading{0.0, 4.0, 100.0});
    TargetShip s(7, VesselState{}, fast_params(), sc);
    run(s, 5.0);
    Frame f = s.make_frame();
    EXPECT_EQ(f.vessel_id, 7);
    EXPECT_NEAR(f.t, s.time(), 1e-12);
    EXPECT_DOUBLE_EQ(f.state.x, s.state().x);
    EXPECT_DOUBLE_EQ(f.u_cmd, 4.0);
    EXPECT_EQ(f.segment, 0u);
}

TEST(TargetShipScript, ParsesAllCommands) {
    std::istringstream in(
        "# patrol\n"
        "goto 100 200 5 15\n"
        "heading 90 3 20   # east\n"
        "hold 10\n"
        "loop\n");
    Script sc = parse_script(in);
    ASSERT_EQ(sc.legs.size(), 3u);
    EXPECT_TRUE(sc.loop);
    EXPECT_TRUE(std::holds_alternative<GoTo>(sc.legs[0]));
    EXPECT_NEAR(std::get<HoldHeading>(sc.legs[1]).psi, kPi / 2.0, 1e-12);
}

TEST(TargetShipScript, BadInputReportsLine) {
    std::istringstream in("goto 1 2 3\nbogus 1\n");
    try {
        parse_script(in);
        FAIL() << "expected throw";
    } catch (const std::runtime_error& e) {
        EXPECT_NE(std::string(e.what()).find("line 2"), std::string::npos);
    }
}