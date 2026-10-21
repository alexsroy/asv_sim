#include <gtest/gtest.h>

#include <cmath>

#include "asv/encounter.hpp"
#include "asv/heading_controller.hpp"
#include "asv/heading_planner.hpp"
#include "asv/math_constants.hpp"
#include "asv/vessel.hpp"

using namespace asv;

namespace {

VesselState own_at(double x, double y, double psi, double u) {
    VesselState s;
    s.x = x;
    s.y = y;
    s.psi = psi;
    s.u = u;
    return s;
}

Kinematics tgt_at(double x, double y, double psi, double u) {
    Kinematics k;
    k.x = x;
    k.y = y;
    k.vx = u * std::cos(psi);
    k.vy = u * std::sin(psi);
    return k;
}

}  // namespace

TEST(HeadingPlanner, NoneReturnsDesiredWithoutSearch) {
    const auto own = own_at(0, 0, 0.0, 2.0);
    const auto tgt = tgt_at(0, 200, 0.0, 2.0);  // far abeam, opening
    const double psi_d = 0.0;
    const auto plan = plan_heading(own, psi_d, tgt, Encounter::None);
    EXPECT_FALSE(plan.active);
    EXPECT_DOUBLE_EQ(plan.psi_cmd, psi_d);
}

TEST(HeadingPlanner, HeadOnPrefersStarboardTurn) {
    // Reciprocal closing; LOS wants to keep going north.
    const auto own = own_at(0, 0, 0.0, 2.0);
    const auto tgt = tgt_at(80, 0, kPi, 2.0);
    ASSERT_EQ(classify_encounter(own, tgt).type, Encounter::HeadOn);

    const auto plan = plan_heading(own, /*psi_desired=*/0.0, tgt, Encounter::HeadOn);
    EXPECT_TRUE(plan.active);
    // Starboard = positive (clockwise) heading change.
    EXPECT_GT(wrap_pi(plan.psi_cmd - own.psi), 0.0);
    // Planned CPA should be at least as safe as holding course.
    const CpaResult hold = compute_cpa(kinematics(own), tgt);
    EXPECT_GE(plan.best.cpa.dcpa, hold.dcpa - 1e-9);
}

TEST(HeadingPlanner, CrossingGiveWayTurnsToClear) {
    const auto own = own_at(0, 0, 0.0, 2.0);
    const auto tgt = tgt_at(40, 60, -kPi / 2.0, 2.0);
    ASSERT_EQ(classify_encounter(own, tgt).type, Encounter::CrossingGiveWay);

    const auto plan =
        plan_heading(own, /*psi_desired=*/0.0, tgt, Encounter::CrossingGiveWay);
    EXPECT_TRUE(plan.active);
    EXPECT_GT(wrap_pi(plan.psi_cmd - own.psi), 0.0);  // starboard bias
}

TEST(HeadingPlanner, StandOnStaysNearDesiredUnlessNecessary) {
    // Port crossing stand-on: should hold nearly north unless forced.
    const auto own = own_at(0, 0, 0.0, 2.0);
    const auto tgt = tgt_at(40, -60, kPi / 2.0, 2.0);
    ASSERT_EQ(classify_encounter(own, tgt).type, Encounter::CrossingStandOn);

    const double psi_d = 0.0;
    const auto plan = plan_heading(own, psi_d, tgt, Encounter::CrossingStandOn);
    EXPECT_TRUE(plan.active);
    EXPECT_LT(std::fabs(wrap_pi(plan.psi_cmd - psi_d)), deg2rad(25.0));
}

TEST(HeadingPlanner, ImprovesDangerousCpa) {
    // Near head-on with small offset so holding course still risks collision.
    const auto own = own_at(0, 0, 0.0, 2.0);
    const auto tgt = tgt_at(70, 5, kPi, 2.0);
    const Encounter enc = classify_encounter(own, tgt).type;
    ASSERT_NE(enc, Encounter::None);

    const CpaResult hold = compute_cpa(kinematics(own), tgt);
    ASSERT_TRUE(collision_risk(hold, 30.0, 60.0));

    const auto plan = plan_heading(own, 0.0, tgt, enc);
    EXPECT_TRUE(plan.active);
    EXPECT_GT(plan.best.cpa.dcpa, hold.dcpa);
}

TEST(HeadingPlanner, DeviationCostFavoursDesiredWhenSafeCandidatesExist) {
    // Mild give-way geometry: planner should not swing a full 90° if a smaller
    // starboard turn already clears.
    const auto own = own_at(0, 0, 0.0, 2.0);
    const auto tgt = tgt_at(50, 40, -kPi / 2.0, 2.0);
    const auto plan =
        plan_heading(own, 0.0, tgt, Encounter::CrossingGiveWay);
    EXPECT_TRUE(plan.active);
    EXPECT_LT(std::fabs(wrap_pi(plan.psi_cmd - 0.0)), deg2rad(75.0));
}
