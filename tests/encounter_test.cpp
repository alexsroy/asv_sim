#include <gtest/gtest.h>

#include <cmath>

#include "asv/encounter.hpp"
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

TEST(Encounter, HeadOnReciprocalCourses) {
    // Own northbound at origin; target southbound ahead on the same meridian.
    const auto own = own_at(0, 0, 0.0, 2.0);
    const auto tgt = tgt_at(80, 0, kPi, 2.0);
    const auto r = classify_encounter(own, tgt);
    EXPECT_EQ(r.type, Encounter::HeadOn);
    EXPECT_STREQ(encounter_name(r.type), "HeadOn");
    EXPECT_TRUE(r.cpa.closing);
}

TEST(Encounter, CrossingGiveWayTargetOnStarboard) {
    // Own northbound; target westbound from the east → crosses from starboard.
    const auto own = own_at(0, 0, 0.0, 2.0);
    const auto tgt = tgt_at(40, 60, -kPi / 2.0, 2.0);
    const auto r = classify_encounter(own, tgt);
    EXPECT_EQ(r.type, Encounter::CrossingGiveWay);
    EXPECT_GT(r.relative_bearing, 0.0);
}

TEST(Encounter, CrossingStandOnTargetOnPort) {
    // Own northbound; target eastbound from the west → crosses from port.
    const auto own = own_at(0, 0, 0.0, 2.0);
    const auto tgt = tgt_at(40, -60, kPi / 2.0, 2.0);
    const auto r = classify_encounter(own, tgt);
    EXPECT_EQ(r.type, Encounter::CrossingStandOn);
    EXPECT_LT(r.relative_bearing, 0.0);
}

TEST(Encounter, OvertakingFromAstern) {
    // Target northbound ahead; own faster, approaching from target's stern.
    const auto own = own_at(0, 0, 0.0, 4.0);
    const auto tgt = tgt_at(50, 0, 0.0, 2.0);
    const auto r = classify_encounter(own, tgt);
    EXPECT_EQ(r.type, Encounter::Overtaking);
}

TEST(Encounter, BeingOvertakenFromAstern) {
    // Faster target approaching from own stern sector.
    const auto own = own_at(50, 0, 0.0, 2.0);
    const auto tgt = tgt_at(0, 0, 0.0, 4.0);
    const auto r = classify_encounter(own, tgt);
    EXPECT_EQ(r.type, Encounter::BeingOvertaken);
}

TEST(Encounter, SafeOpeningIsNone) {
    // Parallel courses, already abeam and opening — no risk.
    const auto own = own_at(0, 0, 0.0, 2.0);
    const auto tgt = tgt_at(0, 100, 0.0, 2.0);
    const auto r = classify_encounter(own, tgt);
    EXPECT_EQ(r.type, Encounter::None);
}

TEST(Encounter, StationaryTargetIsNone) {
    const auto own = own_at(0, 0, 0.0, 2.0);
    Kinematics tgt;
    tgt.x = 40;
    tgt.y = 0;
    tgt.vx = 0;
    tgt.vy = 0;
    const auto r = classify_encounter(own, tgt);
    EXPECT_EQ(r.type, Encounter::None);
}

TEST(Encounter, OvertakingTakesPriorityOverCrossingGeometry) {
    // Own approaching from target's stern quarter (slightly to starboard of
    // the wake). Without Rule 13 priority this can look like a crossing; the
    // stern-sector test must win.
    const auto own = own_at(0, 5, deg2rad(10.0), 4.0);
    const auto tgt = tgt_at(60, 0, 0.0, 2.0);
    const auto r = classify_encounter(own, tgt);
    EXPECT_EQ(r.type, Encounter::Overtaking);
}

TEST(Encounter, EncounterNameCoversAllValues) {
    EXPECT_STREQ(encounter_name(Encounter::None), "None");
    EXPECT_STREQ(encounter_name(Encounter::CrossingGiveWay), "CrossingGiveWay");
    EXPECT_STREQ(encounter_name(Encounter::CrossingStandOn), "CrossingStandOn");
    EXPECT_STREQ(encounter_name(Encounter::Overtaking), "Overtaking");
    EXPECT_STREQ(encounter_name(Encounter::BeingOvertaken), "BeingOvertaken");
}
