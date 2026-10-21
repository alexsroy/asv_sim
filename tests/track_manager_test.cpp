#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "asv/ais_sensor.hpp"
#include "asv/contact.hpp"
#include "asv/radar_sensor.hpp"
#include "asv/target_ship.hpp"
#include "asv/track_manager.hpp"
#include "asv/vessel.hpp"

using namespace asv;

namespace {

constexpr double kPi = 3.14159265358979323846;

Contact ais(int id, double t, double x, double y, double sog, double cog) {
    Contact c;
    c.kind = SensorKind::Ais;
    c.id = id;
    c.t_meas = t;
    c.t_avail = t;
    c.x = x;
    c.y = y;
    c.cov_xx = 4.0;
    c.cov_yy = 4.0;
    c.has_velocity = true;
    c.sog = sog;
    c.cog = cog;
    c.sigma_sog = 0.1;
    c.sigma_cog = 0.02;
    return c;
}

Contact radar(double t, double x, double y, double var = 9.0) {
    Contact c;
    c.kind = SensorKind::Radar;
    c.id = -1;
    c.t_meas = t;
    c.t_avail = t;
    c.x = x;
    c.y = y;
    c.cov_xx = var;
    c.cov_yy = var;
    return c;
}

}  // namespace

TEST(TrackManager, AisBirthCreatesOneTrackPerIdentity) {
    TrackManager mgr;
    auto s = mgr.process(0.0, {ais(10, 0.0, 0.0, 0.0, 3.0, 0.0),
                               ais(20, 0.0, 100.0, 0.0, 3.0, kPi / 2.0)});
    EXPECT_EQ(s.ais_births, 2);
    EXPECT_EQ(mgr.size(), 2u);
    ASSERT_NE(mgr.find(10), nullptr);
    ASSERT_NE(mgr.find(20), nullptr);
}

TEST(TrackManager, AisUpdatesExistingTrackById) {
    TrackManager mgr;
    mgr.process(0.0, {ais(7, 0.0, 0.0, 0.0, 2.0, 0.0)});
    auto s = mgr.process(1.0, {ais(7, 1.0, 2.0, 0.0, 2.0, 0.0)});
    EXPECT_EQ(s.ais_births, 0);
    EXPECT_EQ(s.ais_updates, 1);
    EXPECT_EQ(mgr.size(), 1u);
    const auto e = mgr.find(7)->filter.estimate();
    EXPECT_NEAR(e.kin.x, 2.0, 0.5);
}

TEST(TrackManager, RadarAssignedByMahalanobisNearestNeighbour) {
    TrackManager mgr;
    mgr.process(0.0, {ais(1, 0.0, 0.0, 0.0, 0.0, 0.0),
                      ais(2, 0.0, 0.0, 200.0, 0.0, 0.0)});

    // One hit near track 1, one near track 2, one clutter far away.
    auto s = mgr.process(0.5, {radar(0.5, 1.0, 0.0), radar(0.5, 0.0, 201.0),
                               radar(0.5, 500.0, 500.0)});
    EXPECT_EQ(s.radar_assigned, 2);
    EXPECT_EQ(s.radar_rejected, 1);
    EXPECT_EQ(mgr.totals().radar_assigned, 2);
}

TEST(TrackManager, CompetingRadarHitsPreferCloserTrackOneToOne) {
    TrackManager mgr;
    mgr.process(0.0, {ais(1, 0.0, 0.0, 0.0, 0.0, 0.0),
                      ais(2, 0.0, 0.0, 100.0, 0.0, 0.0)});

    // Two hits both near track 1. GNN assigns the nearer one only (one-to-one);
    // the second is left unassigned because track 2 is outside the gate.
    auto s = mgr.process(0.2, {radar(0.2, 0.0, 3.0), radar(0.2, 0.0, 5.0)});
    EXPECT_EQ(s.radar_assigned, 1);
    EXPECT_EQ(s.radar_rejected, 1);
    EXPECT_GT(mgr.find(1)->filter.updates(), 0);
    EXPECT_EQ(mgr.find(2)->filter.updates(), 0);
}

TEST(TrackManager, ClutterDoesNotCreateTracks) {
    TrackManager mgr;
    auto s = mgr.process(0.0, {radar(0.0, 10.0, 10.0), radar(0.0, 20.0, 20.0)});
    EXPECT_EQ(mgr.size(), 0u);
    EXPECT_EQ(s.radar_rejected, 2);
    EXPECT_EQ(s.ais_births, 0);
}

TEST(TrackManager, PruneDropsStaleTracks) {
    TrackManagerParams p;
    p.coast_timeout = 5.0;
    TrackManager mgr(p);
    mgr.process(0.0, {ais(1, 0.0, 0.0, 0.0, 1.0, 0.0)});
    EXPECT_EQ(mgr.size(), 1u);
    mgr.prune(6.0);
    EXPECT_EQ(mgr.size(), 0u);
}

TEST(TrackManager, EstimatesAtReturnsOnePerTrack) {
    TrackManager mgr;
    mgr.process(0.0, {ais(1, 0.0, 0.0, 0.0, 3.0, 0.0),
                      ais(2, 0.0, 50.0, 50.0, 2.0, kPi / 2.0)});
    const auto est = mgr.estimates_at(2.0);
    ASSERT_EQ(est.size(), 2u);
    EXPECT_EQ(est[0].first, 1);
    EXPECT_EQ(est[1].first, 2);
    // Coasting north at 3 m/s for 2 s.
    EXPECT_NEAR(est[0].second.kin.x, 6.0, 0.5);
}

// Two scripted ships, real sensors, clutter: manager must keep two tracks and
// not swap them when they are well separated.
TEST(TrackManagerEndToEnd, HoldsTwoTargetsThroughNoiseAndClutter) {
    auto make_shuttle = [](int id, double y, double psi) {
        Script script;
        script.loop = true;
        script.legs.emplace_back(GoTo{100.0, y - 200.0, 3.0, 15.0});
        script.legs.emplace_back(GoTo{100.0, y + 200.0, 3.0, 15.0});
        VesselState start;
        start.x = 100.0;
        start.y = y;
        start.psi = psi;
        start.u = 3.0;
        TargetParams tp;
        tp.max_speed = 6.0;
        tp.max_turn_rate = 0.1;
        return TargetShip(id, start, tp, script);
    };

    TargetShip a = make_shuttle(1, 0.0, 1.5 * kPi);    // westbound on y = 0
    TargetShip b = make_shuttle(2, 250.0, 0.5 * kPi);  // eastbound on y = 250

    RadarSensor radar(RadarParams{}, 1);
    AisSensor ais_sensor(AisParams{}, 2);
    TrackManager mgr;
    const VesselState own;
    const double dt = 0.1, t_end = 120.0;

    double max_err_1 = 0.0, max_err_2 = 0.0;
    for (int i = 0; i * dt <= t_end; ++i) {
        const double t = i * dt;
        const std::vector<TruthTarget> truth{{1, a.state()}, {2, b.state()}};
        radar.update(t, own, truth);
        ais_sensor.update(t, own, truth);

        std::vector<Contact> batch = radar.poll(t);
        for (const Contact& c : ais_sensor.poll(t)) batch.push_back(c);
        mgr.process(t, batch);
        mgr.prune(t);

        if (t > 20.0) {
            if (const ManagedTrack* t1 = mgr.find(1)) {
                const auto e = t1->filter.estimate_at(t);
                max_err_1 = std::max(
                    max_err_1, std::hypot(e.kin.x - a.state().x, e.kin.y - a.state().y));
            }
            if (const ManagedTrack* t2 = mgr.find(2)) {
                const auto e = t2->filter.estimate_at(t);
                max_err_2 = std::max(
                    max_err_2, std::hypot(e.kin.x - b.state().x, e.kin.y - b.state().y));
            }
        }
        a.step(dt);
        b.step(dt);
    }

    EXPECT_EQ(mgr.size(), 2u);
    EXPECT_LT(max_err_1, 25.0);
    EXPECT_LT(max_err_2, 25.0);
    EXPECT_GT(mgr.totals().radar_assigned, 50);
    EXPECT_GT(mgr.totals().radar_rejected, 0);
}
