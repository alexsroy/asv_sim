#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <sstream>
#include <vector>

#include "asv/ais_sensor.hpp"
#include "asv/contact.hpp"
#include "asv/radar_sensor.hpp"
#include "asv/rng.hpp"

using namespace asv;

namespace {

constexpr double kDt = 0.1;

// Step a sensor from t = 0 to t_end, collecting everything it delivers, then
// drain whatever is still in the latency queue.
template <class Sensor>
std::vector<Contact> run_sensor(Sensor& s, const VesselState& own,
                                const std::vector<TruthTarget>& targets,
                                double t_end) {
    std::vector<Contact> all;
    for (int i = 0; i * kDt <= t_end; ++i) {
        const double t = i * kDt;
        s.update(t, own, targets);
        auto got = s.poll(t);
        all.insert(all.end(), got.begin(), got.end());
    }
    auto rest = s.poll(t_end + 1.0e6);
    all.insert(all.end(), rest.begin(), rest.end());
    return all;
}

double mean(const std::vector<double>& v) {
    return std::accumulate(v.begin(), v.end(), 0.0) / static_cast<double>(v.size());
}

double stddev(const std::vector<double>& v) {
    const double m = mean(v);
    double acc = 0.0;
    for (double x : v) acc += (x - m) * (x - m);
    return std::sqrt(acc / static_cast<double>(v.size() - 1));
}

// Own ship at the origin, one stationary target 100 m north.
VesselState origin() { return VesselState{}; }
std::vector<TruthTarget> one_target(double north = 100.0, double east = 0.0) {
    TruthTarget t;
    t.id = 7;
    t.state.x = north;
    t.state.y = east;
    return {t};
}

RadarParams clean_radar() {
    RadarParams p;
    p.scan_period = 1.0;
    p.p_detect = 1.0;
    p.clutter_mean = 0.0;
    return p;
}

AisParams clean_ais() {
    AisParams p;
    p.p_drop = 0.0;
    p.latency = 0.0;
    p.latency_jitter = 0.0;
    return p;
}

}  // namespace

// --- Rng ---------------------------------------------------------------------

TEST(Rng, SameSeedSameSequence) {
    Rng a(42), b(42);
    for (int i = 0; i < 100; ++i) EXPECT_EQ(a.normal(), b.normal());
}

TEST(Rng, NormalHasRightMeanAndSigma) {
    Rng r(1);
    std::vector<double> v;
    for (int i = 0; i < 50000; ++i) v.push_back(r.normal(5.0, 2.0));
    EXPECT_NEAR(mean(v), 5.0, 0.05);
    EXPECT_NEAR(stddev(v), 2.0, 0.05);
}

TEST(Rng, PoissonMeanMatches) {
    Rng r(3);
    double sum = 0.0;
    const int n = 50000;
    for (int i = 0; i < n; ++i) sum += r.poisson(2.0);
    EXPECT_NEAR(sum / n, 2.0, 0.05);
}

// --- ContactQueue --------------------------------------------------------------

TEST(ContactQueue, ReleasesInArrivalOrderAndOnlyWhenDue) {
    ContactQueue q;
    for (double ta : {3.0, 1.0, 2.0}) {
        Contact c;
        c.t_avail = ta;
        q.push(c);
    }
    EXPECT_TRUE(q.poll(0.5).empty());
    auto got = q.poll(2.0);
    ASSERT_EQ(got.size(), 2u);
    EXPECT_DOUBLE_EQ(got[0].t_avail, 1.0);
    EXPECT_DOUBLE_EQ(got[1].t_avail, 2.0);
    EXPECT_EQ(q.pending(), 1u);
}

TEST(ContactCsvWriter, WritesHeaderAndRows) {
    std::ostringstream os;
    ContactCsvWriter w(os);
    Contact c;
    c.kind = SensorKind::Ais;
    c.id = 3;
    w.write(1.5, c);
    const std::string s = os.str();
    EXPECT_EQ(std::count(s.begin(), s.end(), '\n'), 2);
    EXPECT_NE(s.find("ais"), std::string::npos);
}

// --- Radar ---------------------------------------------------------------------

TEST(Radar, NoiseMatchesConfiguredSigmas) {
    RadarSensor radar(clean_radar(), 11);
    auto contacts = run_sensor(radar, origin(), one_target(), 20000.0);
    ASSERT_GT(contacts.size(), 15000u);

    std::vector<double> range_err, bearing_err;
    for (const auto& c : contacts) {
        range_err.push_back(std::hypot(c.x, c.y) - 100.0);
        bearing_err.push_back(std::atan2(c.y, c.x) - 0.0);  // target is due north
    }
    const RadarParams p = radar.params();
    EXPECT_NEAR(mean(range_err), 0.0, 0.15);
    EXPECT_NEAR(stddev(range_err), p.sigma_range, 0.05 * p.sigma_range);
    EXPECT_NEAR(mean(bearing_err), 0.0, 0.001);
    EXPECT_NEAR(stddev(bearing_err), p.sigma_bearing, 0.05 * p.sigma_bearing);
}

TEST(Radar, DetectionRateMatchesPDetect) {
    RadarParams p = clean_radar();
    p.p_detect = 0.7;
    RadarSensor radar(p, 5);
    const double t_end = 20000.0;
    auto contacts = run_sensor(radar, origin(), one_target(), t_end);
    const double scans = std::floor(t_end / p.scan_period) + 1.0;
    EXPECT_NEAR(static_cast<double>(contacts.size()) / scans, 0.7, 0.02);
}

TEST(Radar, TargetBeyondMaxRangeIsNeverSeen) {
    RadarSensor radar(clean_radar(), 1);
    auto contacts = run_sensor(radar, origin(), one_target(600.0), 200.0);
    EXPECT_TRUE(contacts.empty());
}

TEST(Radar, ContactsAreDelayedByLatency) {
    RadarParams p = clean_radar();
    p.latency = 0.3;
    RadarSensor radar(p, 1);
    radar.update(0.0, origin(), one_target());  // scan fires at t = 0
    EXPECT_TRUE(radar.poll(0.1).empty());
    EXPECT_TRUE(radar.poll(0.29).empty());
    auto got = radar.poll(0.3);
    ASSERT_EQ(got.size(), 1u);
    EXPECT_DOUBLE_EQ(got[0].t_meas, 0.0);
    EXPECT_DOUBLE_EQ(got[0].t_avail, 0.3);
}

TEST(Radar, ClutterRateAndPlacement) {
    RadarParams p = clean_radar();
    p.p_detect = 0.0;  // no real detections, clutter only
    p.clutter_mean = 2.0;
    RadarSensor radar(p, 9);
    const double t_end = 10000.0;
    auto contacts = run_sensor(radar, origin(), one_target(), t_end);
    const double scans = std::floor(t_end / p.scan_period) + 1.0;
    EXPECT_NEAR(static_cast<double>(contacts.size()) / scans, 2.0, 0.1);
    for (const auto& c : contacts) {
        EXPECT_EQ(c.truth_id, -1);
        EXPECT_EQ(c.id, -1);
        EXPECT_LE(std::hypot(c.x, c.y), p.max_range + 1e-9);
    }
}

TEST(Radar, CovarianceIsConsistentWithPolarNoise) {
    RadarSensor radar(clean_radar(), 2);
    auto contacts = run_sensor(radar, origin(), one_target(200.0, 150.0), 50.0);
    ASSERT_FALSE(contacts.empty());
    const RadarParams p = radar.params();
    for (const auto& c : contacts) {
        const double r = std::hypot(c.x, c.y);
        // The trace of the covariance doesn't depend on orientation.
        EXPECT_NEAR(c.cov_xx + c.cov_yy,
                    p.sigma_range * p.sigma_range +
                        r * r * p.sigma_bearing * p.sigma_bearing,
                    1e-6);
        // Valid covariance: non-negative determinant.
        EXPECT_GE(c.cov_xx * c.cov_yy - c.cov_xy * c.cov_xy, -1e-9);
    }
}

TEST(Radar, ScanTimingDoesNotDependOnStepSize) {
    auto count_with_dt = [](double dt) {
        RadarParams p;
        p.scan_period = 2.0;
        p.p_detect = 1.0;
        p.clutter_mean = 0.0;
        RadarSensor radar(p, 1);
        std::size_t n = 0;
        for (int i = 0; i * dt < 99.0; ++i) {
            radar.update(i * dt, origin(), one_target());
            n += radar.poll(i * dt + 1.0e3).size();
        }
        return n;
    };
    EXPECT_EQ(count_with_dt(0.1), count_with_dt(0.5));
}

TEST(Radar, SameSeedReproducesDifferentSeedDiffers) {
    auto run = [](std::uint32_t seed) {
        RadarSensor radar(RadarParams{}, seed);
        return run_sensor(radar, origin(), one_target(), 100.0);
    };
    auto a = run(4), b = run(4), c = run(5);
    ASSERT_EQ(a.size(), b.size());
    for (std::size_t i = 0; i < a.size(); ++i) {
        EXPECT_EQ(a[i].x, b[i].x);
        EXPECT_EQ(a[i].y, b[i].y);
    }
    bool differs = a.size() != c.size();
    for (std::size_t i = 0; !differs && i < a.size(); ++i) differs = a[i].x != c[i].x;
    EXPECT_TRUE(differs);
}

// --- AIS -----------------------------------------------------------------------

TEST(Ais, ReportsOnItsPeriod) {
    AisSensor ais(clean_ais(), 1);
    auto contacts = run_sensor(ais, origin(), one_target(), 29.9);
    ASSERT_EQ(contacts.size(), 10u);  // t = 0, 3, ..., 27
    for (std::size_t i = 1; i < contacts.size(); ++i) {
        EXPECT_NEAR(contacts[i].t_meas - contacts[i - 1].t_meas, 3.0, kDt + 1e-9);
    }
}

TEST(Ais, CarriesIdentityVelocityAndNoise) {
    AisParams p = clean_ais();
    p.report_period = 1.0;
    AisSensor ais(p, 8);
    TruthTarget t;
    t.id = 7;
    t.state.x = 100.0;
    t.state.y = 50.0;
    t.state.psi = 1.0;
    t.state.u = 4.0;
    auto contacts = run_sensor(ais, origin(), {t}, 20000.0);
    ASSERT_GT(contacts.size(), 15000u);

    std::vector<double> ex, ey, esog, ecog;
    for (const auto& c : contacts) {
        EXPECT_EQ(c.id, 7);
        EXPECT_EQ(c.truth_id, 7);
        ASSERT_TRUE(c.has_velocity);
        ex.push_back(c.x - 100.0);
        ey.push_back(c.y - 50.0);
        esog.push_back(c.sog - 4.0);
        ecog.push_back(c.cog - 1.0);
    }
    EXPECT_NEAR(mean(ex), 0.0, 0.1);
    EXPECT_NEAR(stddev(ex), p.sigma_pos, 0.05 * p.sigma_pos);
    EXPECT_NEAR(stddev(ey), p.sigma_pos, 0.05 * p.sigma_pos);
    EXPECT_NEAR(stddev(esog), p.sigma_sog, 0.05 * p.sigma_sog);
    EXPECT_NEAR(stddev(ecog), p.sigma_cog, 0.05 * p.sigma_cog);
}

TEST(Ais, DropoutsComeInBursts) {
    AisParams p = clean_ais();
    p.report_period = 1.0;
    p.p_drop = 0.3;
    p.drop_duration = 10.0;
    AisSensor ais(p, 6);
    auto contacts = run_sensor(ais, origin(), one_target(), 2000.0);
    ASSERT_GT(contacts.size(), 50u);

    int long_gaps = 0;
    for (std::size_t i = 1; i < contacts.size(); ++i) {
        const double gap = contacts[i].t_meas - contacts[i - 1].t_meas;
        // Either a normal 1 s spacing, or a dropout burst of at least ~10 s.
        EXPECT_TRUE(gap < 1.3 || gap > 10.5) << "gap = " << gap;
        if (gap > 10.5) ++long_gaps;
    }
    EXPECT_GT(long_gaps, 0);
}

TEST(Ais, LatencyStaysWithinBounds) {
    AisParams p = clean_ais();
    p.latency = 1.0;
    p.latency_jitter = 2.0;
    AisSensor ais(p, 3);
    auto contacts = run_sensor(ais, origin(), one_target(), 500.0);
    ASSERT_FALSE(contacts.empty());
    for (const auto& c : contacts) {
        EXPECT_GE(c.t_avail - c.t_meas, 1.0 - 1e-9);
        EXPECT_LE(c.t_avail - c.t_meas, 3.0 + 1e-9);
    }
}

TEST(Ais, OutOfReceptionRangeIsSilent) {
    AisSensor ais(clean_ais(), 1);
    auto contacts = run_sensor(ais, origin(), one_target(5000.0), 100.0);
    EXPECT_TRUE(contacts.empty());
}

TEST(Ais, JitterCanDeliverOutOfOrderButPollIsSortedByArrival) {
    AisParams p = clean_ais();
    p.report_period = 1.0;
    p.latency = 0.0;
    p.latency_jitter = 5.0;  // jitter much larger than the period
    AisSensor ais(p, 12);
    ais.update(0.0, origin(), one_target());  // report 1 delayed by jitter
    for (double t = 0.1; t <= 3.0; t += 0.1) ais.update(t, origin(), one_target());
    auto got = ais.poll(100.0);
    ASSERT_GT(got.size(), 2u);
    for (std::size_t i = 1; i < got.size(); ++i) {
        EXPECT_LE(got[i - 1].t_avail, got[i].t_avail);
    }
}