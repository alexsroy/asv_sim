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


namespace {

// Simulate from rest to t_end with a fixed step; returns final state.
VesselState simulate(const VesselInput& in, const VesselParams& p,
                     double dt, double t_end) {
    VesselState s;
    const int n = static_cast<int>(std::round(t_end / dt));
    for (int i = 0; i < n; ++i) s = step_rk4(s, in, p, dt);
    return s;
}

}  // namespace

TEST(StepRk4, MatchesExactSolutionForConstantInputs) {
    VesselParams p;
    VesselInput in;
    in.delta = 0.1;
    in.u_cmd = 3.0;

    const double t = 10.0;
    VesselState s = simulate(in, p, 0.1, t);

    const double r_exact = p.K * in.delta * (1.0 - std::exp(-t / p.T));
    const double psi_exact =
        p.K * in.delta * (t - p.T * (1.0 - std::exp(-t / p.T)));
    const double u_exact = in.u_cmd * (1.0 - std::exp(-t / p.Tu));

    EXPECT_NEAR(s.r, r_exact, 1e-8);
    EXPECT_NEAR(s.psi, psi_exact, 1e-8);
    EXPECT_NEAR(s.u, u_exact, 1e-8);
}

TEST(StepRk4, StraightLineCoversExpectedDistance) {
    // Already at speed, rudder centered: should travel u * t due north.
    VesselParams p;
    VesselInput in;
    in.u_cmd = 2.0;
    VesselState s;
    s.u = 2.0;
    for (int i = 0; i < 100; ++i) s = step_rk4(s, in, p, 0.1);
    EXPECT_NEAR(s.x, 20.0, 1e-9);
    EXPECT_NEAR(s.y, 0.0, 1e-9);
}

TEST(StepRk4, ErrorShrinksAtFourthOrder) {
    // Halving dt should cut the error by roughly 2^4 = 16.
    VesselParams p;
    VesselInput in;
    in.delta = 0.1;
    const double t = 10.0;
    const double r_exact = p.K * in.delta * (1.0 - std::exp(-t / p.T));

    const double err_coarse = std::abs(simulate(in, p, 1.0, t).r - r_exact);
    const double err_fine = std::abs(simulate(in, p, 0.5, t).r - r_exact);

    EXPECT_GT(err_coarse / err_fine, 12.0);
}