#include "asv/target_ship.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace asv {

namespace {

// Local constants so this file doesn't depend on math_constants.hpp's contents.
// Swap for your shared constants if you like.
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

double wrap_pi(double a) {
    a = std::fmod(a + kPi, kTwoPi);
    if (a < 0.0) a += kTwoPi;
    return a - kPi;
}

double wrap_two_pi(double a) {
    a = std::fmod(a, kTwoPi);
    if (a < 0.0) a += kTwoPi;
    return a;
}

double deg2rad(double d) { return d * kPi / 180.0; }

}  // namespace

TargetShip::TargetShip(int vessel_id, VesselState initial, TargetParams params, Script script)
    : vessel_id_(vessel_id),
      state_(initial),
      params_(params),
      script_(std::move(script)),
      finished_(script_.legs.empty()) {
    state_.psi = wrap_two_pi(state_.psi);
    psi_ref_ = state_.psi;
}

void TargetShip::advance_leg() {
    leg_elapsed_ = 0.0;
    ++leg_index_;
    if (leg_index_ >= script_.legs.size()) {
        if (script_.loop && !script_.legs.empty()) {
            leg_index_ = 0;
        } else {
            leg_index_ = script_.legs.size();
            finished_ = true;
        }
    }
}

void TargetShip::command_for_current_leg(double& psi_cmd, double& u_cmd) {
    psi_cmd = state_.psi;
    u_cmd = 0.0;

    // Finishing a leg can immediately start the next (zero-length legs,
    // waypoint already inside its radius). Bound the hops so a degenerate
    // looping script can't spin forever in one step.
    const std::size_t max_hops = script_.legs.size() + 1;
    for (std::size_t hop = 0; hop < max_hops && !finished_; ++hop) {
        const Leg& leg = script_.legs[leg_index_];

        if (const auto* g = std::get_if<GoTo>(&leg)) {
            const double dn = g->x - state_.x;  // north
            const double de = g->y - state_.y;  // east
            if (std::hypot(dn, de) <= g->radius) {
                advance_leg();
                continue;
            }
            psi_cmd = std::atan2(de, dn);  // bearing: 0 = north, clockwise
            u_cmd = g->speed;
            return;
        }
        if (const auto* h = std::get_if<HoldHeading>(&leg)) {
            if (leg_elapsed_ >= h->duration) {
                advance_leg();
                continue;
            }
            psi_cmd = h->psi;
            u_cmd = h->speed;
            return;
        }
        if (const auto* w = std::get_if<Hold>(&leg)) {
            if (leg_elapsed_ >= w->duration) {
                advance_leg();
                continue;
            }
            u_cmd = 0.0;
            return;
        }
    }
}

void TargetShip::step(double dt) {
    if (dt <= 0.0) return;

    double psi_cmd = state_.psi;
    double u_cmd = 0.0;
    if (!finished_) command_for_current_leg(psi_cmd, u_cmd);

    u_cmd = std::clamp(u_cmd, 0.0, params_.max_speed);
    psi_ref_ = wrap_two_pi(psi_cmd);
    u_ref_ = u_cmd;

    // Heading: rate-limited turn toward the commanded bearing.
    const double err = wrap_pi(psi_cmd - state_.psi);
    const double max_dpsi = params_.max_turn_rate * dt;
    const double dpsi = std::clamp(err, -max_dpsi, max_dpsi);
    state_.psi = wrap_two_pi(state_.psi + dpsi);
    state_.r = dpsi / dt;  // yaw rate [rad/s], + = clockwise

    // Speed: accel/decel limited.
    const double du = u_cmd - state_.u;
    const double limit = (du >= 0.0 ? params_.max_accel : params_.max_decel) * dt;
    state_.u += std::clamp(du, -limit, limit);

    // Position: x north, y east, psi clockwise from north.
    state_.x += state_.u * std::cos(state_.psi) * dt;
    state_.y += state_.u * std::sin(state_.psi) * dt;
    t_ += dt;
    leg_elapsed_ += dt;
}

Frame TargetShip::make_frame() const {
    Frame f;
    f.t = t_;
    f.vessel_id = vessel_id_;
    f.state = state_;
    f.psi_ref = psi_ref_;
    f.delta = 0.0;
    f.u_cmd = u_ref_;
    f.cross_track = 0.0;
    f.segment = leg_index_;
    return f;
}

// --- Script parsing --------------------------------------------------------

Script parse_script(std::istream& in) {
    Script script;
    std::string line;
    int line_no = 0;

    auto fail = [&](const std::string& msg) {
        throw std::runtime_error("script line " + std::to_string(line_no) + ": " + msg);
    };

    while (std::getline(in, line)) {
        ++line_no;
        if (auto pos = line.find('#'); pos != std::string::npos) line.erase(pos);

        std::istringstream ss(line);
        std::string cmd;
        if (!(ss >> cmd)) continue;  // blank line

        std::vector<double> a;
        for (double v; ss >> v;) a.push_back(v);
        if (!ss.eof()) fail("expected numbers after '" + cmd + "'");

        if (cmd == "start") {
            if (a.size() != 3 && a.size() != 4) fail("start <x_north> <y_east> <heading_deg> [speed]");
            if (script.start) fail("start given more than once");
            if (a.size() == 4 && a[3] < 0.0) fail("start speed must be >= 0");
            VesselState st;
            st.x = a[0];
            st.y = a[1];
            st.psi = wrap_two_pi(deg2rad(a[2]));
            st.u = a.size() == 4 ? a[3] : 0.0;
            script.start = st;
        } else if (cmd == "goto") {
            if (a.size() != 3 && a.size() != 4) fail("goto <x_north> <y_east> <speed> [radius]");
            GoTo g{a[0], a[1], a[2], a.size() == 4 ? a[3] : 10.0};
            if (g.speed < 0.0 || g.radius <= 0.0) fail("goto needs speed >= 0 and radius > 0");
            script.legs.emplace_back(g);
        } else if (cmd == "heading") {
            if (a.size() != 3) fail("heading <deg> <speed> <duration>");
            if (a[1] < 0.0 || a[2] < 0.0) fail("heading needs speed >= 0 and duration >= 0");
            script.legs.emplace_back(HoldHeading{wrap_two_pi(deg2rad(a[0])), a[1], a[2]});
        } else if (cmd == "hold") {
            if (a.size() != 1 || a[0] < 0.0) fail("hold <duration>");
            script.legs.emplace_back(Hold{a[0]});
        } else if (cmd == "loop") {
            if (!a.empty()) fail("loop takes no arguments");
            script.loop = true;
        } else {
            fail("unknown command '" + cmd + "'");
        }
    }
    return script;
}

Script parse_script_file(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("cannot open script file: " + path);
    return parse_script(f);
}

}  // namespace asv