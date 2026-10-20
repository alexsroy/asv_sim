#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <iostream>
#include <optional>
#include <variant>
#include <vector>

#include "asv/csv_frame_sink.hpp"
#include "asv/heading_controller.hpp"
#include "asv/los_guidance.hpp"
#include "asv/target_ship.hpp"
#include "asv/vessel.hpp"
#include "raylib_viewer_sink.hpp"

namespace {

constexpr int kOwnShipId = 0;
constexpr int kTargetId = 1;

// World points around each scripted waypoint (plus the start), padded so the
// target's turns and overshoot stay inside the viewer window.
std::vector<asv::Waypoint> target_view_points(const asv::Script& script,
                                              const asv::VesselState& start,
                                              double pad) {
    std::vector<asv::Waypoint> pts;
    auto add = [&](double n, double e) {
        pts.push_back({n - pad, e - pad});
        pts.push_back({n + pad, e + pad});
    };
    add(start.x, start.y);
    for (const auto& leg : script.legs) {
        if (const auto* g = std::get_if<asv::GoTo>(&leg)) add(g->x, g->y);
    }
    return pts;
}

}  // namespace

// Usage: sim_viewer [speed] [target_scenario.course]
int main(int argc, char** argv) {
    using namespace asv;

    // Sim seconds per real second. Each drawn frame waits for target_fps,
    // so speed = target_fps * dt. Usable range is roughly 0.5x to 12x.
    const double speed = (argc > 1) ? std::atof(argv[1]) : 5.0;
    const double dt = 0.1;

    const std::vector<Waypoint> route{{0, 0}, {200, 0}, {200, 200}, {0, 200}};
    LosGuidance guidance(route, 20.0, 10.0);
    HeadingController controller;
    const VesselParams params;
    VesselState s;
    s.u = 2.0;

    ViewerOptions opts;
    opts.target_fps = std::clamp(static_cast<int>(std::lround(speed / dt)), 5, 120);
    opts.primary_vessel_id = kOwnShipId;

    // Optional scripted target ship.
    std::optional<TargetShip> target;
    if (argc > 2) {
        try {
            Script script = parse_script_file(argv[2]);
            const VesselState start = script.start.value_or(VesselState{});
            TargetParams tp;
            tp.max_speed = 6.0;
            tp.max_turn_rate = 0.1;  // rad/s; at 3 m/s that's a 30 m turn radius
            opts.extra_view_points = target_view_points(script, start, 70.0);
            target.emplace(kTargetId, start, tp, std::move(script));
        } catch (const std::exception& e) {
            std::cerr << "Failed to load scenario '" << argv[2] << "': " << e.what() << '\n';
            return 1;
        }
    }

    std::ofstream file("run.csv");
    CsvFrameSink csv(file);
    RaylibViewerSink viewer(route, opts);

    const int n_max = static_cast<int>(600.0 / dt);
    for (int i = 0; i <= n_max; ++i) {
        const double psi_d = guidance.update(s.x, s.y);

        VesselInput in;
        in.u_cmd = 2.0;
        in.delta = controller.update(psi_d, s.psi, s.r, dt);

        Frame f;
        f.t = i * dt;
        f.vessel_id = kOwnShipId;
        f.state = s;
        f.psi_ref = psi_d;
        f.delta = in.delta;
        f.u_cmd = in.u_cmd;
        f.cross_track = guidance.cross_track_error(s.x, s.y);
        f.segment = guidance.segment();

        // Contacts first, own ship last: the viewer redraws on the primary's
        // frame, so the target must already be up to date.
        if (target) {
            const Frame tf = target->make_frame();
            csv.on_frame(tf);
            viewer.on_frame(tf);
        }
        csv.on_frame(f);
        viewer.on_frame(f);

        if (viewer.should_close() || guidance.finished()) break;
        s = step_rk4(s, in, params, dt);
        if (target) target->step(dt);
    }
    csv.flush();

    viewer.run_until_closed();  // leave the final picture up until closed
    return 0;
}