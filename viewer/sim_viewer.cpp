#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <cstdio>
#include <iostream>
#include <optional>
#include <variant>
#include <vector>

#include "asv/ais_sensor.hpp"
#include "asv/contact.hpp"
#include "asv/cpa.hpp"
#include "asv/csv_frame_sink.hpp"
#include "asv/heading_controller.hpp"
#include "asv/kalman_track.hpp"
#include "asv/los_guidance.hpp"
#include "asv/radar_sensor.hpp"
#include "asv/target_ship.hpp"
#include "asv/vessel.hpp"
#include "raylib_viewer_sink.hpp"

namespace {

constexpr int kOwnShipId = 0;
constexpr int kTargetId = 1;

// Safety zone: flag a risk if the target will pass within this distance...
constexpr double kCpaLimit = 30.0;   // [m]
// ...within this time.
constexpr double kTcpaLimit = 60.0;  // [s]

// Radar contacts must fall inside this Mahalanobis gate (chi-square, 2 dof,
// 99%) to update the track.
constexpr double kRadarGate2 = 9.21;

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

    // Sensors watch the target (seeds fixed so runs are repeatable).
    RadarSensor radar(RadarParams{}, 1);
    AisSensor ais(AisParams{}, 2);

    std::ofstream file("run.csv");
    CsvFrameSink csv(file);
    std::ofstream contacts_file("contacts.csv");
    ContactCsvWriter contacts_csv(contacts_file);
    std::ofstream tracks_file("tracks.csv");
    tracks_file << "t,x,y,vx,vy,cov_xx,cov_xy,cov_yy,truth_x,truth_y,truth_vx,truth_vy\n";
    std::optional<KalmanTrack> track;
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
        // frame, so the target and sensor contacts must already be up to date.
        if (target) {
            const Frame tf = target->make_frame();
            csv.on_frame(tf);
            viewer.on_frame(tf);

            // Sensors see ground truth through noise, dropout and latency.
            const std::vector<TruthTarget> truth{{kTargetId, target->state()}};
            radar.update(f.t, s, truth);
            ais.update(f.t, s, truth);
            // Placeholder association until the nearest-neighbour associator:
            // one track, started on the first identified (AIS) contact. AIS is
            // trusted by id; radar must pass the Mahalanobis gate, which is what
            // keeps clutter from dragging the track around.
            auto deliver = [&](const std::vector<Contact>& batch) {
                for (const auto& c : batch) {
                    contacts_csv.write(f.t, c);
                    viewer.on_contact(c);
                    if (!track) {
                        if (c.id != -1) track.emplace(c);
                    } else if (c.id != -1 || track->gate_distance2(c) < kRadarGate2) {
                        track->update(c);
                    }
                }
            };
            deliver(radar.poll(f.t));
            deliver(ais.poll(f.t));

            // CPA from the TRACKED estimate (what the ASV would really know),
            // not from ground truth.
            char buf[96];
            if (!track) {
                std::snprintf(buf, sizeof buf, "no track yet");
            } else {
                const TrackEstimate est = track->estimate_at(f.t);
                viewer.set_tracks({RaylibViewerSink::TrackView{
                    est.kin.x, est.kin.y, est.kin.vx, est.kin.vy,
                    est.P(0, 0), est.P(0, 1), est.P(1, 1)}});

                const auto& truth_state = target->state();
                const Kinematics truth_k = kinematics(truth_state);
                tracks_file << f.t << ',' << est.kin.x << ',' << est.kin.y << ','
                            << est.kin.vx << ',' << est.kin.vy << ',' << est.P(0, 0)
                            << ',' << est.P(0, 1) << ',' << est.P(1, 1) << ','
                            << truth_k.x << ',' << truth_k.y << ',' << truth_k.vx
                            << ',' << truth_k.vy << '\n';

                const CpaResult cpa = compute_cpa(kinematics(s), est.kin);
                if (collision_risk(cpa, kCpaLimit, kTcpaLimit)) {
                    std::snprintf(buf, sizeof buf, "COLLISION RISK: CPA %.0f m in %.0f s",
                                  cpa.dcpa, std::max(cpa.tcpa, 0.0));
                } else if (cpa.closing) {
                    std::snprintf(buf, sizeof buf, "CPA %.0f m in %.0f s", cpa.dcpa, cpa.tcpa);
                } else {
                    std::snprintf(buf, sizeof buf, "CPA: opening (range %.0f m)", cpa.range);
                }
            }
            viewer.set_status(buf);
        }
        csv.on_frame(f);
        viewer.on_frame(f);

        if (viewer.should_close() || guidance.finished()) break;
        s = step_rk4(s, in, params, dt);
        if (target) target->step(dt);
    }
    csv.flush();
    contacts_file.flush();
    tracks_file.flush();

    viewer.run_until_closed();  // leave the final picture up until closed
    return 0;
}