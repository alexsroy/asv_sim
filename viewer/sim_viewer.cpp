#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <cstdio>
#include <iostream>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "asv/ais_sensor.hpp"
#include "asv/contact.hpp"
#include "asv/cpa.hpp"
#include "asv/csv_frame_sink.hpp"
#include "asv/encounter.hpp"
#include "asv/heading_controller.hpp"
#include "asv/heading_planner.hpp"
#include "asv/los_guidance.hpp"
#include "asv/radar_sensor.hpp"
#include "asv/target_ship.hpp"
#include "asv/track_manager.hpp"
#include "asv/vessel.hpp"
#include "raylib_viewer_sink.hpp"

namespace {

constexpr int kOwnShipId = 0;
constexpr int kFirstTargetId = 1;

// Safety zone: flag a risk if a target will pass within this distance...
constexpr double kCpaLimit = 30.0;   // [m]
// ...within this time.
constexpr double kTcpaLimit = 60.0;  // [s]

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

// Usage: sim_viewer [speed] [course...]
//   Multiple .course files load multiple scripted targets (AIS ids 1, 2, ...).
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

    std::vector<TargetShip> targets;
    for (int a = 2; a < argc; ++a) {
        try {
            Script script = parse_script_file(argv[a]);
            const VesselState start = script.start.value_or(VesselState{});
            TargetParams tp;
            tp.max_speed = 6.0;
            tp.max_turn_rate = 0.1;  // rad/s; at 3 m/s that's a 30 m turn radius
            auto pts = target_view_points(script, start, 70.0);
            opts.extra_view_points.insert(opts.extra_view_points.end(), pts.begin(),
                                          pts.end());
            const int id = kFirstTargetId + static_cast<int>(targets.size());
            targets.emplace_back(id, start, tp, std::move(script));
        } catch (const std::exception& e) {
            std::cerr << "Failed to load scenario '" << argv[a] << "': " << e.what()
                      << '\n';
            return 1;
        }
    }

    RadarSensor radar(RadarParams{}, 1);
    AisSensor ais(AisParams{}, 2);
    TrackManager tracker;

    std::ofstream file("run.csv");
    CsvFrameSink csv(file);
    std::ofstream contacts_file("contacts.csv");
    ContactCsvWriter contacts_csv(contacts_file);
    std::ofstream tracks_file("tracks.csv");
    tracks_file << "t,id,x,y,vx,vy,cov_xx,cov_xy,cov_yy\n";
    RaylibViewerSink viewer(route, opts);

    const int n_max = static_cast<int>(600.0 / dt);
    for (int i = 0; i <= n_max; ++i) {
        const double t = i * dt;
        const double psi_los = guidance.update(s.x, s.y);
        double psi_cmd = psi_los;

        std::vector<RaylibViewerSink::TrackView> views;
        char status[192] = "";
        std::optional<HeadingPlan> plan;

        // Sense and track before planning so the COLREGS command uses current
        // estimates. Target frames are drawn before own-ship (primary redraw).
        if (!targets.empty()) {
            std::vector<TruthTarget> truth;
            truth.reserve(targets.size());
            for (auto& tgt : targets) {
                const Frame tf = tgt.make_frame();
                csv.on_frame(tf);
                viewer.on_frame(tf);
                truth.push_back({tgt.vessel_id(), tgt.state()});
            }

            radar.update(t, s, truth);
            ais.update(t, s, truth);

            std::vector<Contact> batch = radar.poll(t);
            for (const Contact& c : ais.poll(t)) batch.push_back(c);
            for (const Contact& c : batch) {
                contacts_csv.write(t, c);
                viewer.on_contact(c);
            }
            tracker.process(t, batch);
            tracker.prune(t);

            if (tracker.size() == 0) {
                std::snprintf(status, sizeof status, "no tracks yet");
            } else {
                const Kinematics own_k = kinematics(s);
                bool any_risk = false;
                bool any_closing = false;
                CpaResult worst{};
                worst.dcpa = 1e9;
                int worst_id = -1;
                Kinematics worst_kin;

                for (const auto& [id, est] : tracker.estimates_at(t)) {
                    views.push_back({est.kin.x, est.kin.y, est.kin.vx, est.kin.vy,
                                     est.P(0, 0), est.P(0, 1), est.P(1, 1)});
                    tracks_file << t << ',' << id << ',' << est.kin.x << ','
                                << est.kin.y << ',' << est.kin.vx << ',' << est.kin.vy
                                << ',' << est.P(0, 0) << ',' << est.P(0, 1) << ','
                                << est.P(1, 1) << '\n';

                    const CpaResult cpa = compute_cpa(own_k, est.kin);
                    if (collision_risk(cpa, kCpaLimit, kTcpaLimit)) {
                        any_risk = true;
                        if (cpa.dcpa < worst.dcpa) {
                            worst = cpa;
                            worst_id = id;
                            worst_kin = est.kin;
                        }
                    } else if (cpa.closing) {
                        any_closing = true;
                        if (cpa.dcpa < worst.dcpa) {
                            worst = cpa;
                            worst_id = id;
                            worst_kin = est.kin;
                        }
                    } else if (worst_id < 0 || cpa.range < worst.range) {
                        worst = cpa;
                        worst_id = id;
                        worst_kin = est.kin;
                    }
                }

                const EncounterResult enc = classify_encounter(s, worst_kin);
                plan = plan_heading(s, psi_los, worst_kin, enc.type);
                psi_cmd = plan->psi_cmd;

                const char* label = encounter_name(enc.type);
                if (any_risk) {
                    std::snprintf(status, sizeof status,
                                  "RISK id %d %s: CPA %.0f m in %.0f s%s", worst_id,
                                  label, plan->best.cpa.dcpa,
                                  std::max(plan->best.cpa.tcpa, 0.0),
                                  plan->active ? " [avoid]" : "");
                } else if (any_closing) {
                    std::snprintf(status, sizeof status,
                                  "CPA id %d %s: %.0f m in %.0f s (%zu tracks)", worst_id,
                                  label, worst.dcpa, worst.tcpa, tracker.size());
                } else {
                    std::snprintf(status, sizeof status,
                                  "CPA: opening id %d %s range %.0f m (%zu tracks)",
                                  worst_id, label, worst.range, tracker.size());
                }
            }
            viewer.set_tracks(std::move(views));
            viewer.set_status(status);
        }

        VesselInput in;
        in.u_cmd = 2.0;
        in.delta = controller.update(psi_cmd, s.psi, s.r, dt);

        Frame f;
        f.t = t;
        f.vessel_id = kOwnShipId;
        f.state = s;
        f.psi_ref = psi_cmd;
        f.delta = in.delta;
        f.u_cmd = in.u_cmd;
        f.cross_track = guidance.cross_track_error(s.x, s.y);
        f.segment = guidance.segment();
        csv.on_frame(f);
        viewer.on_frame(f);

        if (viewer.should_close() || guidance.finished()) break;
        s = step_rk4(s, in, params, dt);
        for (auto& tgt : targets) tgt.step(dt);
    }
    csv.flush();
    contacts_file.flush();
    tracks_file.flush();

    viewer.run_until_closed();  // leave the final picture up until closed
    return 0;
}
