#include "raylib_viewer_sink.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "raylib.h"

namespace asv {

namespace {

Vector2 to_v2(Vec2f p) { return Vector2{p.x, p.y}; }

// Primary vessel red with a blue trail; contacts orange with a faded trail.
Color ship_color(bool primary) { return primary ? RED : ORANGE; }
Color trail_color(bool primary) { return primary ? SKYBLUE : Fade(ORANGE, 0.5f); }

}  // namespace

RaylibViewerSink::RaylibViewerSink(std::vector<Waypoint> route,
                                   ViewerOptions options)
    : route_(std::move(route)), opts_(std::move(options)) {
    double min_n = std::numeric_limits<double>::max();
    double max_n = std::numeric_limits<double>::lowest();
    double min_e = min_n;
    double max_e = max_n;
    auto include = [&](const Waypoint& w) {
        min_n = std::min(min_n, w.x);
        max_n = std::max(max_n, w.x);
        min_e = std::min(min_e, w.y);
        max_e = std::max(max_e, w.y);
    };
    for (const auto& w : route_) include(w);
    for (const auto& w : opts_.extra_view_points) include(w);
    if (route_.empty() && opts_.extra_view_points.empty()) {
        min_n = max_n = min_e = max_e = 0.0;
    }
    view_ = ViewTransform::fit(min_n, max_n, min_e, max_e, opts_.width,
                               opts_.height);

    InitWindow(opts_.width, opts_.height, opts_.title.c_str());
    SetTargetFPS(opts_.target_fps);
}

RaylibViewerSink::~RaylibViewerSink() { CloseWindow(); }

void RaylibViewerSink::on_frame(const Frame& f) {
    ShipView& ship = ships_[f.vessel_id];
    ship.state = f.state;
    ship.trail.push_back({f.state.x, f.state.y});

    // Only the primary vessel's frame triggers a redraw, so the draw rate
    // (and playback speed) doesn't change with the number of vessels.
    if (f.vessel_id != opts_.primary_vessel_id) return;
    last_frame_ = f;

    if (++frame_count_ % std::max(opts_.frames_per_draw, 1) == 0) draw();
}

void RaylibViewerSink::on_contact(const Contact& c) {
    contacts_.push_back({c.kind, c.x, c.y, c.t_meas});
}

bool RaylibViewerSink::should_close() {
    if (WindowShouldClose()) closed_ = true;
    return closed_;
}

void RaylibViewerSink::run_until_closed() {
    while (!should_close()) draw();
}

void RaylibViewerSink::draw() {
    BeginDrawing();
    ClearBackground(RAYWHITE);

    // Route: lines between waypoints, dots on the waypoints.
    for (std::size_t i = 0; i + 1 < route_.size(); ++i) {
        DrawLineEx(to_v2(view_.to_screen(route_[i].x, route_[i].y)),
                   to_v2(view_.to_screen(route_[i + 1].x, route_[i + 1].y)),
                   2.0f, LIGHTGRAY);
    }
    for (const auto& w : route_) {
        DrawCircleV(to_v2(view_.to_screen(w.x, w.y)), 4.0f, GRAY);
    }

    // Sensor contacts, fading with age. Radar: green ring. AIS: blue cross.
    while (!contacts_.empty() &&
           last_frame_.t - contacts_.front().t_meas > opts_.contact_fade_s) {
        contacts_.pop_front();
    }
    for (const auto& c : contacts_) {
        const float age = static_cast<float>(last_frame_.t - c.t_meas);
        const float alpha =
            std::clamp(1.0f - age / static_cast<float>(opts_.contact_fade_s), 0.0f, 1.0f);
        const Vector2 p = to_v2(view_.to_screen(c.x, c.y));
        if (c.kind == SensorKind::Radar) {
            DrawCircleLines(static_cast<int>(p.x), static_cast<int>(p.y), 5.0f,
                            Fade(DARKGREEN, alpha));
        } else {
            DrawLineV(Vector2{p.x - 5.0f, p.y - 5.0f}, Vector2{p.x + 5.0f, p.y + 5.0f},
                      Fade(BLUE, alpha));
            DrawLineV(Vector2{p.x - 5.0f, p.y + 5.0f}, Vector2{p.x + 5.0f, p.y - 5.0f},
                      Fade(BLUE, alpha));
        }
    }

    // Ships: contacts first, primary last so it ends up on top.
    // For each ship, trail first, then the triangle.
    for (int pass = 0; pass < 2; ++pass) {
        for (const auto& [id, ship] : ships_) {
            const bool primary = (id == opts_.primary_vessel_id);
            if (primary != (pass == 1)) continue;

            for (std::size_t i = 1; i < ship.trail.size(); ++i) {
                DrawLineV(to_v2(view_.to_screen(ship.trail[i - 1].x, ship.trail[i - 1].y)),
                          to_v2(view_.to_screen(ship.trail[i].x, ship.trail[i].y)),
                          trail_color(primary));
            }
            const Triangle t = ship_triangle(view_, ship.state.x, ship.state.y,
                                             ship.state.psi, 12.0);
            // raylib wants counter-clockwise vertex order on screen.
            DrawTriangle(to_v2(t.nose), to_v2(t.port), to_v2(t.starboard),
                         ship_color(primary));
        }
    }

    // Simple HUD (primary vessel).
    const double heading_deg =
        std::fmod(last_frame_.state.psi * 180.0 / 3.14159265358979323846 + 360.0, 360.0);
    DrawText(TextFormat("t = %.1f s   u = %.2f m/s   heading = %.0f deg",
                        last_frame_.t, last_frame_.state.u, heading_deg),
             10, 10, 18, DARKGRAY);
    DrawText(TextFormat("cross-track = %.1f m   segment = %d",
                        last_frame_.cross_track,
                        static_cast<int>(last_frame_.segment)),
             10, 32, 18, DARKGRAY);

    // Range to the nearest contact, if there are any.
    double nearest = std::numeric_limits<double>::max();
    for (const auto& [id, ship] : ships_) {
        if (id == opts_.primary_vessel_id) continue;
        nearest = std::min(nearest, std::hypot(ship.state.x - last_frame_.state.x,
                                               ship.state.y - last_frame_.state.y));
    }
    if (nearest < std::numeric_limits<double>::max()) {
        DrawText(TextFormat("nearest contact = %.0f m", nearest), 10, 54, 18, ORANGE);
    }

    EndDrawing();
}

}  // namespace asv