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

    // Tracks: 95% position-uncertainty ellipse, centre mark, 10 s velocity line.
    for (const auto& tr : tracks_) {
        constexpr double kPi = 3.14159265358979323846;
        constexpr double kChi2_95 = 5.991;  // 2 degrees of freedom
        const double a = tr.cov_xx, b = tr.cov_xy, c = tr.cov_yy;
        const double mid = 0.5 * (a + c);
        const double rad = std::sqrt(0.25 * (a - c) * (a - c) + b * b);
        const double major = std::sqrt(std::max(mid + rad, 0.0) * kChi2_95);
        const double minor = std::sqrt(std::max(mid - rad, 0.0) * kChi2_95);
        const double theta = 0.5 * std::atan2(2.0 * b, a - c);  // major axis, in (x, y)

        constexpr int kSegments = 40;
        Vector2 prev{};
        for (int i = 0; i <= kSegments; ++i) {
            const double phi = 2.0 * kPi * i / kSegments;
            const double ex = major * std::cos(phi), ey = minor * std::sin(phi);
            const double wx = tr.x + ex * std::cos(theta) - ey * std::sin(theta);
            const double wy = tr.y + ex * std::sin(theta) + ey * std::cos(theta);
            const Vector2 p = to_v2(view_.to_screen(wx, wy));
            if (i > 0) DrawLineV(prev, p, PURPLE);
            prev = p;
        }
        const Vector2 centre = to_v2(view_.to_screen(tr.x, tr.y));
        const Vector2 ahead = to_v2(view_.to_screen(tr.x + tr.vx * 10.0, tr.y + tr.vy * 10.0));
        DrawLineV(centre, ahead, PURPLE);
        DrawCircleV(centre, 3.0f, PURPLE);
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
    if (!status_.empty()) DrawText(status_.c_str(), 10, 76, 18, RED);

    EndDrawing();
}

}  // namespace asv