#include "raylib_viewer_sink.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "raylib.h"

namespace asv {

namespace {
Vector2 to_v2(Vec2f p) { return Vector2{p.x, p.y}; }
}  // namespace

RaylibViewerSink::RaylibViewerSink(std::vector<Waypoint> route,
                                   ViewerOptions options)
    : route_(std::move(route)), opts_(std::move(options)) {
    double min_n = std::numeric_limits<double>::max();
    double max_n = std::numeric_limits<double>::lowest();
    double min_e = min_n;
    double max_e = max_n;
    for (const auto& w : route_) {
        min_n = std::min(min_n, w.x);
        max_n = std::max(max_n, w.x);
        min_e = std::min(min_e, w.y);
        max_e = std::max(max_e, w.y);
    }
    if (route_.empty()) min_n = max_n = min_e = max_e = 0.0;
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
    last_frame_ = f;

    if (++frame_count_ % std::max(opts_.frames_per_draw, 1) == 0) draw();
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

    // Ships: trail first, then triangle on top.
    for (const auto& [id, ship] : ships_) {
        for (std::size_t i = 1; i < ship.trail.size(); ++i) {
            DrawLineV(to_v2(view_.to_screen(ship.trail[i - 1].x, ship.trail[i - 1].y)),
                      to_v2(view_.to_screen(ship.trail[i].x, ship.trail[i].y)),
                      SKYBLUE);
        }
        const Triangle t = ship_triangle(view_, ship.state.x, ship.state.y,
                                         ship.state.psi, 12.0);
        // raylib wants counter-clockwise vertex order on screen.
        DrawTriangle(to_v2(t.nose), to_v2(t.port), to_v2(t.starboard), RED);
    }

    // Simple HUD.
    const double heading_deg =
        std::fmod(last_frame_.state.psi * 180.0 / 3.14159265358979323846 + 360.0, 360.0);
    DrawText(TextFormat("t = %.1f s   u = %.2f m/s   heading = %.0f deg",
                        last_frame_.t, last_frame_.state.u, heading_deg),
             10, 10, 18, DARKGRAY);
    DrawText(TextFormat("cross-track = %.1f m   segment = %d",
                        last_frame_.cross_track,
                        static_cast<int>(last_frame_.segment)),
             10, 32, 18, DARKGRAY);

    EndDrawing();
}

}  // namespace asv