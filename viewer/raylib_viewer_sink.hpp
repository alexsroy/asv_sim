#pragma once

#include <map>
#include <string>
#include <vector>

#include "asv/frame_sink.hpp"
#include "asv/los_guidance.hpp"
#include "asv/view_transform.hpp"

namespace asv {

struct ViewerOptions {
    int width = 900;
    int height = 700;
    int target_fps = 30;       // each drawn frame waits to hit this rate
    int frames_per_draw = 1;   // draw every Nth step (counted on the primary vessel)
    std::string title = "ASV simulation";

    // The primary vessel drives the HUD and the redraw cadence. Every other
    // vessel is drawn as a contact. Send contacts' frames BEFORE the primary's
    // each step so they are current when the screen is redrawn.
    int primary_vessel_id = 0;

    // Extra world points (x = north, y = east) to include when fitting the
    // view, so contacts that roam outside the route stay on screen.
    std::vector<Waypoint> extra_view_points;
};

// Draws the route, each ship as a triangle, and its track trail.
// Opens a window on construction and closes it on destruction.
class RaylibViewerSink : public FrameSink {
public:
    RaylibViewerSink(std::vector<Waypoint> route, ViewerOptions options = {});
    ~RaylibViewerSink() override;

    RaylibViewerSink(const RaylibViewerSink&) = delete;
    RaylibViewerSink& operator=(const RaylibViewerSink&) = delete;

    void on_frame(const Frame& frame) override;

    // True once the user closed the window (or pressed Esc).
    bool should_close();

    // Keep showing the last frame until the window is closed.
    void run_until_closed();

private:
    struct ShipView {
        std::vector<Waypoint> trail;  // world positions (x = north, y = east)
        VesselState state;
    };

    void draw();

    std::vector<Waypoint> route_;
    ViewerOptions opts_;
    ViewTransform view_;
    std::map<int, ShipView> ships_;
    Frame last_frame_;  // latest frame of the primary vessel
    long frame_count_ = 0;
    bool closed_ = false;
};

}  // namespace asv