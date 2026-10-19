#pragma once

#include <algorithm>
#include <cmath>

namespace asv {

struct Vec2f {
    float x;
    float y;
};

// Maps world coordinates (north, east in meters) to screen pixels.
struct ViewTransform {
    double center_north = 0.0;
    double center_east = 0.0;
    double pixels_per_meter = 1.0;
    int width = 800;
    int height = 600;

    Vec2f to_screen(double north, double east) const {
        return {
            static_cast<float>(width / 2.0 + (east - center_east) * pixels_per_meter),
            static_cast<float>(height / 2.0 - (north - center_north) * pixels_per_meter)};
    }

    // Choose center and scale so the given bounds fit inside the window.
    static ViewTransform fit(double min_north, double max_north,
                             double min_east, double max_east, int width,
                             int height, double margin_px = 50.0) {
        ViewTransform v;
        v.width = width;
        v.height = height;
        v.center_north = 0.5 * (min_north + max_north);
        v.center_east = 0.5 * (min_east + max_east);
        const double span_n = std::max(max_north - min_north, 1.0);
        const double span_e = std::max(max_east - min_east, 1.0);
        v.pixels_per_meter = std::min((width - 2.0 * margin_px) / span_e,
                                      (height - 2.0 * margin_px) / span_n);
        return v;
    }
};

struct Triangle {
    Vec2f nose;
    Vec2f port;
    Vec2f starboard;
};

// Ship marker in screen space. size_px is the nose distance from the center.
inline Triangle ship_triangle(const ViewTransform& v, double north,
                              double east, double psi, double size_px) {
    const Vec2f c = v.to_screen(north, east);
    // Unit vectors on screen: forward, and starboard (right of forward).
    const float fx = static_cast<float>(std::sin(psi));
    const float fy = static_cast<float>(-std::cos(psi));
    const float sx = static_cast<float>(std::cos(psi));
    const float sy = static_cast<float>(std::sin(psi));
    const float s = static_cast<float>(size_px);
    const float back = 0.6f * s;
    const float half = 0.5f * s;
    return {{c.x + fx * s, c.y + fy * s},
            {c.x - fx * back - sx * half, c.y - fy * back - sy * half},
            {c.x - fx * back + sx * half, c.y - fy * back + sy * half}};
}

}  // namespace asv