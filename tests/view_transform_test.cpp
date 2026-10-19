#include <gtest/gtest.h>

#include <cmath>

#include "asv/view_transform.hpp"

using namespace asv;

namespace {
ViewTransform simple_view() {
    ViewTransform v;
    v.width = 800;
    v.height = 600;
    v.pixels_per_meter = 2.0;
    return v;  // centered on world origin
}
}  // namespace

TEST(ViewTransform, CenterMapsToScreenCenter) {
    auto p = simple_view().to_screen(0.0, 0.0);
    EXPECT_FLOAT_EQ(p.x, 400.0f);
    EXPECT_FLOAT_EQ(p.y, 300.0f);
}

TEST(ViewTransform, NorthIsUp) {
    auto p = simple_view().to_screen(10.0, 0.0);
    EXPECT_FLOAT_EQ(p.x, 400.0f);
    EXPECT_FLOAT_EQ(p.y, 280.0f);  // smaller y = higher on screen
}

TEST(ViewTransform, EastIsRight) {
    auto p = simple_view().to_screen(0.0, 10.0);
    EXPECT_FLOAT_EQ(p.x, 420.0f);
    EXPECT_FLOAT_EQ(p.y, 300.0f);
}

TEST(ViewTransform, FitKeepsRouteInsideMargins) {
    auto v = ViewTransform::fit(0, 200, 0, 200, 800, 600, 50.0);
    auto lo = v.to_screen(0, 0);      // south-west corner
    auto hi = v.to_screen(200, 200);  // north-east corner
    EXPECT_GE(lo.x, 50.0f - 1e-3f);
    EXPECT_LE(hi.x, 750.0f + 1e-3f);
    EXPECT_LE(lo.y, 550.0f + 1e-3f);
    EXPECT_GE(hi.y, 50.0f - 1e-3f);
    // Window is wider than tall, so height is the binding dimension.
    EXPECT_NEAR(hi.y, 50.0f, 1e-3f);
    EXPECT_NEAR(lo.y, 550.0f, 1e-3f);
}

TEST(ShipTriangle, HeadingNorthPointsUp) {
    auto v = simple_view();
    auto c = v.to_screen(0, 0);
    auto t = ship_triangle(v, 0, 0, 0.0, 10.0);
    EXPECT_NEAR(t.nose.x, c.x, 1e-4f);
    EXPECT_LT(t.nose.y, c.y);
    EXPECT_GT(t.starboard.x, t.port.x);  // starboard is on the right
}

TEST(ShipTriangle, HeadingEastPointsRight) {
    auto v = simple_view();
    auto c = v.to_screen(0, 0);
    auto t = ship_triangle(v, 0, 0, M_PI / 2.0, 10.0);
    EXPECT_GT(t.nose.x, c.x);
    EXPECT_NEAR(t.nose.y, c.y, 1e-4f);
    EXPECT_GT(t.starboard.y, t.port.y);  // facing east, starboard is south = down
}