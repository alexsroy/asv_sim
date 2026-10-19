#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <vector>

#include "asv/csv_frame_sink.hpp"
#include "asv/heading_controller.hpp"
#include "asv/los_guidance.hpp"
#include "asv/vessel.hpp"
#include "raylib_viewer_sink.hpp"

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

    std::ofstream file("run.csv");
    CsvFrameSink csv(file);

    ViewerOptions opts;
    opts.target_fps = std::clamp(static_cast<int>(std::lround(speed / dt)), 5, 120);
    RaylibViewerSink viewer(route, opts);

    const int n_max = static_cast<int>(600.0 / dt);
    for (int i = 0; i <= n_max; ++i) {
        const double psi_d = guidance.update(s.x, s.y);

        VesselInput in;
        in.u_cmd = 2.0;
        in.delta = controller.update(psi_d, s.psi, s.r, dt);

        Frame f;
        f.t = i * dt;
        f.state = s;
        f.psi_ref = psi_d;
        f.delta = in.delta;
        f.u_cmd = in.u_cmd;
        f.cross_track = guidance.cross_track_error(s.x, s.y);
        f.segment = guidance.segment();

        csv.on_frame(f);
        viewer.on_frame(f);

        if (viewer.should_close() || guidance.finished()) break;
        s = step_rk4(s, in, params, dt);
    }
    csv.flush();

    viewer.run_until_closed();  // leave the final picture up until closed
    return 0;
}