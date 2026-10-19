#include <fstream>
#include <iostream>
#include <string>

#include "asv/csv_frame_sink.hpp"
#include "asv/heading_controller.hpp"
#include "asv/los_guidance.hpp"
#include "asv/vessel.hpp"

int main(int argc, char** argv) {
    using namespace asv;

    const std::string out_path = (argc > 1) ? argv[1] : "run.csv";
    std::ofstream file(out_path);
    if (!file) {
        std::cerr << "Could not open " << out_path << " for writing\n";
        return 1;
    }
    CsvFrameSink sink(file);

    LosGuidance guidance({{0, 0}, {200, 0}, {200, 200}, {0, 200}}, 20.0, 10.0);
    HeadingController controller;
    const VesselParams params;
    VesselState s;
    s.u = 2.0;

    const double dt = 0.1;
    const double t_max = 600.0;
    const int n_max = static_cast<int>(t_max / dt);

    int steps = 0;
    for (int i = 0; i <= n_max; ++i, ++steps) {
        const double t = i * dt;
        const double psi_d = guidance.update(s.x, s.y);

        VesselInput in;
        in.u_cmd = 2.0;
        in.delta = controller.update(psi_d, s.psi, s.r, dt);

        Frame f;
        f.t = t;
        f.state = s;
        f.psi_ref = psi_d;
        f.delta = in.delta;
        f.u_cmd = in.u_cmd;
        f.cross_track = guidance.cross_track_error(s.x, s.y);
        f.segment = guidance.segment();
        sink.on_frame(f);

        if (guidance.finished()) break;
        s = step_rk4(s, in, params, dt);
    }
    sink.flush();

    std::cout << (guidance.finished() ? "Route complete" : "Timed out")
              << " after " << steps << " steps; wrote " << out_path << "\n";
    return 0;
}