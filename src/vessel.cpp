#include "asv/vessel.hpp"

#include <cmath>

namespace asv {

VesselState derivative(const VesselState& s, const VesselInput& in,
                       const VesselParams& p) {
    VesselState d;
    d.x = s.u * std::cos(s.psi);
    d.y = s.u * std::sin(s.psi);
    d.psi = s.r;
    d.r = (p.K * in.delta - s.r) / p.T;
    d.u = (in.u_cmd - s.u) / p.Tu;
    return d;
}

}  // namespace asv


namespace {

// a + b * scale, component-wise
asv::VesselState add_scaled(const asv::VesselState& a,
                            const asv::VesselState& b, double scale) {
    asv::VesselState out;
    out.x = a.x + b.x * scale;
    out.y = a.y + b.y * scale;
    out.psi = a.psi + b.psi * scale;
    out.r = a.r + b.r * scale;
    out.u = a.u + b.u * scale;
    return out;
}

}  // namespace

namespace asv {

VesselState step_rk4(const VesselState& s, const VesselInput& in,
                     const VesselParams& p, double dt) {
    const VesselState k1 = derivative(s, in, p);
    const VesselState k2 = derivative(add_scaled(s, k1, dt / 2.0), in, p);
    const VesselState k3 = derivative(add_scaled(s, k2, dt / 2.0), in, p);
    const VesselState k4 = derivative(add_scaled(s, k3, dt), in, p);

    VesselState out = s;
    out = add_scaled(out, k1, dt / 6.0);
    out = add_scaled(out, k2, dt / 3.0);
    out = add_scaled(out, k3, dt / 3.0);
    out = add_scaled(out, k4, dt / 6.0);
    return out;
}

}  // namespace asv