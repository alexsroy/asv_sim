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