#include "asv/csv_frame_sink.hpp"

#include <iomanip>

namespace asv {

CsvFrameSink::CsvFrameSink(std::ostream& os) : os_(os) {
    os_ << std::setprecision(9);
    os_ << "t,vessel_id,x,y,psi,r,u,psi_ref,delta,u_cmd,cross_track,segment\n";
}

void CsvFrameSink::on_frame(const Frame& f) {
    os_ << f.t << ',' << f.vessel_id << ',' << f.state.x << ',' << f.state.y
        << ',' << f.state.psi << ',' << f.state.r << ',' << f.state.u << ','
        << f.psi_ref << ',' << f.delta << ',' << f.u_cmd << ','
        << f.cross_track << ',' << f.segment << '\n';
}

void CsvFrameSink::flush() { os_.flush(); }

}  // namespace asv