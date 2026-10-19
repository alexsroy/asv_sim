#pragma once

#include <ostream>

#include "asv/frame_sink.hpp"

namespace asv {

// Writes one CSV row per frame. The header is written on construction.
// The stream must outlive the sink. Sets the stream's precision.
class CsvFrameSink : public FrameSink {
public:
    explicit CsvFrameSink(std::ostream& os);

    void on_frame(const Frame& frame) override;
    void flush() override;

private:
    std::ostream& os_;
};

}  // namespace asv