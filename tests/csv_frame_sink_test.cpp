#include <gtest/gtest.h>

#include <sstream>
#include <string>
#include <vector>

#include "asv/csv_frame_sink.hpp"

using namespace asv;

namespace {

std::vector<std::string> split(const std::string& line, char sep = ',') {
    std::vector<std::string> out;
    std::stringstream ss(line);
    std::string item;
    while (std::getline(ss, item, sep)) out.push_back(item);
    return out;
}

std::vector<std::string> lines_of(const std::string& text) {
    return split(text, '\n');
}

}  // namespace

TEST(CsvFrameSink, HeaderIsFirstLine) {
    std::ostringstream os;
    CsvFrameSink sink(os);
    auto lines = lines_of(os.str());
    ASSERT_EQ(lines.size(), 1u);
    EXPECT_EQ(lines[0],
              "t,vessel_id,x,y,psi,r,u,psi_ref,delta,u_cmd,cross_track,segment");
}

TEST(CsvFrameSink, WritesOneRowPerFrame) {
    std::ostringstream os;
    CsvFrameSink sink(os);
    for (int i = 0; i < 5; ++i) sink.on_frame(Frame{});
    EXPECT_EQ(lines_of(os.str()).size(), 6u);  // header + 5 rows
}

TEST(CsvFrameSink, RowValuesAppearInColumnOrder) {
    std::ostringstream os;
    CsvFrameSink sink(os);

    Frame f;
    f.t = 1.5;
    f.vessel_id = 2;
    f.state.x = 10.0;
    f.state.y = -4.25;
    f.state.psi = 0.5;
    f.state.r = 0.01;
    f.state.u = 2.0;
    f.psi_ref = 0.75;
    f.delta = -0.1;
    f.u_cmd = 3.0;
    f.cross_track = 1.25;
    f.segment = 3;
    sink.on_frame(f);

    auto lines = lines_of(os.str());
    ASSERT_EQ(lines.size(), 2u);
    auto cols = split(lines[1]);
    ASSERT_EQ(cols.size(), 12u);

    EXPECT_DOUBLE_EQ(std::stod(cols[0]), 1.5);
    EXPECT_EQ(std::stoi(cols[1]), 2);
    EXPECT_DOUBLE_EQ(std::stod(cols[2]), 10.0);
    EXPECT_DOUBLE_EQ(std::stod(cols[3]), -4.25);
    EXPECT_DOUBLE_EQ(std::stod(cols[4]), 0.5);
    EXPECT_DOUBLE_EQ(std::stod(cols[5]), 0.01);
    EXPECT_DOUBLE_EQ(std::stod(cols[6]), 2.0);
    EXPECT_DOUBLE_EQ(std::stod(cols[7]), 0.75);
    EXPECT_DOUBLE_EQ(std::stod(cols[8]), -0.1);
    EXPECT_DOUBLE_EQ(std::stod(cols[9]), 3.0);
    EXPECT_DOUBLE_EQ(std::stod(cols[10]), 1.25);
    EXPECT_EQ(std::stoul(cols[11]), 3u);
}

TEST(FrameSink, UsableThroughBasePointer) {
    std::ostringstream os;
    CsvFrameSink csv(os);
    FrameSink* sink = &csv;
    sink->on_frame(Frame{});
    sink->flush();
    EXPECT_EQ(lines_of(os.str()).size(), 2u);
}