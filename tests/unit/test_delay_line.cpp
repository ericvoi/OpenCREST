#include <gtest/gtest.h>
#include "dsp/delay_line.hpp"

using openCREST::dsp::DelayLine;

TEST(DelayLine, ZeroBeforeAnyWrite) {
    DelayLine dl(8);
    for (size_t d = 0; d < 8; ++d) {
        EXPECT_FLOAT_EQ(dl.read(d), 0.0f) << "delay=" << d;
    }
}

TEST(DelayLine, DelayZeroReturnsMostRecent) {
    DelayLine dl(8);
    dl.write(1.0f);
    dl.write(2.0f);
    dl.write(3.0f);
    EXPECT_FLOAT_EQ(dl.read(0), 3.0f);
}

TEST(DelayLine, DelayNReturnsNthPast) {
    DelayLine dl(8);
    for (float v = 1.0f; v <= 5.0f; v += 1.0f) dl.write(v);
    // After writing 1,2,3,4,5:
    EXPECT_FLOAT_EQ(dl.read(0), 5.0f);
    EXPECT_FLOAT_EQ(dl.read(1), 4.0f);
    EXPECT_FLOAT_EQ(dl.read(2), 3.0f);
    EXPECT_FLOAT_EQ(dl.read(3), 2.0f);
    EXPECT_FLOAT_EQ(dl.read(4), 1.0f);
}

TEST(DelayLine, WriteBlock) {
    DelayLine dl(16);
    float data[] = {10.0f, 20.0f, 30.0f, 40.0f};
    dl.write_block(data, 4);
    EXPECT_FLOAT_EQ(dl.read(0), 40.0f);
    EXPECT_FLOAT_EQ(dl.read(1), 30.0f);
    EXPECT_FLOAT_EQ(dl.read(2), 20.0f);
    EXPECT_FLOAT_EQ(dl.read(3), 10.0f);
}

TEST(DelayLine, WrapAround) {
    // Capacity 4; write 6 samples — oldest 2 should be gone
    DelayLine dl(4);
    for (float v = 1.0f; v <= 6.0f; v += 1.0f) dl.write(v);
    EXPECT_FLOAT_EQ(dl.read(0), 6.0f);
    EXPECT_FLOAT_EQ(dl.read(1), 5.0f);
    EXPECT_FLOAT_EQ(dl.read(2), 4.0f);
    EXPECT_FLOAT_EQ(dl.read(3), 3.0f);
}

TEST(DelayLine, Reset) {
    DelayLine dl(8);
    dl.write(42.0f);
    dl.reset();
    EXPECT_FLOAT_EQ(dl.read(0), 0.0f);
}

TEST(DelayLine, CapacityAccessor) {
    DelayLine dl(32);
    EXPECT_EQ(dl.capacity(), 32u);
}
