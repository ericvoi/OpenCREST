#include <gtest/gtest.h>
#include "core/sample_conversion.hpp"

using openCREST::adc_to_float;
using openCREST::float_to_dac;

// ---------------------------------------------------------------------------
// adc_to_float
// ---------------------------------------------------------------------------

TEST(SampleConversion, AdcMidpointIsZero) {
    EXPECT_FLOAT_EQ(adc_to_float(32768u, 16), 0.0f);  // 16-bit: midpoint = 32768
}

TEST(SampleConversion, AdcZeroIsMinusOne) {
    EXPECT_FLOAT_EQ(adc_to_float(0u, 16), -1.0f);
}

TEST(SampleConversion, AdcFullScaleIsNearPlusOne) {
    // 0xFFFF / 32768 - 1 = 65535/32768 - 1 ≈ +0.9999...
    const float v = adc_to_float(65535u, 16);
    EXPECT_GT(v, 0.999f);
    EXPECT_LE(v, 1.0f + 1e-4f);
}

TEST(SampleConversion, Adc12Bit) {
    // 12-bit midpoint = 2048
    EXPECT_FLOAT_EQ(adc_to_float(2048u, 12), 0.0f);
    EXPECT_FLOAT_EQ(adc_to_float(0u, 12), -1.0f);
}

// ---------------------------------------------------------------------------
// float_to_dac
// ---------------------------------------------------------------------------

TEST(SampleConversion, DacZeroFloat) {
    EXPECT_EQ(float_to_dac(0.0f, 16), 32768u);
}

TEST(SampleConversion, DacPlusOneFloat) {
    // +1.0 * 32768 + 32768 = 65536, clamped to 65535
    EXPECT_EQ(float_to_dac(1.0f, 16), 65535u);
}

TEST(SampleConversion, DacMinusOneFloat) {
    EXPECT_EQ(float_to_dac(-1.0f, 16), 0u);
}

TEST(SampleConversion, DacClipsAboveOne) {
    EXPECT_EQ(float_to_dac(2.0f, 16), 65535u);
}

TEST(SampleConversion, DacClipsBelowMinusOne) {
    EXPECT_EQ(float_to_dac(-2.0f, 16), 0u);
}

// ---------------------------------------------------------------------------
// Round-trip: float → DAC → float
// ---------------------------------------------------------------------------

TEST(SampleConversion, RoundTripWithinOneLSB) {
    // For 16-bit the LSB width is 1/32768 ≈ 3.05e-5
    const float lsb = 1.0f / 32768.0f;
    for (float v : {-0.9f, -0.5f, -0.1f, 0.0f, 0.1f, 0.5f, 0.9f}) {
        const uint16_t raw  = float_to_dac(v, 16);
        const float    back = adc_to_float(raw, 16);
        EXPECT_NEAR(back, v, lsb + 1e-6f) << "round-trip failure at v=" << v;
    }
}
