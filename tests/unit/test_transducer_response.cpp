#include <gtest/gtest.h>
#include <array>
#include <cmath>
#include "dsp/transducer_response.hpp"

using namespace openCREST::dsp;

TEST(FlatResponse, GainDbAtIsConstantAcrossFrequencies) {
    FlatResponse r(20.0f);
    for (float f : {1.0f, 1'000.0f, 25'000.0f, 250'000.0f}) {
        EXPECT_FLOAT_EQ(r.gain_db_at(f), 20.0f);
    }
}

TEST(FlatResponse, ApplyMultipliesByPrecomputedLinearGain) {
    FlatResponse r(20.0f);  // 10x linear
    EXPECT_NEAR(r.gain_lin(), 10.0f, 1e-5f);

    std::array<float, 256> in{}, out{};
    for (size_t i = 0; i < in.size(); ++i) {
        in[i] = static_cast<float>(i) * 0.01f;
    }
    r.apply(in.data(), out.data(), in.size());

    for (size_t i = 0; i < in.size(); ++i) {
        EXPECT_NEAR(out[i], in[i] * 10.0f, 1e-5f) << "i=" << i;
    }
}

TEST(FlatResponse, ApplyHandlesAlias) {
    // out aliasing in must work because real call sites will reuse the
    // same scratch buffer for in/out in the hot path.
    FlatResponse r(-6.0f);  // ~0.5 linear
    std::array<float, 8> buf{};
    for (size_t i = 0; i < buf.size(); ++i) buf[i] = 1.0f;

    r.apply(buf.data(), buf.data(), buf.size());

    const float expected = std::pow(10.0f, -6.0f / 20.0f);
    for (float v : buf) {
        EXPECT_NEAR(v, expected, 1e-5f);
    }
}

TEST(FlatResponse, ZeroGainDbIsIdentity) {
    FlatResponse r(0.0f);
    EXPECT_FLOAT_EQ(r.gain_lin(), 1.0f);
    std::array<float, 4> in{0.1f, -0.2f, 0.3f, -0.4f};
    std::array<float, 4> out{};
    r.apply(in.data(), out.data(), in.size());
    for (size_t i = 0; i < 4; ++i) EXPECT_FLOAT_EQ(out[i], in[i]);
}

TEST(FlatResponse, NegativeGainAttenuates) {
    FlatResponse r(-40.0f);  // 0.01x
    EXPECT_NEAR(r.gain_lin(), 0.01f, 1e-6f);
    float in = 1.0f, out = 0.0f;
    r.apply(&in, &out, 1);
    EXPECT_NEAR(out, 0.01f, 1e-6f);
}
