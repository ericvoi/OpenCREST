#include <gtest/gtest.h>
#include <cmath>
#include "dsp/path_loss.hpp"

using openCREST::dsp::thorp_absorption_db_per_km;
using openCREST::dsp::transmission_loss_db;
using openCREST::dsp::path_loss_linear;

// ---------------------------------------------------------------------------
// Thorp absorption
// ---------------------------------------------------------------------------

// Reference values from Thorp (1967) / Urick "Principles of Underwater Sound":
//   α(1 kHz)  ≈ 0.07 dB/km
//   α(10 kHz) ≈ 1.0  dB/km
//   α(25 kHz) ≈ 3.0  dB/km  (approximate)
//   α(50 kHz) ≈ 10   dB/km  (approximate)

TEST(PathLoss, ThorpAt1kHz) {
    const float alpha = thorp_absorption_db_per_km(1.0f);
    EXPECT_NEAR(alpha, 0.07f, 0.02f) << "α(1 kHz)=" << alpha;
}

TEST(PathLoss, ThorpAt10kHz) {
    const float alpha = thorp_absorption_db_per_km(10.0f);
    EXPECT_NEAR(alpha, 1.0f, 0.3f) << "α(10 kHz)=" << alpha;
}

TEST(PathLoss, ThorpAt25kHz) {
    const float alpha = thorp_absorption_db_per_km(25.0f);
    // Should be in the ~2-4 dB/km range
    EXPECT_GT(alpha, 1.5f);
    EXPECT_LT(alpha, 8.0f);
}

TEST(PathLoss, ThorpIncreasesWithFrequency) {
    EXPECT_LT(thorp_absorption_db_per_km(1.0f),
              thorp_absorption_db_per_km(10.0f));
    EXPECT_LT(thorp_absorption_db_per_km(10.0f),
              thorp_absorption_db_per_km(100.0f));
}

TEST(PathLoss, ThorpFreshwaterLessThanSaltwater) {
    const float sw = thorp_absorption_db_per_km(25.0f, true);
    const float fw = thorp_absorption_db_per_km(25.0f, false);
    EXPECT_GT(sw, fw);
}

// ---------------------------------------------------------------------------
// Transmission loss
// ---------------------------------------------------------------------------

TEST(PathLoss, SphericalSpreadingDoubleRange) {
    // With purely spherical spreading (factor=2) and zero absorption:
    // TL = 20·log10(r) → doubling range adds 6 dB
    const float tl1 = transmission_loss_db(100.0f, 0.001f, 2.0f);  // ~0 absorption
    const float tl2 = transmission_loss_db(200.0f, 0.001f, 2.0f);
    EXPECT_NEAR(tl2 - tl1, 6.0f, 0.1f);
}

TEST(PathLoss, CylindricalSpreadingDoubleRange) {
    // Cylindrical spreading (factor=1): doubling range adds 3 dB
    const float tl1 = transmission_loss_db(100.0f, 0.001f, 1.0f);
    const float tl2 = transmission_loss_db(200.0f, 0.001f, 1.0f);
    EXPECT_NEAR(tl2 - tl1, 3.0f, 0.1f);
}

TEST(PathLoss, TLIncreasesWithRange) {
    EXPECT_LT(transmission_loss_db(100.0f, 25.0f),
              transmission_loss_db(1000.0f, 25.0f));
}

TEST(PathLoss, ZeroRangeReturnsZero) {
    EXPECT_FLOAT_EQ(transmission_loss_db(0.0f, 25.0f), 0.0f);
}

// ---------------------------------------------------------------------------
// path_loss_linear
// ---------------------------------------------------------------------------

TEST(PathLoss, LinearGainAtZeroRange) {
    EXPECT_FLOAT_EQ(path_loss_linear(0.0f, 25.0f), 1.0f);
}

TEST(PathLoss, LinearGainBetweenZeroAndOne) {
    const float g = path_loss_linear(150.0f, 25.0f);
    EXPECT_GT(g, 0.0f);
    EXPECT_LT(g, 1.0f);
}

TEST(PathLoss, LinearGainDecreasesWithRange) {
    EXPECT_GT(path_loss_linear(100.0f, 25.0f),
              path_loss_linear(200.0f, 25.0f));
}

TEST(PathLoss, LinearGainConsistentWithTL) {
    const float tl = transmission_loss_db(150.0f, 25.0f, 2.0f, true);
    const float g  = path_loss_linear(150.0f, 25.0f, 2.0f, true);
    EXPECT_NEAR(g, std::pow(10.0f, -tl / 20.0f), 1e-5f);
}
