#include "dsp/path_loss.hpp"
#include <cmath>
#include <algorithm>

namespace openCREST::dsp {

float thorp_absorption_db_per_km(float freq_khz, bool saltwater) {
    const float f2 = freq_khz * freq_khz;
    // Thorp (1967), valid 0.1 – 100 kHz.
    float alpha = 0.11f  * f2 / (1.0f    + f2)
                + 44.0f  * f2 / (4100.0f + f2)
                + 2.75e-4f * f2
                + 0.003f;
    if (!saltwater) {
        // Freshwater correction (Francois & Garrison 1982, simplified).
        alpha -= 0.002f * f2 / (f2 + 1.0f);
        alpha  = std::max(alpha, 0.0f);
    }
    return alpha;
}

float transmission_loss_db(float range_m, float freq_khz,
                            float spreading_factor, bool saltwater) {
    if (range_m <= 0.0f) return 0.0f;
    const float alpha_db_km = thorp_absorption_db_per_km(freq_khz, saltwater);
    const float absorption  = alpha_db_km * range_m / 1000.0f;   // dB
    const float spreading   = 10.0f * spreading_factor * std::log10(range_m);
    return spreading + absorption;
}

float path_loss_linear(float range_m, float freq_khz,
                       float spreading_factor, bool saltwater) {
    const float tl_db = transmission_loss_db(range_m, freq_khz,
                                             spreading_factor, saltwater);
    return std::pow(10.0f, -tl_db / 20.0f);
}

} // namespace openCREST::dsp
