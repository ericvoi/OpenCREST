#include <gtest/gtest.h>
#include <cmath>
#include <numeric>
#include <vector>

#include "channel/channel.hpp"
#include "channel/pair_buffer.hpp"
#include "config/scenario.hpp"
#include "core/types.hpp"
#include "dsp/path_loss.hpp"
#include "test_helpers/signal_generators.hpp"
#include "test_helpers/analysis.hpp"

using openCREST::Channel;
using openCREST::ChannelConfig;
using openCREST::CalibrationData;
using openCREST::MultipathTap;
using openCREST::PairBuffer;

namespace {

constexpr uint32_t FS = 500'000;

CalibrationData make_cal(uint32_t rate = FS, float center_freq_hz = 1.0f) {
    CalibrationData c;
    c.adc_sampling_rate = rate;
    c.dac_sampling_rate = rate;
    c.adc_bits = 16;
    c.dac_bits = 16;
    // Default to near-DC so path-loss is negligible in tests that don't
    // care about Thorp absorption. Tests that exercise path loss override.
    c.center_freq_hz    = center_freq_hz;
    return c;
}

ChannelConfig make_config(float delay_s, float gain,
                          float range_m = 1.0f) {
    ChannelConfig cfg;
    cfg.from_modem       = "A";
    cfg.to_modem         = "A";
    cfg.range_m          = range_m;
    cfg.spreading_factor = 2.0f;
    cfg.saltwater        = true;
    cfg.sound_speed_m_s  = 1500.0f;

    MultipathTap t;
    t.delay_s     = delay_s;
    t.gain_linear = gain;
    t.phase_rad   = 0.0f;
    cfg.multipath_taps.push_back(t);
    return cfg;
}

ChannelConfig make_multitap_config(std::vector<MultipathTap> taps,
                                   float range_m = 1.0f) {
    auto cfg = make_config(0.0f, 1.0f, range_m);
    cfg.multipath_taps = std::move(taps);
    return cfg;
}

// Drain everything currently visible in the PairBuffer.
std::vector<float> drain_all(PairBuffer& pb) {
    std::vector<float> out;
    std::vector<float> tmp(256);
    while (true) {
        const size_t n = pb.read_advance(tmp.data(), tmp.size());
        if (n == 0) break;
        out.insert(out.end(), tmp.begin(), tmp.begin() + n);
    }
    return out;
}

// Find the absolute-deviation peak position in a signal (for impulse-arrival
// time estimation). Returns the index of the max |x|.
size_t peak_index(const std::vector<float>& v) {
    size_t best = 0;
    float best_mag = std::abs(v[0]);
    for (size_t i = 1; i < v.size(); ++i) {
        if (std::abs(v[i]) > best_mag) { best = i; best_mag = std::abs(v[i]); }
    }
    return best;
}

constexpr size_t BIG_CAP = 16'384;

} // namespace

// ===========================================================================
// Construction-time properties
// ===========================================================================

TEST(Channel, BaseDelayMatchesRangeAndSoundSpeed) {
    auto cfg = make_config(0.0f, 1.0f, /*range_m=*/3.0f);
    cfg.sound_speed_m_s = 1500.0f;
    Channel ch(cfg, make_cal(), make_cal(), {}, {});
    // 3 m / 1500 m/s = 2 ms = 1000 samples at 500 kSPS
    EXPECT_EQ(ch.base_delay_samples(), 1000u);
}

TEST(Channel, MaxTapDeltaIsMaxOfTaps) {
    std::vector<MultipathTap> taps = {
        {0.0f / FS,  1.0f, 0.0f},
        {10.0f / FS, 1.0f, 0.0f},
        {25.0f / FS, 1.0f, 0.0f},
    };
    auto cfg = make_multitap_config(taps);
    Channel ch(cfg, make_cal(), make_cal(), {}, {});
    EXPECT_EQ(ch.max_tap_delta_samples(), 25u);
}

TEST(Channel, InputNeededForBatchIsPositive) {
    auto cfg = make_config(0.0f, 1.0f);
    Channel ch(cfg, make_cal(), make_cal(), {}, {});
    EXPECT_GT(ch.input_needed_for_batch(), 0u);
}

// ===========================================================================
// on_message_start: places source sample 0 at receiver-time base_delay
// ===========================================================================

TEST(Channel, FirstSampleArrivesAtBaseDelay) {
    // Single direct-path tap, near-DC center freq → negligible path loss.
    auto cfg = make_config(0.0f, 1.0f, /*range_m=*/3.0f);
    Channel ch(cfg, make_cal(), make_cal(), {}, {});

    PairBuffer pb(BIG_CAP, ch.base_delay_samples());
    ch.on_message_start(pb, 0);

    // Feed an impulse at source position 0
    auto impulse = openCREST::test::make_impulse(64, 0, 1.0f);
    ch.process(impulse.data(), 64, pb);
    ch.on_message_end(pb);

    auto out = drain_all(pb);

    // Receiver should see ch.base_delay_samples() of zeros, then a peak from
    // the impulse (delayed by the Farrow's history latency of ~3 samples for
    // cubic Hermite interpolation).
    const size_t base = ch.base_delay_samples();
    ASSERT_GT(out.size(), base + 8);

    // Pre-base-delay region must be silence
    for (size_t i = 0; i < base; ++i) {
        EXPECT_FLOAT_EQ(out[i], 0.0f) << "i=" << i;
    }

    // Peak should land at base + Farrow latency (within a few samples)
    const size_t expected = base + 3;
    EXPECT_NEAR(static_cast<int>(peak_index(out)),
                static_cast<int>(expected), 2);
}

// ===========================================================================
// Direct-path DC passthrough
// ===========================================================================

TEST(Channel, DirectPathDCPassthroughHasUnitAmplitude) {
    auto cfg = make_config(0.0f, 1.0f, /*range_m=*/1.0f);
    Channel ch(cfg, make_cal(), make_cal(), {}, {});

    PairBuffer pb(BIG_CAP, ch.base_delay_samples());
    ch.on_message_start(pb, 0);

    std::vector<float> dc(256, 0.5f);
    ch.process(dc.data(), 256, pb);
    ch.on_message_end(pb);

    auto out = drain_all(pb);
    const size_t base = ch.base_delay_samples();
    ASSERT_GT(out.size(), base + 64);

    // Path loss now uses src_cal.center_freq_hz (default 1 Hz from
    // make_cal()), so this is effectively a no-op multiplier at 1 m.
    const float pl = openCREST::dsp::path_loss_linear(
        cfg.range_m, /*center_freq_khz=*/0.001f,
        cfg.spreading_factor, cfg.saltwater);

    // Skip Farrow latency, then expect 0.5 * pl (≈ 0.5 since 1m near-DC)
    for (size_t i = base + 8; i < base + 64; ++i) {
        EXPECT_NEAR(out[i], 0.5f * pl, 1e-3f) << "i=" << i;
    }
}

// ===========================================================================
// Multipath: two taps produce two impulse arrivals
// ===========================================================================

TEST(Channel, TwoTapImpulseProducesTwoArrivals) {
    std::vector<MultipathTap> taps = {
        {0.0f       / FS, 1.0f, 0.0f},
        {20.0f      / FS, 0.5f, 0.0f},
    };
    auto cfg = make_multitap_config(taps);
    Channel ch(cfg, make_cal(), make_cal(), {}, {});

    PairBuffer pb(BIG_CAP, ch.base_delay_samples());
    ch.on_message_start(pb, 0);

    auto impulse = openCREST::test::make_impulse(128, 0, 1.0f);
    ch.process(impulse.data(), 128, pb);
    ch.on_message_end(pb);

    auto out = drain_all(pb);
    const size_t base = ch.base_delay_samples();

    // Direct path peak: at base + Farrow_latency (~3)
    // Reflection: at base + 20 + Farrow_latency (~3) with half amplitude
    const size_t direct_pos     = base + 3;
    const size_t reflection_pos = base + 23;

    ASSERT_GT(out.size(), reflection_pos + 4);

    EXPECT_NEAR(out[direct_pos],     1.0f, 1e-3f);
    EXPECT_NEAR(out[reflection_pos], 0.5f, 1e-3f);
}

// ===========================================================================
// Tone preserves frequency at ratio = 1
// ===========================================================================

TEST(Channel, TonePreservedAtUnityRatio) {
    auto cfg = make_config(0.0f, 1.0f, 1.0f);
    cfg.clock_offset_ppm = 0.0f;
    cfg.velocity_radial_m_s = 0.0f;
    Channel ch(cfg, make_cal(), make_cal(), {}, {});

    PairBuffer pb(BIG_CAP, ch.base_delay_samples());
    ch.on_message_start(pb, 0);

    constexpr float FREQ = 10'000.0f;
    constexpr size_t N = 4096;
    auto tone = openCREST::test::make_tone(FREQ, 0.5f,
                                           static_cast<float>(FS), N);

    // Process in batches that match input_needed_for_batch granularity.
    size_t pos = 0;
    while (pos < N) {
        const size_t take = std::min(N - pos, ch.input_needed_for_batch());
        const size_t consumed = ch.process(tone.data() + pos, take, pb);
        if (consumed == 0) break;
        pos += consumed;
    }
    ch.on_message_end(pb);

    auto out = drain_all(pb);
    const size_t base = ch.base_delay_samples();

    // Trim warmup and any trailing tail
    std::vector<float> body(out.begin() + base + 64, out.end() - 64);
    const float f_meas = openCREST::test::estimate_frequency_hz(body, static_cast<float>(FS));
    EXPECT_NEAR(f_meas, FREQ, FREQ * 0.02f);
}

// ===========================================================================
// Doppler shift: clock offset modifies frequency
// ===========================================================================

TEST(Channel, ClockOffsetShiftsFrequency) {
    auto cfg = make_config(0.0f, 1.0f, 1.0f);
    cfg.clock_offset_ppm = 10000.0f;  // ratio = 1.01 → freq up by 1%
    Channel ch(cfg, make_cal(), make_cal(), {}, {});

    PairBuffer pb(BIG_CAP, ch.base_delay_samples());
    ch.on_message_start(pb, 0);

    constexpr float FREQ = 10'000.0f;
    constexpr size_t N = 8192;
    auto tone = openCREST::test::make_tone(FREQ, 0.5f,
                                           static_cast<float>(FS), N);

    size_t pos = 0;
    while (pos < N) {
        const size_t take = std::min(N - pos, ch.input_needed_for_batch());
        const size_t consumed = ch.process(tone.data() + pos, take, pb);
        if (consumed == 0) break;
        pos += consumed;
    }
    ch.on_message_end(pb);

    auto out = drain_all(pb);
    const size_t base = ch.base_delay_samples();

    std::vector<float> body(out.begin() + base + 64, out.end() - 64);
    const float f_meas = openCREST::test::estimate_frequency_hz(body, static_cast<float>(FS));

    // Expected: ratio = 1.01 → output is time-compressed → frequency UP by 1%
    EXPECT_NEAR(f_meas, FREQ * 1.01f, FREQ * 0.01f);
}

// ===========================================================================
// Complex multipath taps (phase ≠ 0): Hilbert pair scatters real & imag
// ===========================================================================

TEST(Channel, ComplexTapShiftsToneByPhase) {
    // Compare two channels — phase=0 (real-only) vs phase=π/2 (complex). For
    // a cosine input, the real-only channel reproduces the cosine and the
    // π/2 channel produces -sin = Hilbert(cos). The two outputs are
    // orthogonal across many cycles, so their inner product is ≈0 while
    // each individually retains substantial RMS. This avoids any need to
    // reason about absolute phase references between input and output.
    constexpr float FREQ = 30'000.0f;
    constexpr size_t N   = 8192;

    std::vector<MultipathTap> taps_real    = {{0.0f, 1.0f, 0.0f}};
    std::vector<MultipathTap> taps_complex = {{0.0f, 1.0f,
                                                static_cast<float>(M_PI_2)}};

    auto cfg_real    = make_multitap_config(taps_real);
    auto cfg_complex = make_multitap_config(taps_complex);
    cfg_real   .range_m = 1.0f;
    cfg_complex.range_m = 1.0f;

    Channel ch_real   (cfg_real,    make_cal(), make_cal(), {}, {});
    Channel ch_complex(cfg_complex, make_cal(), make_cal(), {}, {});

    PairBuffer pb_real   (64 * 1024, ch_real   .base_delay_samples());
    PairBuffer pb_complex(64 * 1024, ch_complex.base_delay_samples());

    ch_real   .on_message_start(pb_real,    0);
    ch_complex.on_message_start(pb_complex, 0);

    std::vector<float> cos_in(N);
    for (size_t i = 0; i < N; ++i) {
        cos_in[i] = std::cos(2.0f * static_cast<float>(M_PI) *
                              FREQ * i / static_cast<float>(FS));
    }

    auto run_through = [&](Channel& ch, PairBuffer& pb) {
        size_t pos = 0;
        while (pos < N) {
            const size_t take = std::min(N - pos, ch.input_needed_for_batch());
            const size_t got  = ch.process(cos_in.data() + pos, take, pb);
            if (got == 0) break;
            pos += got;
        }
        ch.on_message_end(pb);
    };
    run_through(ch_real,    pb_real);
    run_through(ch_complex, pb_complex);

    auto out_real    = drain_all(pb_real);
    auto out_complex = drain_all(pb_complex);

    // Align the two outputs by their effective delays. Real-only channel:
    // base_delay + Farrow_latency (~3). Complex: + Hilbert_group_delay (15).
    const size_t base_real    = ch_real   .base_delay_samples();
    const size_t base_complex = ch_complex.base_delay_samples();

    constexpr size_t SKIP = 32;
    constexpr size_t SAMPLES_FOR_CORRELATION = 4096;

    // RMS of real output (skip warmup).
    double rms_real_sq = 0.0, rms_complex_sq = 0.0;
    double inner = 0.0;
    size_t n = 0;
    for (size_t k = SKIP; k < SKIP + SAMPLES_FOR_CORRELATION; ++k) {
        const size_t ir = base_real    + 3      + k;   // Farrow only
        const size_t ic = base_complex + 3 + 15 + k;   // Farrow + Hilbert
        if (ir >= out_real.size() || ic >= out_complex.size()) break;
        rms_real_sq    += static_cast<double>(out_real   [ir]) * out_real   [ir];
        rms_complex_sq += static_cast<double>(out_complex[ic]) * out_complex[ic];
        inner          += static_cast<double>(out_real   [ir]) * out_complex[ic];
        ++n;
    }
    ASSERT_GT(n, 1024u);

    const double rms_real    = std::sqrt(rms_real_sq    / n);
    const double rms_complex = std::sqrt(rms_complex_sq / n);
    const double normed_inner =
        inner / (n * std::max(rms_real * rms_complex, 1e-12));

    // Both channels preserve roughly the same energy (Hilbert filter has
    // ≈1.0 mid-band gain after Hamming windowing).
    EXPECT_GT(rms_real,    0.4);
    EXPECT_GT(rms_complex, 0.4);
    EXPECT_NEAR(rms_complex, rms_real, rms_real * 0.25);

    // Orthogonality: cos vs -sin → normalized inner product ≈ 0.
    // Tolerance allows for filter rolloff and finite-window leakage.
    EXPECT_LT(std::abs(normed_inner), 0.1)
        << "cos(real channel) and -sin(complex channel) should be "
           "≈orthogonal; got normed_inner=" << normed_inner;
}

TEST(Channel, ComplexAndRealTapsCoexist) {
    // Two taps: direct (real, gain 1.0, delta 0) + reflected (complex, gain
    // 0.5, delta 80, phase = π/4). Output should show both arrivals; the
    // real-tap path benefits from Hilbert's group-delay alignment so the
    // direct arrival also picks up the +15-sample shift.
    std::vector<MultipathTap> taps = {
        {0.0f       / FS, 1.0f, 0.0f},
        {80.0f      / FS, 0.5f, /*phase=*/static_cast<float>(M_PI_4)},
    };
    auto cfg = make_multitap_config(taps);
    Channel ch(cfg, make_cal(), make_cal(), {}, {});

    PairBuffer pb(BIG_CAP, ch.base_delay_samples());
    ch.on_message_start(pb, 0);

    auto impulse = openCREST::test::make_impulse(256, 0, 1.0f);
    ch.process(impulse.data(), 256, pb);
    ch.on_message_end(pb);

    auto out = drain_all(pb);
    const size_t base = ch.base_delay_samples();

    // Both impulse responses should be visible with Hilbert group-delay
    // shift (~15) on top of the existing Farrow latency (~3).
    // Direct path peak: base + Farrow_latency + Hilbert_group_delay (~18)
    // Reflection peak: base + 80 + 18
    const size_t direct_pos     = base + 3 + 15;
    const size_t reflection_pos = base + 80 + 3 + 15;
    ASSERT_GT(out.size(), reflection_pos + 32);

    // Find peaks within ±4 samples of expected positions; magnitudes should
    // match cos(0)=1 for direct (gain 1.0) and cos(π/4)·0.5 ≈ 0.354 for
    // the reflection's real component (the imag component contributes too,
    // but at an impulse the imag part is the Hilbert response of an
    // impulse — small contribution to the peak sample).
    size_t direct_peak = direct_pos;
    float  direct_mag  = std::abs(out[direct_pos]);
    for (long off = -4; off <= 4; ++off) {
        const size_t i = static_cast<size_t>(
            static_cast<long>(direct_pos) + off);
        if (i < out.size() && std::abs(out[i]) > direct_mag) {
            direct_mag  = std::abs(out[i]);
            direct_peak = i;
        }
    }
    EXPECT_GT(direct_mag, 0.5f);

    size_t refl_peak = reflection_pos;
    float  refl_mag  = std::abs(out[reflection_pos]);
    for (long off = -8; off <= 8; ++off) {
        const size_t i = static_cast<size_t>(
            static_cast<long>(reflection_pos) + off);
        if (i < out.size() && std::abs(out[i]) > refl_mag) {
            refl_mag  = std::abs(out[i]);
            refl_peak = i;
        }
    }
    EXPECT_GT(refl_mag, 0.1f);
    // The reflection should arrive AFTER the direct path.
    EXPECT_GT(refl_peak, direct_peak + 50);
}

TEST(Channel, RealOnlyChannelDoesNotShiftByHilbertDelay) {
    // Sanity: confirm the Hilbert path is not engaged when all taps have
    // phase = 0. Existing FirstSampleArrivesAtBaseDelay test relied on this
    // implicitly; this is the explicit pin.
    std::vector<MultipathTap> taps = {{0.0f, 1.0f, 0.0f}};
    auto cfg = make_multitap_config(taps);
    cfg.range_m = 3.0f;
    Channel ch(cfg, make_cal(), make_cal(), {}, {});

    PairBuffer pb(BIG_CAP, ch.base_delay_samples());
    ch.on_message_start(pb, 0);

    auto impulse = openCREST::test::make_impulse(64, 0, 1.0f);
    ch.process(impulse.data(), 64, pb);
    ch.on_message_end(pb);

    auto out = drain_all(pb);
    const size_t base = ch.base_delay_samples();
    const size_t expected = base + 3;     // Farrow latency only, no Hilbert
    EXPECT_NEAR(static_cast<int>(peak_index(out)),
                static_cast<int>(expected), 2);
}

// ===========================================================================
// Acceleration: time-varying Doppler ratio over message duration
// ===========================================================================

TEST(Channel, AccelerationLeavesInitialRatioUnchangedAtMessageStart) {
    auto cfg = make_config(0.0f, 1.0f, 1.0f);
    cfg.velocity_radial_m_s     = 1.0f;       // ratio_0 = 1 + 1/1500
    cfg.acceleration_radial_m_s2 = 100.0f;    // huge accel; should not affect t=0
    Channel ch(cfg, make_cal(), make_cal(), {}, {});

    PairBuffer pb(BIG_CAP, ch.base_delay_samples());
    ch.on_message_start(pb, 0);

    // Right after on_message_start, source_samples_processed_ == 0 → t=0.
    EXPECT_NEAR(ch.current_doppler_ratio(), 1.0 + 1.0 / 1500.0, 1e-9);
}

TEST(Channel, ConstantVelocityRatioStaysConstantThroughMessage) {
    auto cfg = make_config(0.0f, 1.0f, 1.0f);
    cfg.velocity_radial_m_s     = 3.0f;       // ratio = 1 + 3/1500 = 1.002
    cfg.acceleration_radial_m_s2 = 0.0f;
    Channel ch(cfg, make_cal(), make_cal(), {}, {});

    PairBuffer pb(BIG_CAP, ch.base_delay_samples());
    ch.on_message_start(pb, 0);

    const double initial = ch.current_doppler_ratio();

    // Push a few small batches; ratio should not budge.
    std::vector<float> in(256, 0.1f);
    for (int batch = 0; batch < 4; ++batch) {
        ch.process(in.data(), in.size(), pb);
        EXPECT_NEAR(ch.current_doppler_ratio(), initial, 1e-12);
    }
}

TEST(Channel, AccelerationAdvancesRatioMonotonically) {
    auto cfg = make_config(0.0f, 1.0f, 1.0f);
    cfg.velocity_radial_m_s     = 0.0f;       // start at ratio = 1
    cfg.acceleration_radial_m_s2 = 1500.0f;   // 1 m/s²/c after ~1 s; pick big
                                              // so per-batch delta is observable
    Channel ch(cfg, make_cal(), make_cal(), {}, {});

    PairBuffer pb(BIG_CAP, ch.base_delay_samples());
    ch.on_message_start(pb, 0);

    EXPECT_NEAR(ch.current_doppler_ratio(), 1.0, 1e-9);

    // After 1000 source samples (2 ms at 500 kSPS), v = 1500 × 0.002 = 3 m/s
    // → ratio ≈ 1 + 3/1500 = 1.002
    std::vector<float> in(1000, 0.1f);
    ch.process(in.data(), in.size(), pb);
    EXPECT_NEAR(ch.current_doppler_ratio(), 1.0 + 3.0 / 1500.0, 1e-4);

    // Another 1000 source samples → t = 4 ms → v = 6 m/s → ratio ≈ 1.004
    ch.process(in.data(), in.size(), pb);
    EXPECT_GT(ch.current_doppler_ratio(), 1.0 + 5.0 / 1500.0);
}

TEST(Channel, NewMessageResetsRatioToInitial) {
    auto cfg = make_config(0.0f, 1.0f, 1.0f);
    cfg.velocity_radial_m_s     = 0.0f;
    cfg.acceleration_radial_m_s2 = 1500.0f;
    Channel ch(cfg, make_cal(), make_cal(), {}, {});

    PairBuffer pb(BIG_CAP, ch.base_delay_samples());

    ch.on_message_start(pb, 0);
    std::vector<float> in(1000, 0.1f);
    ch.process(in.data(), in.size(), pb);
    const double after_first = ch.current_doppler_ratio();
    EXPECT_GT(after_first, 1.001);
    ch.on_message_end(pb);

    // New message: t resets to 0 → ratio back to initial.
    ch.on_message_start(pb, 0);
    EXPECT_NEAR(ch.current_doppler_ratio(), 1.0, 1e-9);
}

TEST(Channel, AccelerationShiftsToneFrequencyOverMessage) {
    // Compare frequency at the start vs end of a long message under
    // acceleration. Use a clean tone, no multipath, ratio=1 baseline.
    auto cfg = make_config(0.0f, 1.0f, 1.0f);
    cfg.velocity_radial_m_s     = 0.0f;
    cfg.acceleration_radial_m_s2 = 750.0f;   // by t=20 ms → v=15 m/s → ratio≈1.01
    Channel ch(cfg, make_cal(), make_cal(), {}, {});

    PairBuffer pb(65'536, ch.base_delay_samples());
    ch.on_message_start(pb, 0);

    constexpr float FREQ = 10'000.0f;
    constexpr size_t N   = 16'384;     // ~33 ms at 500 kSPS
    auto tone = openCREST::test::make_tone(FREQ, 0.5f,
                                            static_cast<float>(FS), N);

    size_t pos = 0;
    while (pos < N) {
        const size_t take = std::min(N - pos, ch.input_needed_for_batch());
        const size_t got  = ch.process(tone.data() + pos, take, pb);
        if (got == 0) break;
        pos += got;
    }
    ch.on_message_end(pb);

    auto out = drain_all(pb);
    const size_t base = ch.base_delay_samples();

    ASSERT_GT(out.size(), base + 4096 + 4096);

    // Compare frequency in the first quarter vs the last quarter of the
    // body. Under acceleration, the late portion should be measurably
    // higher in instantaneous frequency.
    const size_t body_start = base + 64;
    const size_t body_len   = out.size() - body_start - 64;
    const size_t qlen       = body_len / 4;

    std::vector<float> early(out.begin() + body_start,
                              out.begin() + body_start + qlen);
    std::vector<float> late (out.begin() + body_start + 3 * qlen,
                              out.begin() + body_start + 4 * qlen);

    const float f_early = openCREST::test::estimate_frequency_hz(
        early, static_cast<float>(FS));
    const float f_late  = openCREST::test::estimate_frequency_hz(
        late,  static_cast<float>(FS));

    EXPECT_GT(f_late, f_early)
        << "acceleration should produce monotonically rising instantaneous "
           "frequency; early=" << f_early << " late=" << f_late;

    // Sanity: shift over the whole message should be roughly comparable to
    // ratio change. Total source duration ~33 ms × 750 m/s² → Δv ≈ 24.6 m/s
    // → Δratio ≈ 0.0164. f_late - f_early should be in that ballpark.
    EXPECT_GT(f_late - f_early, FREQ * 0.005f)
        << "f_late=" << f_late << " f_early=" << f_early;
}

// ===========================================================================
// on_message_end: Farrow tail + multipath tail are exposed
// ===========================================================================

TEST(Channel, MessageEndExposesMultipathTail) {
    // Single tap at delta = 50 samples.
    std::vector<MultipathTap> taps = {{50.0f / FS, 1.0f, 0.0f}};
    auto cfg = make_multitap_config(taps);
    Channel ch(cfg, make_cal(), make_cal(), {}, {});

    PairBuffer pb(BIG_CAP, ch.base_delay_samples());
    ch.on_message_start(pb, 0);

    auto impulse = openCREST::test::make_impulse(64, 0, 1.0f);
    ch.process(impulse.data(), 64, pb);

    // Without on_message_end, the impulse at delta=50 wouldn't be visible
    // because commit_source_progress only exposed up to the last source pos
    // which is < 50 + Farrow_latency.
    const size_t avail_before = pb.available_read();

    ch.on_message_end(pb);

    const size_t avail_after = pb.available_read();
    EXPECT_GT(avail_after, avail_before);

    auto out = drain_all(pb);
    const size_t base = ch.base_delay_samples();
    const size_t expected_peak = base + 50 + 3;  // Farrow latency
    ASSERT_GT(out.size(), expected_peak + 4);
    EXPECT_NEAR(out[expected_peak], 1.0f, 1e-3f);
}

// ===========================================================================
// Multiple messages: subsequent messages stack in receiver time
// ===========================================================================

TEST(Channel, TwoBackToBackMessages) {
    auto cfg = make_config(0.0f, 1.0f, 1.0f);
    Channel ch(cfg, make_cal(), make_cal(), {}, {});

    PairBuffer pb(BIG_CAP, ch.base_delay_samples());

    // Message 1: 64 samples of value 0.3
    ch.on_message_start(pb, 0);
    std::vector<float> m1(64, 0.3f);
    ch.process(m1.data(), 64, pb);
    ch.on_message_end(pb);

    // Message 2: 64 samples of value -0.5
    ch.on_message_start(pb, 0);
    std::vector<float> m2(64, -0.5f);
    ch.process(m2.data(), 64, pb);
    ch.on_message_end(pb);

    auto out = drain_all(pb);
    const size_t base = ch.base_delay_samples();
    ASSERT_GT(out.size(), base + 64 + 64);

    // Pre-base zeros
    for (size_t i = 0; i < base; ++i) {
        EXPECT_FLOAT_EQ(out[i], 0.0f) << "i=" << i;
    }

    // Skip Farrow latency at start of msg1, check sustained 0.3
    for (size_t i = base + 8; i < base + 60; ++i) {
        EXPECT_NEAR(out[i], 0.3f, 0.01f) << "i=" << i;
    }

    // After message 1 fully drains (around base + 64), look for message 2.
    // Find the first sample with negative magnitude > 0.4 — that marks the
    // start of message 2's body.
    size_t m2_start = 0;
    for (size_t i = base + 64; i < out.size(); ++i) {
        if (out[i] < -0.4f) { m2_start = i; break; }
    }
    ASSERT_GT(m2_start, base + 64);
    for (size_t i = m2_start + 4; i < m2_start + 50; ++i) {
        EXPECT_NEAR(out[i], -0.5f, 0.02f) << "i=" << i;
    }
}

// ===========================================================================
// Path loss attenuates output amplitude
// ===========================================================================

TEST(Channel, PathLossAttenuatesAtRange) {
    // At 25 kHz center freq + saltwater, Thorp absorption is ~3.6 dB/km.
    // Path-loss frequency now comes from src_cal.center_freq_hz.
    auto cfg_short = make_config(0.0f, 1.0f, /*range_m=*/1.0f);
    auto cfg_long  = make_config(0.0f, 1.0f, /*range_m=*/100.0f);

    const auto cal_25k = make_cal(FS, /*center_freq_hz=*/25'000.0f);
    Channel ch_short(cfg_short, cal_25k, cal_25k, {}, {});
    Channel ch_long (cfg_long,  cal_25k, cal_25k, {}, {});

    PairBuffer pb_short(BIG_CAP, ch_short.base_delay_samples());
    // 100 m range at 500 kSPS / 1500 m·s⁻¹ ≈ 33 k samples of propagation delay.
    PairBuffer pb_long (65'536, ch_long.base_delay_samples());

    ch_short.on_message_start(pb_short, 0);
    ch_long .on_message_start(pb_long,  0);

    std::vector<float> dc(256, 1.0f);
    ch_short.process(dc.data(), 256, pb_short);
    ch_long .process(dc.data(), 256, pb_long);
    ch_short.on_message_end(pb_short);
    ch_long .on_message_end(pb_long);

    auto out_short = drain_all(pb_short);
    auto out_long  = drain_all(pb_long);

    const size_t pos_short = ch_short.base_delay_samples() + 32;
    const size_t pos_long  = ch_long .base_delay_samples() + 32;

    EXPECT_GT(std::abs(out_short[pos_short]), std::abs(out_long[pos_long]) * 1.1f)
        << "short range should be louder than long range";
}

// ===========================================================================
// Channel-level gain_db is folded into every tap (cancels path loss in
// physical loopback)
// ===========================================================================

TEST(Channel, ChannelGainDbCancelsPathLoss) {
    // 100 m, 25 kHz, saltwater, spherical → ~40 dB path loss. Set gain_db to
    // exactly the transmission loss so output amplitude returns to unity.
    const float range_m = 100.0f;
    const float fc_khz  = 25.0f;
    const float tl_db   = openCREST::dsp::transmission_loss_db(
        range_m, fc_khz, /*spreading=*/2.0f, /*saltwater=*/true);

    auto cfg = make_config(0.0f, 1.0f, range_m);
    cfg.gain_db = tl_db;  // cancel exactly
    const auto cal = make_cal(FS, /*center_freq_hz=*/fc_khz * 1000.0f);
    Channel ch(cfg, cal, cal, {}, {});

    PairBuffer pb(65'536, ch.base_delay_samples());
    ch.on_message_start(pb, 0);

    std::vector<float> dc(256, 0.5f);
    ch.process(dc.data(), 256, pb);
    ch.on_message_end(pb);

    auto out = drain_all(pb);
    const size_t base = ch.base_delay_samples();
    ASSERT_GT(out.size(), base + 64);

    // After Farrow latency, expect 0.5 (gain_db × path_loss = 1.0)
    for (size_t i = base + 8; i < base + 64; ++i) {
        EXPECT_NEAR(out[i], 0.5f, 5e-3f) << "i=" << i;
    }
}

TEST(Channel, MultipathTapsAreRelativeToMainTapAfterGainDb) {
    // Two taps: direct (0 dB) and reflected (-6.02 dB → 0.5×). With gain_db
    // canceling path loss, the direct path should be ~unity and the reflected
    // path ~0.5 — multipath stays visible regardless of range.
    const float range_m = 100.0f;
    const float fc_khz  = 25.0f;
    const float tl_db   = openCREST::dsp::transmission_loss_db(
        range_m, fc_khz, 2.0f, true);

    std::vector<MultipathTap> taps = {
        {0.0f       / FS, 1.0f, 0.0f},
        {30.0f      / FS, 0.5f, 0.0f},
    };
    auto cfg = make_multitap_config(taps, range_m);
    cfg.gain_db = tl_db;
    const auto cal = make_cal(FS, /*center_freq_hz=*/fc_khz * 1000.0f);
    Channel ch(cfg, cal, cal, {}, {});

    PairBuffer pb(65'536, ch.base_delay_samples());
    ch.on_message_start(pb, 0);

    auto impulse = openCREST::test::make_impulse(128, 0, 1.0f);
    ch.process(impulse.data(), 128, pb);
    ch.on_message_end(pb);

    auto out = drain_all(pb);
    const size_t base = ch.base_delay_samples();
    const size_t direct_pos     = base + 3;
    const size_t reflection_pos = base + 33;

    ASSERT_GT(out.size(), reflection_pos + 4);
    EXPECT_NEAR(out[direct_pos],     1.0f, 5e-3f);
    EXPECT_NEAR(out[reflection_pos], 0.5f, 5e-3f);
}

// ===========================================================================
// propagation_delay_s overrides range-derived base delay (refraction)
// ===========================================================================

TEST(Channel, PropagationDelayOverrideUsedInsteadOfRange) {
    // Range would give 1 m / 1500 m/s ≈ 333 samples; override to 50 ms.
    auto cfg = make_config(0.0f, 1.0f, /*range_m=*/1.0f);
    cfg.propagation_delay_s = 0.050f;  // 25'000 samples at 500 kSPS
    Channel ch(cfg, make_cal(), make_cal(), {}, {});

    EXPECT_EQ(ch.base_delay_samples(), 25'000u);
}

TEST(Channel, NegativePropagationDelayFallsBackToRange) {
    auto cfg = make_config(0.0f, 1.0f, /*range_m=*/3.0f);
    cfg.propagation_delay_s = -1.0f;  // sentinel
    cfg.sound_speed_m_s = 1500.0f;
    Channel ch(cfg, make_cal(), make_cal(), {}, {});

    // 3 m / 1500 m/s = 2 ms = 1000 samples
    EXPECT_EQ(ch.base_delay_samples(), 1000u);
}

// ===========================================================================
// process() can be called repeatedly with partial input; conservation holds
// ===========================================================================

TEST(Channel, RepeatedProcessConservesInput) {
    // At ratio=1, total input consumed should equal total provided.
    auto cfg = make_config(0.0f, 1.0f, 1.0f);
    Channel ch(cfg, make_cal(), make_cal(), {}, {});

    PairBuffer pb(BIG_CAP, ch.base_delay_samples());
    ch.on_message_start(pb, 0);

    constexpr size_t TOTAL = 2048;
    std::vector<float> in(TOTAL);
    for (size_t i = 0; i < TOTAL; ++i) {
        in[i] = std::sin(2.0f * static_cast<float>(M_PI) * 8'000.0f * i / FS) * 0.3f;
    }

    size_t consumed = 0;
    while (consumed < TOTAL) {
        size_t take = std::min(TOTAL - consumed, size_t{300});
        const size_t got = ch.process(in.data() + consumed, take, pb);
        if (got == 0) break;
        consumed += got;
    }

    EXPECT_EQ(consumed, TOTAL);
}
