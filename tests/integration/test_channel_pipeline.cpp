#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <thread>
#include <vector>

#include "channel/channel_engine.hpp"
#include "channel/receiver_mix.hpp"
#include "config/scenario.hpp"
#include "core/constants.hpp"
#include "core/ring_buffer.hpp"
#include "core/sample_conversion.hpp"
#include "dsp/noise_generator.hpp"

using namespace openCREST;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

namespace {

CalibrationData silent_cal() {
    CalibrationData cal;
    cal.adc_bits          = 16;
    cal.dac_bits          = 16;
    cal.adc_sampling_rate = 500'000;
    cal.dac_sampling_rate = 500'000;
    cal.loopback_gain     = 1.0f;
    return cal;
}

dsp::NoiseConfig silent_noise() {
    dsp::NoiseConfig cfg;
    cfg.wenz_sea_state        = 0;
    cfg.target_level_db_re_fs = -200.0f;
    cfg.saltwater             = true;
    return cfg;
}

PerModemContext make_ctx(const std::string& id,
                          ModemRuntimeState* rt,
                          SPSCRingBuffer<uint16_t>* tx,
                          SPSCRingBuffer<uint16_t>* rx,
                          uint32_t fs = 500'000u) {
    PerModemContext c;
    c.id          = id;
    c.calibration = silent_cal();
    c.runtime     = rt;
    c.tx_ring     = tx;
    c.rx_ring     = rx;
    c.noise_cfg   = silent_noise();
    c.sample_rate = fs;
    return c;
}

ScenarioConfig loopback_scenario(float range_m = 1.0f, float tap_delay_s = 0.0f) {
    ScenarioConfig sc;
    sc.name = "loopback_pipeline";
    sc.environment.sound_speed_m_s  = 1500.0f;
    sc.environment.spreading_factor = 2.0f;
    sc.environment.saltwater        = true;
    sc.environment.max_range_m      = std::max(range_m, 1.0f);

    ModemConfig mc;
    mc.id         = "modem_a";
    mc.usb_serial = "SN-A";
    sc.modems.push_back(mc);

    ChannelConfig cc;
    cc.from_modem       = "modem_a";
    cc.to_modem         = "modem_a";
    cc.range_m          = range_m;
    cc.spreading_factor = 2.0f;
    cc.saltwater        = true;
    cc.sound_speed_m_s  = 1500.0f;
    cc.multipath_taps.push_back({tap_delay_s, 1.0f, 0.0f});
    sc.channels.push_back(cc);
    return sc;
}

// Repeatedly invoke ReceiverMix::pull until either the predicate fires or
// the deadline expires. Used to drain a PairBuffer at receiver pace.
template <typename Pred>
bool drain_until(ReceiverMix& mix, SPSCRingBuffer<uint16_t>& rx_ring,
                 size_t per_pull, Pred predicate,
                 std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        mix.pull(rx_ring, per_pull);
        if (predicate()) return true;
        std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
    return predicate();
}

template <typename Pred>
bool wait_for(Pred pred, std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (pred()) return true;
        std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
    return pred();
}

} // namespace

// ===========================================================================
// Direct-path loopback: TX waveform appears in rx_ring with non-trivial
// signal energy after engine workers process it.
// ===========================================================================

TEST(ChannelPipeline, DirectPathLoopbackPreservesEnergy) {
    auto sc = loopback_scenario(/*range_m=*/1.0f, /*tap_delay_s=*/0.0f);

    SPSCRingBuffer<uint16_t> tx_ring(TX_RING_CAPACITY);
    SPSCRingBuffer<uint16_t> rx_ring(RX_RING_CAPACITY);
    ModemRuntimeState runtime;
    runtime.state.store(ModemState::IDLE, std::memory_order_relaxed);

    std::vector<PerModemContext> ctxs;
    ctxs.push_back(make_ctx("modem_a", &runtime, &tx_ring, &rx_ring));

    ChannelEngine engine(sc, std::move(ctxs));
    engine.start();

    // Pre-load TX samples.
    constexpr uint16_t MID   = 1u << 15;
    constexpr uint16_t DELTA = 6000;
    constexpr size_t   N     = PROCESSING_BLOCK_SIZE * 4;
    std::vector<uint16_t> tx(N, static_cast<uint16_t>(MID + DELTA));
    ASSERT_EQ(tx_ring.write(tx.data(), N), N);

    runtime.state.store(ModemState::TX, std::memory_order_release);
    ASSERT_TRUE(wait_for([&] { return tx_ring.available_read() == 0; },
                          std::chrono::milliseconds(500)));

    runtime.state.store(ModemState::SETTLING, std::memory_order_release);

    // Drive the receiver side: pull samples into rx_ring at packet cadence.
    auto* mix = engine.receiver_mix(0);
    ASSERT_NE(mix, nullptr);
    // Drain enough samples that the propagation-delay silence region is
    // followed by a substantial signal body in rx_ring.
    const size_t TARGET = 333 + N;
    ASSERT_TRUE(drain_until(*mix, rx_ring, /*per_pull=*/256,
                             [&] { return rx_ring.available_read() >= TARGET; },
                             std::chrono::milliseconds(500)));

    engine.stop();

    // Expect non-trivial deviation from midpoint in the post-base region.
    std::vector<uint16_t> out(rx_ring.available_read());
    rx_ring.read(out.data(), out.size());

    double body_sum_dev = 0.0;
    size_t body_count   = 0;
    for (size_t i = 333 + 8; i < out.size(); ++i) {
        body_sum_dev += std::abs(static_cast<int>(out[i]) - static_cast<int>(MID));
        ++body_count;
    }
    ASSERT_GT(body_count, 0u);
    EXPECT_GT(body_sum_dev / body_count, 100.0);
}

// ===========================================================================
// Delayed path: impulse arrives no earlier than the configured tap delay.
// ===========================================================================

TEST(ChannelPipeline, DelayedTapImpulseArrivesAfterDelay) {
    constexpr float DELAY_S = 10.0f / 500'000.0f;  // 10 samples
    auto sc = loopback_scenario(/*range_m=*/1.0f, DELAY_S);

    SPSCRingBuffer<uint16_t> tx_ring(TX_RING_CAPACITY);
    SPSCRingBuffer<uint16_t> rx_ring(RX_RING_CAPACITY);
    ModemRuntimeState runtime;
    runtime.state.store(ModemState::IDLE, std::memory_order_relaxed);

    std::vector<PerModemContext> ctxs;
    ctxs.push_back(make_ctx("modem_a", &runtime, &tx_ring, &rx_ring));
    ChannelEngine engine(sc, std::move(ctxs));
    engine.start();

    constexpr uint16_t MID = 1u << 15;
    std::vector<uint16_t> tx(PROCESSING_BLOCK_SIZE, MID);
    tx[0] = MID + 16'000;  // impulse
    ASSERT_EQ(tx_ring.write(tx.data(), tx.size()), tx.size());

    runtime.state.store(ModemState::TX, std::memory_order_release);
    ASSERT_TRUE(wait_for([&] { return tx_ring.available_read() == 0; },
                          std::chrono::milliseconds(500)));
    runtime.state.store(ModemState::SETTLING, std::memory_order_release);

    auto* mix = engine.receiver_mix(0);
    ASSERT_NE(mix, nullptr);
    // Pull through the propagation-delay region (~333 samples) plus the
    // tap delta plus comfortable Farrow-latency margin.
    constexpr size_t TARGET = 500;
    drain_until(*mix, rx_ring, 256,
                 [&] { return rx_ring.available_read() >= TARGET; },
                 std::chrono::milliseconds(500));

    engine.stop();

    std::vector<uint16_t> out(rx_ring.available_read());
    rx_ring.read(out.data(), out.size());

    // Find the deviation peak.
    size_t peak_idx = 0;
    int    peak_dev = 0;
    for (size_t i = 0; i < out.size(); ++i) {
        const int d = std::abs(static_cast<int>(out[i]) - static_cast<int>(MID));
        if (d > peak_dev) { peak_dev = d; peak_idx = i; }
    }

    // Channel base_delay = round(1m × 500kHz / 1500 m/s) = 333 samples.
    constexpr size_t BASE_DELAY = 333;
    constexpr size_t TAP_DELTA  = 10;
    EXPECT_GT(peak_dev, 100);
    EXPECT_GE(peak_idx, BASE_DELAY + TAP_DELTA);
}

// ===========================================================================
// Geometric-mode pipeline smoke test: same fixture as the static loopback
// but with `mode: geometric`, direct-only, v=0. The end-to-end pipeline
// must still deliver non-trivial signal energy to rx_ring — proves the
// SourceWorker / Channel / PairBuffer wiring carries the geometric branch.
// ===========================================================================

TEST(ChannelPipeline, GeometricDirectOnlyLoopbackPreservesEnergy) {
    ScenarioConfig sc;
    sc.name = "geometric_loopback_pipeline";
    sc.environment.sound_speed_m_s  = 1500.0f;
    sc.environment.spreading_factor = 2.0f;
    sc.environment.saltwater        = true;
    sc.environment.max_range_m      = 10.0f;

    ModemConfig mc; mc.id = "modem_a"; mc.usb_serial = "SN-A";
    sc.modems.push_back(mc);

    ChannelConfig cc;
    cc.from_modem       = "modem_a";
    cc.to_modem         = "modem_a";
    cc.range_m          = 1.0f;
    cc.spreading_factor = 2.0f;
    cc.saltwater        = true;
    cc.sound_speed_m_s  = 1500.0f;
    cc.mode             = ChannelMode::Geometric;
    cc.geometry.water_depth_m         = 10.0f;
    cc.geometry.source_depth_m        =  5.0f;
    cc.geometry.receiver_depth_m      =  5.0f;
    cc.geometry.gamma_surface         = -0.9f;
    cc.geometry.gamma_bottom          =  0.5f;
    cc.geometry.spreading_exponent_k  =  1.5f;
    cc.geometry.enable_direct         = true;
    cc.geometry.enable_surface        = false;
    cc.geometry.enable_bottom         = false;
    cc.geometry.r_min_m               = 0.5f;
    cc.geometry.r_max_m               = 2.0f;
    cc.initial_range_m                = 1.0f;
    sc.channels.push_back(cc);

    SPSCRingBuffer<uint16_t> tx_ring(TX_RING_CAPACITY);
    SPSCRingBuffer<uint16_t> rx_ring(RX_RING_CAPACITY);
    ModemRuntimeState runtime;
    runtime.state.store(ModemState::IDLE, std::memory_order_relaxed);

    std::vector<PerModemContext> ctxs;
    ctxs.push_back(make_ctx("modem_a", &runtime, &tx_ring, &rx_ring));

    ChannelEngine engine(sc, std::move(ctxs));
    engine.start();

    constexpr uint16_t MID   = 1u << 15;
    constexpr uint16_t DELTA = 6000;
    constexpr size_t   N     = PROCESSING_BLOCK_SIZE * 4;
    std::vector<uint16_t> tx(N, static_cast<uint16_t>(MID + DELTA));
    ASSERT_EQ(tx_ring.write(tx.data(), N), N);

    runtime.state.store(ModemState::TX, std::memory_order_release);
    ASSERT_TRUE(wait_for([&] { return tx_ring.available_read() == 0; },
                          std::chrono::milliseconds(500)));
    runtime.state.store(ModemState::SETTLING, std::memory_order_release);

    auto* mix = engine.receiver_mix(0);
    ASSERT_NE(mix, nullptr);
    // base_delay at r_min=0.5m: round(0.5/1500*500000) = 167 samples.
    const size_t TARGET = 167 + N;
    drain_until(*mix, rx_ring, /*per_pull=*/256,
                 [&] { return rx_ring.available_read() >= TARGET; },
                 std::chrono::milliseconds(500));

    engine.stop();

    std::vector<uint16_t> out(rx_ring.available_read());
    rx_ring.read(out.data(), out.size());

    double body_sum_dev = 0.0;
    size_t body_count   = 0;
    for (size_t i = 167 + 16; i < out.size(); ++i) {
        body_sum_dev += std::abs(static_cast<int>(out[i]) - static_cast<int>(MID));
        ++body_count;
    }
    ASSERT_GT(body_count, 0u);
    EXPECT_GT(body_sum_dev / body_count, 50.0);
}

// ===========================================================================
// Multi-channel mix: two source modems both transmit to a single receiver;
// receiver sees both contributions summed.
// ===========================================================================

TEST(ChannelPipeline, TwoSourcesMixIntoSingleReceiver) {
    ScenarioConfig sc;
    sc.name = "two_source_mix";
    sc.environment.sound_speed_m_s  = 1500.0f;
    sc.environment.spreading_factor = 2.0f;
    sc.environment.saltwater        = true;
    sc.environment.max_range_m      = 1.0f;

    for (const auto& id : {"modem_a", "modem_b", "modem_c"}) {
        ModemConfig mc; mc.id = id; mc.usb_serial = id;
        sc.modems.push_back(mc);
    }
    for (const auto& src : {"modem_a", "modem_b"}) {
        ChannelConfig cc;
        cc.from_modem       = src;
        cc.to_modem         = "modem_c";
        cc.range_m          = 1.0f;
        cc.spreading_factor = 2.0f;
        cc.saltwater        = true;
        cc.sound_speed_m_s  = 1500.0f;
        cc.multipath_taps.push_back({0.0f, 0.5f, 0.0f});
        sc.channels.push_back(cc);
    }

    SPSCRingBuffer<uint16_t> tx_a(TX_RING_CAPACITY), rx_a(RX_RING_CAPACITY);
    SPSCRingBuffer<uint16_t> tx_b(TX_RING_CAPACITY), rx_b(RX_RING_CAPACITY);
    SPSCRingBuffer<uint16_t> tx_c(TX_RING_CAPACITY), rx_c(RX_RING_CAPACITY);
    ModemRuntimeState rt_a, rt_b, rt_c;
    rt_a.state.store(ModemState::IDLE, std::memory_order_relaxed);
    rt_b.state.store(ModemState::IDLE, std::memory_order_relaxed);
    rt_c.state.store(ModemState::RX,   std::memory_order_relaxed);

    std::vector<PerModemContext> ctxs;
    ctxs.push_back(make_ctx("modem_a", &rt_a, &tx_a, &rx_a));
    ctxs.push_back(make_ctx("modem_b", &rt_b, &tx_b, &rx_b));
    ctxs.push_back(make_ctx("modem_c", &rt_c, &tx_c, &rx_c));

    ChannelEngine engine(sc, std::move(ctxs));
    engine.start();

    constexpr uint16_t MID = 1u << 15;
    constexpr uint16_t DELTA = 5000;
    constexpr size_t N = PROCESSING_BLOCK_SIZE * 2;
    std::vector<uint16_t> in(N, static_cast<uint16_t>(MID + DELTA));
    ASSERT_EQ(tx_a.write(in.data(), N), N);
    ASSERT_EQ(tx_b.write(in.data(), N), N);

    rt_a.state.store(ModemState::TX, std::memory_order_release);
    rt_b.state.store(ModemState::TX, std::memory_order_release);

    ASSERT_TRUE(wait_for(
        [&] { return tx_a.available_read() == 0 && tx_b.available_read() == 0; },
        std::chrono::milliseconds(500)));

    rt_a.state.store(ModemState::SETTLING, std::memory_order_release);
    rt_b.state.store(ModemState::SETTLING, std::memory_order_release);

    auto* mix = engine.receiver_mix(2);
    ASSERT_NE(mix, nullptr);
    drain_until(*mix, rx_c, 256,
                 [&] { return rx_c.available_read() >= N; },
                 std::chrono::milliseconds(500));

    engine.stop();

    std::vector<uint16_t> out(rx_c.available_read());
    rx_c.read(out.data(), out.size());

    // Mixed signal: each source contributes 0.5×, sum ≈ full DELTA.
    const float input_amp_float =
        adc_to_float(static_cast<uint16_t>(MID + DELTA), 16);

    // Find body region (post propagation delay + Farrow latency).
    constexpr size_t BASE_DELAY = 333;
    bool any_strong = false;
    for (size_t i = BASE_DELAY + 16; i < out.size(); ++i) {
        const float v = adc_to_float(out[i], 16);
        if (std::abs(v) > std::abs(input_amp_float) * 0.6f) {
            any_strong = true;
            break;
        }
    }
    EXPECT_TRUE(any_strong) << "Receiver did not see summed contribution";
}
