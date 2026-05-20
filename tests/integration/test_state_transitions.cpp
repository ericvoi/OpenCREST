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
                          SPSCRingBuffer<uint16_t>* rx) {
    PerModemContext c;
    c.id          = id;
    c.calibration = silent_cal();
    c.runtime     = rt;
    c.tx_ring     = tx;
    c.rx_ring     = rx;
    c.noise_cfg   = silent_noise();
    c.sample_rate = 500'000;
    return c;
}

ScenarioConfig loopback_scenario() {
    ScenarioConfig sc;
    sc.name = "state_transition";
    sc.environment.sound_speed_m_s  = 1500.0f;
    sc.environment.spreading_factor = 2.0f;
    sc.environment.saltwater        = true;
    sc.environment.max_range_m      = 1.0f;

    ModemConfig mc; mc.id = "modem_a"; mc.usb_serial = "SN-A";
    sc.modems.push_back(mc);

    ChannelConfig cc;
    cc.from_modem       = "modem_a";
    cc.to_modem         = "modem_a";
    cc.range_m          = 1.0f;
    cc.spreading_factor = 2.0f;
    cc.saltwater        = true;
    cc.sound_speed_m_s  = 1500.0f;
    cc.multipath_taps.push_back({0.0f, 1.0f, 0.0f});
    sc.channels.push_back(cc);
    return sc;
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
// IDLE state: SourceWorker doesn't drain tx_ring or write to PairBuffer
// ===========================================================================

TEST(StateTransitions, IdleDoesNotDrainTxRing) {
    auto sc = loopback_scenario();
    SPSCRingBuffer<uint16_t> tx_ring(TX_RING_CAPACITY);
    SPSCRingBuffer<uint16_t> rx_ring(RX_RING_CAPACITY);
    ModemRuntimeState runtime;
    runtime.state.store(ModemState::IDLE, std::memory_order_relaxed);

    std::vector<PerModemContext> ctxs;
    ctxs.push_back(make_ctx("modem_a", &runtime, &tx_ring, &rx_ring));
    ChannelEngine engine(sc, std::move(ctxs));
    engine.start();

    // Pre-load tx_ring while IDLE; the worker shouldn't drain it.
    constexpr uint16_t MID = 1u << 15;
    std::vector<uint16_t> tx(128, MID);
    tx_ring.write(tx.data(), tx.size());

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    EXPECT_EQ(tx_ring.available_read(), 128u)
        << "SourceWorker drained tx_ring while modem was IDLE";

    engine.stop();
}

// ===========================================================================
// IDLE → TX → SETTLING: PairBuffer captures the message; receiver pull
// drains it into rx_ring.
// ===========================================================================

TEST(StateTransitions, FullCycleIdleTxSettlingProducesOutput) {
    auto sc = loopback_scenario();
    SPSCRingBuffer<uint16_t> tx_ring(TX_RING_CAPACITY);
    SPSCRingBuffer<uint16_t> rx_ring(RX_RING_CAPACITY);
    ModemRuntimeState runtime;
    runtime.state.store(ModemState::IDLE, std::memory_order_relaxed);

    std::vector<PerModemContext> ctxs;
    ctxs.push_back(make_ctx("modem_a", &runtime, &tx_ring, &rx_ring));
    ChannelEngine engine(sc, std::move(ctxs));
    engine.start();

    // Push samples, set TX
    constexpr uint16_t MID = 1u << 15;
    constexpr uint16_t DELTA = 4000;
    std::vector<uint16_t> tx(PROCESSING_BLOCK_SIZE * 2,
                              static_cast<uint16_t>(MID + DELTA));
    tx_ring.write(tx.data(), tx.size());
    runtime.state.store(ModemState::TX, std::memory_order_release);

    ASSERT_TRUE(wait_for([&] { return tx_ring.available_read() == 0; },
                          std::chrono::milliseconds(500)));

    runtime.state.store(ModemState::SETTLING, std::memory_order_release);

    // Receiver-side drain via ReceiverMix.
    auto* mix = engine.receiver_mix(0);
    ASSERT_NE(mix, nullptr);

    // Pull enough to skip the propagation-delay silence (~333 samples for
    // 1 m at 1500 m·s⁻¹) plus a body.
    constexpr size_t TARGET = 333 + PROCESSING_BLOCK_SIZE;
    const auto deadline = std::chrono::steady_clock::now()
                        + std::chrono::milliseconds(500);
    while (std::chrono::steady_clock::now() < deadline) {
        mix->pull(rx_ring, 256);
        if (rx_ring.available_read() >= TARGET) break;
        std::this_thread::sleep_for(std::chrono::microseconds(200));
    }

    engine.stop();

    EXPECT_GT(rx_ring.available_read(), 0u);

    std::vector<uint16_t> out(rx_ring.available_read());
    rx_ring.read(out.data(), out.size());
    bool any_signal = false;
    for (auto s : out) {
        if (std::abs(static_cast<int>(s) - static_cast<int>(MID)) > 100) {
            any_signal = true; break;
        }
    }
    EXPECT_TRUE(any_signal);
}

// ===========================================================================
// TX with empty tx_ring: worker tolerates underrun (no crash, no spin).
// ===========================================================================

TEST(StateTransitions, TxStateWithEmptyTxRingNoCrash) {
    auto sc = loopback_scenario();
    SPSCRingBuffer<uint16_t> tx_ring(TX_RING_CAPACITY);
    SPSCRingBuffer<uint16_t> rx_ring(RX_RING_CAPACITY);
    ModemRuntimeState runtime;
    runtime.state.store(ModemState::TX, std::memory_order_relaxed);

    std::vector<PerModemContext> ctxs;
    ctxs.push_back(make_ctx("modem_a", &runtime, &tx_ring, &rx_ring));
    ChannelEngine engine(sc, std::move(ctxs));
    engine.start();

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    runtime.state.store(ModemState::IDLE, std::memory_order_release);

    engine.stop();
    SUCCEED() << "Engine did not crash on TX-with-empty-ring";
}
