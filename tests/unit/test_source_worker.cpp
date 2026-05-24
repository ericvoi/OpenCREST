#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <thread>
#include <vector>

#include "channel/source_worker.hpp"
#include "channel/channel.hpp"
#include "channel/pair_buffer.hpp"
#include "config/scenario.hpp"
#include "core/ring_buffer.hpp"
#include "core/sample_conversion.hpp"
#include "core/types.hpp"

using openCREST::CalibrationData;
using openCREST::Channel;
using openCREST::ChannelConfig;
using openCREST::ModemRuntimeState;
using openCREST::ModemState;
using openCREST::PairBuffer;
using openCREST::SourceWorker;
using openCREST::SPSCRingBuffer;

namespace {

constexpr uint32_t FS = 500'000;

CalibrationData make_cal() {
    CalibrationData c;
    c.adc_bits = 16;
    c.dac_bits = 16;
    c.adc_sampling_rate = FS;
    c.dac_sampling_rate = FS;
    c.center_freq_hz    = 1.0f;        // negligible path loss at 1 m
    return c;
}

ChannelConfig make_direct_path_config() {
    ChannelConfig cfg;
    cfg.from_modem       = "A";
    cfg.to_modem         = "B";
    cfg.range_m          = 1.0f;       // small base_delay
    cfg.spreading_factor = 2.0f;
    cfg.saltwater        = true;
    cfg.sound_speed_m_s  = 1500.0f;
    cfg.multipath_taps.push_back({0.0f, 1.0f, 0.0f});
    return cfg;
}

// Wait until `predicate()` is true or `timeout` elapses. Returns true if the
// predicate succeeded.
template <typename Pred>
bool wait_for(Pred predicate, std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (predicate()) return true;
        std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
    return predicate();
}

constexpr size_t BIG_CAP = 16'384;

} // namespace

// ===========================================================================
// IDLE state: no work performed
// ===========================================================================

TEST(SourceWorker, IdleStateDoesNothing) {
    ModemRuntimeState rt;
    rt.state.store(ModemState::IDLE, std::memory_order_relaxed);
    SPSCRingBuffer<uint16_t> tx_ring(1024);

    auto cfg = make_direct_path_config();
    auto cal = make_cal();
    auto ch = std::make_unique<Channel>(cfg, cal, cal,
                                          openCREST::TransducerSpec{},
                                          openCREST::TransducerSpec{});
    PairBuffer pb(BIG_CAP, ch->base_delay_samples());

    std::vector<SourceWorker::Outgoing> outgoing;
    outgoing.push_back({std::move(ch), &pb});

    SourceWorker worker("A", rt, tx_ring, cal, std::move(outgoing));

    std::thread t([&] { worker.run(); });
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    EXPECT_EQ(worker.tx_samples_consumed(), 0u);
    EXPECT_EQ(pb.available_read(), 0u);

    worker.stop();
    t.join();
}

// ===========================================================================
// TX state: samples drained from tx_ring and scattered into PairBuffer
// ===========================================================================

TEST(SourceWorker, TxStateDrainsTxRingAndScatters) {
    ModemRuntimeState rt;
    rt.state.store(ModemState::IDLE, std::memory_order_relaxed);
    SPSCRingBuffer<uint16_t> tx_ring(4096);

    auto cfg = make_direct_path_config();
    auto cal = make_cal();
    auto ch = std::make_unique<Channel>(cfg, cal, cal,
                                          openCREST::TransducerSpec{},
                                          openCREST::TransducerSpec{});
    const size_t base_delay = ch->base_delay_samples();
    PairBuffer pb(BIG_CAP, base_delay);

    std::vector<SourceWorker::Outgoing> outgoing;
    outgoing.push_back({std::move(ch), &pb});

    SourceWorker worker("A", rt, tx_ring, cal, std::move(outgoing));
    std::thread t([&] { worker.run(); });

    // Pre-load tx_ring with 1024 samples of mid-amp DC, then signal TX.
    constexpr size_t N = 1024;
    constexpr uint16_t MID = 1u << 15;
    constexpr uint16_t DELTA = 8000;
    std::vector<uint16_t> samples(N, static_cast<uint16_t>(MID + DELTA));
    ASSERT_EQ(tx_ring.write(samples.data(), N), N);

    rt.state.store(ModemState::TX, std::memory_order_release);

    // Wait for the worker to consume them.
    ASSERT_TRUE(wait_for([&] { return tx_ring.available_read() == 0; },
                          std::chrono::milliseconds(500)));

    // Now end TX and let the worker call on_message_end.
    rt.state.store(ModemState::SETTLING, std::memory_order_release);

    ASSERT_TRUE(wait_for(
        [&] { return pb.available_read() >= base_delay + 8; },
        std::chrono::milliseconds(500)));

    worker.stop();
    t.join();

    EXPECT_GE(worker.tx_samples_consumed(), N);

    // Drain the PairBuffer; the propagation-delay region should be silence
    // and the body should track the input signal.
    std::vector<float> out(BIG_CAP, std::numeric_limits<float>::quiet_NaN());
    const size_t avail = pb.available_read();
    ASSERT_GT(avail, base_delay + 8);
    const size_t n = pb.read_advance(out.data(), avail);
    EXPECT_EQ(n, avail);

    for (size_t i = 0; i < base_delay; ++i) {
        EXPECT_FLOAT_EQ(out[i], 0.0f) << "i=" << i;
    }
    // Skip Farrow latency (~3 samples) then verify body is the expected DC.
    const float expected = openCREST::adc_to_float(MID + DELTA, 16);
    for (size_t i = base_delay + 8; i < base_delay + 200; ++i) {
        EXPECT_NEAR(out[i], expected, 1e-3f) << "i=" << i;
    }
}

// ===========================================================================
// TX → SETTLING edge: on_message_end exposes the multipath tail
// ===========================================================================

TEST(SourceWorker, TxExitExposesMultipathTail) {
    ModemRuntimeState rt;
    rt.state.store(ModemState::IDLE, std::memory_order_relaxed);
    SPSCRingBuffer<uint16_t> tx_ring(4096);

    // Channel with a single tap at delta = 50 samples.
    ChannelConfig cfg = make_direct_path_config();
    cfg.multipath_taps.clear();
    cfg.multipath_taps.push_back({50.0f / FS, 1.0f, 0.0f});

    auto cal = make_cal();
    auto ch = std::make_unique<Channel>(cfg, cal, cal,
                                          openCREST::TransducerSpec{},
                                          openCREST::TransducerSpec{});
    const size_t base_delay = ch->base_delay_samples();
    PairBuffer pb(BIG_CAP, base_delay);

    std::vector<SourceWorker::Outgoing> outgoing;
    outgoing.push_back({std::move(ch), &pb});

    SourceWorker worker("A", rt, tx_ring, cal, std::move(outgoing));
    std::thread t([&] { worker.run(); });

    // Push a single impulse at sample 0.
    constexpr uint16_t MID = 1u << 15;
    std::vector<uint16_t> samples(64, MID);
    samples[0] = MID + 16'000;
    ASSERT_EQ(tx_ring.write(samples.data(), 64), 64u);

    rt.state.store(ModemState::TX, std::memory_order_release);
    ASSERT_TRUE(wait_for([&] { return tx_ring.available_read() == 0; },
                          std::chrono::milliseconds(500)));

    rt.state.store(ModemState::SETTLING, std::memory_order_release);

    // Wait for the worker to flush; total exposure ≥ base + 50 + 3 (Farrow).
    ASSERT_TRUE(wait_for(
        [&] { return pb.available_read() >= base_delay + 64 + 50; },
        std::chrono::milliseconds(500)));

    worker.stop();
    t.join();

    std::vector<float> out(pb.available_read());
    pb.read_advance(out.data(), out.size());

    // The impulse arrives at base_delay + 50 + Farrow_latency(~3).
    const size_t expected = base_delay + 50 + 3;
    ASSERT_GT(out.size(), expected + 4);

    // Find the peak; allow ±2 sample tolerance.
    size_t best = 0;
    float best_mag = 0.0f;
    for (size_t i = 0; i < out.size(); ++i) {
        if (std::abs(out[i]) > best_mag) { best_mag = std::abs(out[i]); best = i; }
    }
    EXPECT_NEAR(static_cast<int>(best), static_cast<int>(expected), 3);
    EXPECT_GT(best_mag, 0.1f);
}

// ===========================================================================
// Two outgoing channels: scatter to both PairBuffers
// ===========================================================================

TEST(SourceWorker, ScattersToMultipleOutgoingChannels) {
    ModemRuntimeState rt;
    rt.state.store(ModemState::IDLE, std::memory_order_relaxed);
    SPSCRingBuffer<uint16_t> tx_ring(4096);

    auto cfg = make_direct_path_config();
    auto cal = make_cal();
    auto ch1 = std::make_unique<Channel>(cfg, cal, cal,
                                           openCREST::TransducerSpec{},
                                           openCREST::TransducerSpec{});
    auto ch2 = std::make_unique<Channel>(cfg, cal, cal,
                                           openCREST::TransducerSpec{},
                                           openCREST::TransducerSpec{});
    const size_t base = ch1->base_delay_samples();
    PairBuffer pb1(BIG_CAP, base);
    PairBuffer pb2(BIG_CAP, base);

    std::vector<SourceWorker::Outgoing> outgoing;
    outgoing.push_back({std::move(ch1), &pb1});
    outgoing.push_back({std::move(ch2), &pb2});

    SourceWorker worker("A", rt, tx_ring, cal, std::move(outgoing));
    std::thread t([&] { worker.run(); });

    constexpr uint16_t MID = 1u << 15;
    std::vector<uint16_t> samples(256, static_cast<uint16_t>(MID + 4000));
    ASSERT_EQ(tx_ring.write(samples.data(), 256), 256u);

    rt.state.store(ModemState::TX, std::memory_order_release);
    ASSERT_TRUE(wait_for([&] { return tx_ring.available_read() == 0; },
                          std::chrono::milliseconds(500)));

    rt.state.store(ModemState::SETTLING, std::memory_order_release);
    ASSERT_TRUE(wait_for(
        [&] { return pb1.available_read() > base && pb2.available_read() > base; },
        std::chrono::milliseconds(500)));

    worker.stop();
    t.join();

    EXPECT_GT(pb1.available_read(), base);
    EXPECT_GT(pb2.available_read(), base);
}

// ===========================================================================
// Multiple TX cycles: each begins a new message in the PairBuffer
// ===========================================================================

// SourceWorker passes gap=0 to PairBuffer; message 2 lands immediately
// after message 1's committed watermark, regardless of wall-clock idle
// between TX windows.
TEST(SourceWorker, ConsecutiveTxWindowsAppendInReceiverTime) {
    ModemRuntimeState rt;
    rt.state.store(ModemState::IDLE, std::memory_order_relaxed);
    SPSCRingBuffer<uint16_t> tx_ring(4096);

    auto cfg = make_direct_path_config();
    auto cal = make_cal();
    auto ch = std::make_unique<Channel>(cfg, cal, cal,
                                          openCREST::TransducerSpec{},
                                          openCREST::TransducerSpec{});
    const size_t base = ch->base_delay_samples();
    constexpr size_t LARGE_CAP = 256 * 1024;
    PairBuffer pb(LARGE_CAP, base);

    std::vector<SourceWorker::Outgoing> outgoing;
    outgoing.push_back({std::move(ch), &pb});

    SourceWorker worker("A", rt, tx_ring, cal, std::move(outgoing));
    std::thread t([&] { worker.run(); });

    constexpr uint16_t MID = 1u << 15;

    // ---- Message 1 ----
    {
        std::vector<uint16_t> samples(128, static_cast<uint16_t>(MID + 4000));
        ASSERT_EQ(tx_ring.write(samples.data(), 128), 128u);
        rt.state.store(ModemState::TX, std::memory_order_release);
        ASSERT_TRUE(wait_for([&] { return tx_ring.available_read() == 0; },
                              std::chrono::milliseconds(500)));
        rt.state.store(ModemState::IDLE, std::memory_order_release);
        ASSERT_TRUE(wait_for(
            [&] { return pb.available_read() >= base + 128; },
            std::chrono::milliseconds(500)));
    }

    const size_t available_after_msg1 = pb.available_read();

    // Substantial wall-clock idle to prove gap_samples is ignored.
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // ---- Message 2 ----
    {
        std::vector<uint16_t> samples(128, static_cast<uint16_t>(MID - 4000));
        ASSERT_EQ(tx_ring.write(samples.data(), 128), 128u);
        rt.state.store(ModemState::TX, std::memory_order_release);
        ASSERT_TRUE(wait_for([&] { return tx_ring.available_read() == 0; },
                              std::chrono::milliseconds(500)));
        rt.state.store(ModemState::IDLE, std::memory_order_release);
        ASSERT_TRUE(wait_for(
            [&] { return pb.available_read() >= available_after_msg1 + 128; },
            std::chrono::milliseconds(500)));
    }

    worker.stop();
    t.join();

    // The PairBuffer should contain (back-to-back, ignoring wall-clock idle):
    //   [0, base)         — propagation-delay zeros
    //   [base, base+128)  — message 1 body (positive value)
    //   [base+128, ...)   — message 2 body (negative value), modulo a small
    //                       Farrow / multipath-tail transition region.
    std::vector<float> out(pb.available_read());
    pb.read_advance(out.data(), out.size());

    const float pos_value = openCREST::adc_to_float(MID + 4000, 16);
    const float neg_value = openCREST::adc_to_float(MID - 4000, 16);

    size_t end_of_msg1 = 0;
    for (size_t i = base + 8; i < out.size(); ++i) {
        if (out[i] > pos_value * 0.5f) end_of_msg1 = i;
    }
    ASSERT_GT(end_of_msg1, base);

    size_t start_of_msg2 = 0;
    for (size_t i = end_of_msg1 + 1; i < out.size(); ++i) {
        if (out[i] < neg_value * 0.5f) { start_of_msg2 = i; break; }
    }
    ASSERT_GT(start_of_msg2, end_of_msg1);

    // With gap_samples forced to 0, message 2 abuts message 1's commit point.
    // Allow a small Farrow drain / multipath tail region between them
    // (kFarrowDrainZeros = 4 + a few samples of slop), but nothing close to
    // the 50 ms wall-clock idle.
    const size_t gap_observed = start_of_msg2 - end_of_msg1;
    EXPECT_LT(gap_observed, static_cast<size_t>(64))
        << "expected back-to-back placement, gap_observed=" << gap_observed
        << " (msg1 ends @ " << end_of_msg1
        << ", msg2 starts @ " << start_of_msg2 << ")";
}

TEST(SourceWorker, MultipleTxCyclesProduceSequentialMessages) {
    ModemRuntimeState rt;
    rt.state.store(ModemState::IDLE, std::memory_order_relaxed);
    SPSCRingBuffer<uint16_t> tx_ring(4096);

    auto cfg = make_direct_path_config();
    auto cal = make_cal();
    auto ch = std::make_unique<Channel>(cfg, cal, cal,
                                          openCREST::TransducerSpec{},
                                          openCREST::TransducerSpec{});
    const size_t base = ch->base_delay_samples();
    PairBuffer pb(BIG_CAP, base);

    std::vector<SourceWorker::Outgoing> outgoing;
    outgoing.push_back({std::move(ch), &pb});

    SourceWorker worker("A", rt, tx_ring, cal, std::move(outgoing));
    std::thread t([&] { worker.run(); });

    auto run_one_cycle = [&](uint16_t marker) {
        constexpr uint16_t MID = 1u << 15;
        std::vector<uint16_t> samples(128, static_cast<uint16_t>(MID + marker));
        ASSERT_EQ(tx_ring.write(samples.data(), 128), 128u);

        rt.state.store(ModemState::TX, std::memory_order_release);
        ASSERT_TRUE(wait_for([&] { return tx_ring.available_read() == 0; },
                              std::chrono::milliseconds(500)));
        rt.state.store(ModemState::IDLE, std::memory_order_release);
        // Give the worker a moment to call on_message_end.
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    };

    run_one_cycle(2000);
    run_one_cycle(6000);

    worker.stop();
    t.join();

    // Total consumed = 256 source samples across two messages.
    EXPECT_EQ(worker.tx_samples_consumed(), 256u);

    // PairBuffer should hold base + ~256 sample window covering both messages.
    EXPECT_GT(pb.available_read(), base + 200);
}
