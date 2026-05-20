#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include "channel/channel_engine.hpp"
#include "config/scenario.hpp"
#include "core/constants.hpp"
#include "core/ring_buffer.hpp"
#include "io/modem_io.hpp"
#include "modem/modem.hpp"
#include "transport/mock_transport.hpp"

using namespace openCREST;

// Wires up a single-modem loopback through the new Phase 2 pipeline:
//   MockTransport ↔ ModemIO ↔ tx_ring/rx_ring ↔ ChannelEngine workers
class LoopbackMockFixture {
public:
    LoopbackMockFixture() {
        cal_.adc_bits = 16;
        cal_.dac_bits = 16;
        cal_.adc_sampling_rate = 500'000;
        cal_.dac_sampling_rate = 500'000;
        cal_.loopback_gain = 1.0f;

        mock_raw_ = new MockTransport();
        mock_raw_->set_calibration(cal_);
        mock_raw_->set_settling_time_ms(2);

        modem_ = std::make_unique<Modem>(
            std::unique_ptr<IModemTransport>(mock_raw_), "SN-TEST");
        EXPECT_TRUE(modem_->connect());
        modem_->calibrate();

        // Scenario
        scenario_.name = "loopback_mock_pipeline";
        scenario_.environment.sound_speed_m_s = 1500.0f;
        scenario_.environment.saltwater       = true;
        scenario_.environment.spreading_factor = 2.0f;
        scenario_.environment.max_range_m      = 1.0f;

        ModemConfig mc;
        mc.id = "modem_a"; mc.usb_serial = "SN-TEST";
        scenario_.modems.push_back(mc);

        ChannelConfig cc;
        cc.from_modem = "modem_a"; cc.to_modem = "modem_a";
        cc.range_m = 1.0f;
        cc.spreading_factor = 2.0f;
        cc.saltwater        = true;
        cc.sound_speed_m_s  = 1500.0f;
        cc.multipath_taps.push_back({0.0f, 1.0f, 0.0f});
        scenario_.channels.push_back(cc);

        scenario_.noise.wenz_sea_state = 0;
        scenario_.noise.disable        = true;

        // Per-modem context
        PerModemContext ctx;
        ctx.id          = "modem_a";
        ctx.calibration = cal_;
        ctx.runtime     = &modem_->runtime_state();
        ctx.tx_ring     = &tx_ring_;
        ctx.rx_ring     = &rx_ring_;
        dsp::NoiseConfig nc;
        nc.wenz_sea_state = 0;
        nc.target_level_db_re_fs = -100.0f;
        nc.saltwater = true;
        ctx.noise_cfg   = nc;
        ctx.sample_rate = 500'000;

        std::vector<PerModemContext> ctxs;
        ctxs.push_back(std::move(ctx));
        engine_ = std::make_unique<ChannelEngine>(scenario_, std::move(ctxs));

        // ModemIO with the receiver mix from the engine.
        io_ = std::make_unique<ModemIO>(*modem_, tx_ring_, rx_ring_,
                                         engine_->receiver_mix(0),
                                         /*logger=*/nullptr,
                                         /*metrics=*/nullptr);
    }

    MockTransport*                 mock_raw_ = nullptr;
    CalibrationData                cal_{};
    ScenarioConfig                 scenario_;
    SPSCRingBuffer<uint16_t>       tx_ring_{TX_RING_CAPACITY};
    SPSCRingBuffer<uint16_t>       rx_ring_{RX_RING_CAPACITY};
    std::unique_ptr<Modem>         modem_;
    std::unique_ptr<ChannelEngine> engine_;
    std::unique_ptr<ModemIO>       io_;
};

// ---------------------------------------------------------------------------

TEST(LoopbackMockPhase2, CalibrationSucceeds) {
    LoopbackMockFixture fix;
    EXPECT_EQ(fix.modem_->calibration().adc_bits, 16);
    EXPECT_EQ(fix.modem_->calibration().adc_sampling_rate, 500'000u);
}

TEST(LoopbackMockPhase2, EnterHilModeTransitionsMockToRx) {
    LoopbackMockFixture fix;
    EXPECT_EQ(fix.mock_raw_->current_state(), ModemState::IDLE);
    fix.modem_->enter_hil_mode();
    EXPECT_EQ(fix.mock_raw_->current_state(), ModemState::RX);
}

// ---------------------------------------------------------------------------
// End-to-end: TX waveform → ChannelEngine → rx packets sent to mock modem
// ---------------------------------------------------------------------------

TEST(LoopbackMockPhase2, FullTxSettlingRxCycleProducesRxPackets) {
    LoopbackMockFixture fix;

    // Pre-load TX waveform (3 packets worth)
    const size_t N = static_cast<size_t>(protocol::DATA_SAMPLES_PER_PKT) * 3;
    constexpr uint16_t MID = 1u << 15;
    std::vector<uint16_t> waveform(N, static_cast<uint16_t>(MID + 4000));
    fix.mock_raw_->enqueue_tx_waveform(waveform);

    fix.engine_->start();

    // Schedule mock state transitions: TX immediately, then auto-progresses
    // through SETTLING → RX once the waveform is consumed.
    fix.mock_raw_->schedule_state_transition(0, ModemState::TX);

    std::thread io_thread([&] { fix.io_->run(); });

    // Run for a fixed window. We deliberately do NOT poll
    // mock_raw_->received_rx_packets() here — that's a vector mutated by
    // the I/O thread via send_data, and reading it concurrently is a real
    // data race (TSan-flagged). Stop the I/O thread first, then read.
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    fix.io_->stop();
    io_thread.join();
    fix.engine_->stop();

    EXPECT_GT(fix.mock_raw_->received_rx_packets().size(), 0u)
        << "MockTransport received no RX packets through the full Phase 2 pipeline";
}
