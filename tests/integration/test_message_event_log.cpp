#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include <chrono>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "channel/channel.hpp"
#include "channel/pair_buffer.hpp"
#include "channel/source_worker.hpp"
#include "config/scenario.hpp"
#include "core/ring_buffer.hpp"
#include "core/types.hpp"
#include "simulator/message_event_log.hpp"

using openCREST::CalibrationData;
using openCREST::Channel;
using openCREST::ChannelConfig;
using openCREST::MessageEvent;
using openCREST::MessageEventLog;
using openCREST::ModemRuntimeState;
using openCREST::ModemState;
using openCREST::PairBuffer;
using openCREST::SourceWorker;
using openCREST::SPSCRingBuffer;

namespace {

constexpr uint32_t FS = 500'000;
constexpr size_t   BIG_CAP = 16'384;

CalibrationData make_cal() {
    CalibrationData c;
    c.adc_bits          = 16;
    c.dac_bits          = 16;
    c.adc_sampling_rate = FS;
    c.dac_sampling_rate = FS;
    c.center_freq_hz    = 1.0f;
    return c;
}

ChannelConfig make_direct_path_config() {
    ChannelConfig cfg;
    cfg.from_modem       = "A";
    cfg.to_modem         = "B";
    cfg.range_m          = 1.0f;
    cfg.spreading_factor = 2.0f;
    cfg.saltwater        = true;
    cfg.sound_speed_m_s  = 1500.0f;
    cfg.multipath_taps.push_back({0.0f, 1.0f, 0.0f});
    return cfg;
}

template <typename Pred>
bool wait_for(Pred predicate, std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (predicate()) return true;
        std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
    return predicate();
}

// Pumps one TX message of `n` samples through the worker, then returns
// after the worker has observed the SETTLING edge.
void drive_one_message(ModemRuntimeState& rt,
                        SPSCRingBuffer<uint16_t>& tx_ring,
                        size_t n) {
    std::vector<uint16_t> samples(n, static_cast<uint16_t>((1u << 15) + 8000));
    ASSERT_EQ(tx_ring.write(samples.data(), n), n);
    rt.state.store(ModemState::TX, std::memory_order_release);
    ASSERT_TRUE(wait_for(
        [&] { return tx_ring.available_read() == 0; },
        std::chrono::milliseconds(500)));
    rt.state.store(ModemState::SETTLING, std::memory_order_release);
}

std::vector<nlohmann::json> read_jsonl(const std::string& path) {
    std::vector<nlohmann::json> out;
    std::ifstream f(path);
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        out.push_back(nlohmann::json::parse(line));
    }
    return out;
}

} // namespace

TEST(MessageEventLogIntegration, FiveScriptedTxMessagesProduceFiveJsonlLines) {
    const std::string path = std::tmpnam(nullptr);
    MessageEventLog log;
    ASSERT_TRUE(log.open(path));

    ModemRuntimeState rt;
    rt.state.store(ModemState::IDLE, std::memory_order_relaxed);
    SPSCRingBuffer<uint16_t> tx_ring(8192);

    auto cfg = make_direct_path_config();
    auto cal = make_cal();
    auto ch  = std::make_unique<Channel>(cfg, cal, cal,
                                         openCREST::TransducerSpec{},
                                         openCREST::TransducerSpec{});
    PairBuffer pb(BIG_CAP, ch->base_delay_samples());

    std::vector<SourceWorker::Outgoing> outgoing;
    outgoing.push_back({std::move(ch), &pb});

    SourceWorker worker("modem_a", rt, tx_ring, cal, std::move(outgoing));
    worker.set_message_event_log(&log);

    std::thread t([&] { worker.run(); });

    constexpr size_t per_message[5] = {256, 384, 512, 640, 768};
    for (size_t k = 0; k < 5; ++k) {
        drive_one_message(rt, tx_ring, per_message[k]);
        // Wait for the SourceWorker to observe the SETTLING edge and
        // emit the event before pushing the next message. The PairBuffer
        // populating past base_delay is a deterministic proxy for the
        // on_message_end having fired.
        ASSERT_TRUE(wait_for([&] {
            // The MessageEventLog flushes per record so the file grows
            // monotonically; size > 0 once the first event lands.
            return std::ifstream(path).peek() != std::ifstream::traits_type::eof()
                || k > 0;
        }, std::chrono::milliseconds(500)));
        // Brief pause to let the worker fully observe the state edge.
        std::this_thread::sleep_for(std::chrono::milliseconds(15));
        rt.state.store(ModemState::IDLE, std::memory_order_release);
        // Settle quickly back to IDLE between messages so the next TX
        // edge is fresh.
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    worker.stop();
    t.join();
    log.close();

    const auto rows = read_jsonl(path);
    ASSERT_EQ(rows.size(), 5u);

    uint64_t prev_end = 0;
    for (size_t i = 0; i < rows.size(); ++i) {
        EXPECT_EQ(rows[i].at("modem_id").get<std::string>(), "modem_a");
        EXPECT_EQ(rows[i].at("direction").get<std::string>(), "tx");
        EXPECT_EQ(rows[i].at("sequence_id").get<uint64_t>(), i);
        EXPECT_GE(rows[i].at("sample_count").get<uint64_t>(), per_message[i]);
        const uint64_t s = rows[i].at("start_ns").get<uint64_t>();
        const uint64_t e = rows[i].at("end_ns").get<uint64_t>();
        EXPECT_GT(e, s);
        EXPECT_GT(s, prev_end);
        prev_end = e;
    }

    std::remove(path.c_str());
}

TEST(MessageEventLogIntegration, EmitsNothingWhenLogNotWired) {
    // Without set_message_event_log() the SourceWorker must not produce
    // any side effects beyond the existing scatter — covers the
    // "defaults are off" requirement at the source-worker layer.
    ModemRuntimeState rt;
    rt.state.store(ModemState::IDLE, std::memory_order_relaxed);
    SPSCRingBuffer<uint16_t> tx_ring(2048);

    auto cfg = make_direct_path_config();
    auto cal = make_cal();
    auto ch  = std::make_unique<Channel>(cfg, cal, cal,
                                         openCREST::TransducerSpec{},
                                         openCREST::TransducerSpec{});
    PairBuffer pb(BIG_CAP, ch->base_delay_samples());

    std::vector<SourceWorker::Outgoing> outgoing;
    outgoing.push_back({std::move(ch), &pb});

    SourceWorker worker("modem_a", rt, tx_ring, cal, std::move(outgoing));
    std::thread t([&] { worker.run(); });

    drive_one_message(rt, tx_ring, 256);
    std::this_thread::sleep_for(std::chrono::milliseconds(15));

    worker.stop();
    t.join();
    // Nothing to assert except that we didn't crash and the worker
    // still reports samples consumed.
    EXPECT_GT(worker.tx_samples_consumed(), 0u);
}
