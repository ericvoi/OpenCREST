#include "simulator/run_summary.hpp"

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

using openCREST::RunSummary;
using openCREST::serialize_run_summary;
using openCREST::write_run_summary_json;
using openCREST::resolve_run_summary_path;
using openCREST::snapshot_engine_counters;

namespace {

RunSummary make_sample_summary() {
    RunSummary s;
    s.scenario_name = "geometric_approach";
    s.scenario_path = "experiments/configs/run_0042.yaml";
    s.random_seed   = 12345;
    s.started_at    = "2026-05-18T18:34:21Z";
    s.ended_at      = "2026-05-18T18:35:21Z";
    s.duration_s    = 60.0;
    s.modems        = {"modem_a", "modem_b"};

    s.processing_time.count          = 60000;
    s.processing_time.mean_us        = 312.4;
    s.processing_time.p50_us         = 280;
    s.processing_time.p95_us         = 540;
    s.processing_time.p99_us         = 720;
    s.processing_time.max_us         = 1430;
    s.processing_time.underrun_count = 0;

    s.channel_engine.rx_ring_underruns = 0;
    s.channel_engine.fw_rx_underruns   = 0;
    s.channel_engine.tx_packets_total  = 117612;
    s.channel_engine.rx_packets_total  = 117612;

    s.log_files.events = {"modem_a_events.jsonl", "modem_b_events.jsonl"};
    s.log_files.wav    = {"modem_a_tx.wav", "modem_b_tx.wav"};
    return s;
}

} // namespace

TEST(RunSummary, SerializeRoundTripIsFieldWiseEqual) {
    const RunSummary in = make_sample_summary();
    const std::string text = serialize_run_summary(in);

    auto j = nlohmann::json::parse(text);
    EXPECT_EQ(j.at("scenario_name").get<std::string>(),  in.scenario_name);
    EXPECT_EQ(j.at("scenario_path").get<std::string>(),  in.scenario_path);
    EXPECT_EQ(j.at("random_seed").get<uint64_t>(),       in.random_seed);
    EXPECT_EQ(j.at("started_at").get<std::string>(),     in.started_at);
    EXPECT_EQ(j.at("ended_at").get<std::string>(),       in.ended_at);
    EXPECT_DOUBLE_EQ(j.at("duration_s").get<double>(),   in.duration_s);
    EXPECT_EQ(j.at("modems").get<std::vector<std::string>>(), in.modems);

    auto pt = j.at("processing_time");
    EXPECT_EQ(pt.at("count").get<uint64_t>(),    in.processing_time.count);
    EXPECT_DOUBLE_EQ(pt.at("mean_us").get<double>(),
                     in.processing_time.mean_us);
    EXPECT_EQ(pt.at("p50_us").get<uint64_t>(),   in.processing_time.p50_us);
    EXPECT_EQ(pt.at("p95_us").get<uint64_t>(),   in.processing_time.p95_us);
    EXPECT_EQ(pt.at("p99_us").get<uint64_t>(),   in.processing_time.p99_us);
    EXPECT_EQ(pt.at("max_us").get<uint64_t>(),   in.processing_time.max_us);
    EXPECT_EQ(pt.at("underrun_count").get<uint64_t>(),
              in.processing_time.underrun_count);

    auto ce = j.at("channel_engine");
    EXPECT_EQ(ce.at("rx_ring_underruns").get<uint64_t>(),
              in.channel_engine.rx_ring_underruns);
    EXPECT_EQ(ce.at("fw_rx_underruns").get<uint64_t>(),
              in.channel_engine.fw_rx_underruns);
    EXPECT_EQ(ce.at("tx_packets_total").get<uint64_t>(),
              in.channel_engine.tx_packets_total);
    EXPECT_EQ(ce.at("rx_packets_total").get<uint64_t>(),
              in.channel_engine.rx_packets_total);

    auto lf = j.at("log_files");
    EXPECT_EQ(lf.at("events").get<std::vector<std::string>>(),
              in.log_files.events);
    EXPECT_TRUE(lf.at("cdc").get<std::vector<std::string>>().empty());
    EXPECT_EQ(lf.at("wav").get<std::vector<std::string>>(),
              in.log_files.wav);
}

TEST(RunSummary, WriteToFileReadsBackIdentically) {
    const RunSummary in = make_sample_summary();
    const std::string path = std::tmpnam(nullptr);
    ASSERT_TRUE(write_run_summary_json(in, path));

    std::ifstream f(path);
    std::stringstream buf;
    buf << f.rdbuf();
    auto j = nlohmann::json::parse(buf.str());
    EXPECT_EQ(j.at("scenario_name").get<std::string>(), in.scenario_name);
    std::remove(path.c_str());
}

TEST(RunSummary, PathResolutionUsesScenarioWhenEmpty) {
    openCREST::ScenarioConfig sc;
    sc.name = "two_modem_shallow";
    sc.logging.output_directory = "/tmp/results";
    EXPECT_EQ(resolve_run_summary_path(sc),
              "/tmp/results/two_modem_shallow_summary.json");
}

TEST(RunSummary, PathResolutionAcceptsTrailingSlash) {
    openCREST::ScenarioConfig sc;
    sc.name = "foo";
    sc.logging.output_directory = "/tmp/out/";
    EXPECT_EQ(resolve_run_summary_path(sc), "/tmp/out/foo_summary.json");
}

TEST(RunSummary, PathResolutionHonoursExplicitOverride) {
    openCREST::ScenarioConfig sc;
    sc.name = "ignored";
    sc.logging.output_directory = "/ignored";
    sc.logging.run_summary_path = "/custom/elsewhere.json";
    EXPECT_EQ(resolve_run_summary_path(sc), "/custom/elsewhere.json");
}

TEST(RunSummary, EngineCountersSnapshotsLiveMetrics) {
    openCREST::Metrics m;
    m.tx_packets_received.store(42);
    m.rx_packets_sent.store(43);
    m.rx_ring_underruns.store(5);
    m.fw_rx_underruns.store(7);
    const auto c = snapshot_engine_counters(m);
    EXPECT_EQ(c.tx_packets_total, 42u);
    EXPECT_EQ(c.rx_packets_total, 43u);
    EXPECT_EQ(c.rx_ring_underruns, 5u);
    EXPECT_EQ(c.fw_rx_underruns,   7u);
}
