#include <gtest/gtest.h>
#include "config/scenario.hpp"
#include "config/scenario_loader.hpp"
#include <string>

using openCREST::ChannelMode;
using openCREST::ScenarioLoader;

#ifndef OPENCREST_SCENARIO_DIR
#error "OPENCREST_SCENARIO_DIR must be defined by the build system"
#endif

TEST(ScenarioFiles, AllShippedYamlsLoad) {
    const std::string dir = OPENCREST_SCENARIO_DIR;
    for (const std::string name : {
        "loopback_identity.yaml",
        "loopback_simple.yaml",
        "loopback_multipath.yaml",
        "two_modem_shallow.yaml",
        "geometric_approach.yaml",
    }) {
        const std::string path = dir + "/" + name;
        EXPECT_NO_THROW({
            auto sc = ScenarioLoader::load(path);
            EXPECT_FALSE(sc.modems.empty()) << path;
            EXPECT_FALSE(sc.channels.empty()) << path;
        }) << path;
    }
}

TEST(ScenarioFiles, GeometricApproachIsGeometricMode) {
    const std::string path =
        std::string(OPENCREST_SCENARIO_DIR) + "/geometric_approach.yaml";
    const auto sc = ScenarioLoader::load(path);
    ASSERT_FALSE(sc.channels.empty());
    EXPECT_EQ(sc.channels[0].mode, ChannelMode::Geometric);
    EXPECT_FLOAT_EQ(sc.channels[0].initial_range_m, 1000.0f);
    EXPECT_FLOAT_EQ(sc.channels[0].geometry.water_depth_m, 120.0f);
    EXPECT_TRUE(sc.channels[0].multipath_taps.empty());
}
