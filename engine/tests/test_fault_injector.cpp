#include <gtest/gtest.h>

#include "faults/fault_injector.hpp"

using orbitforge::faults::FaultConfig;
using orbitforge::faults::FaultQueue;
using orbitforge::faults::FaultType;

TEST(FaultQueue, TryTakeFailsWhenEmpty) {
    FaultQueue q;
    FaultConfig out;
    EXPECT_FALSE(q.try_take(out));
}

TEST(FaultQueue, SetThenTryTakeReturnsConfig) {
    FaultQueue q;
    FaultConfig cfg;
    cfg.type = FaultType::gps_spike;
    cfg.onset_t = 60.0;
    cfg.duration = 0.0;
    cfg.magnitude = 500.0;
    q.set(cfg);

    FaultConfig out;
    ASSERT_TRUE(q.try_take(out));
    EXPECT_EQ(out.type, FaultType::gps_spike);
    EXPECT_DOUBLE_EQ(out.onset_t, 60.0);
    EXPECT_DOUBLE_EQ(out.magnitude, 500.0);
}

TEST(FaultQueue, TryTakeClearsPending) {
    FaultQueue q;
    FaultConfig cfg;
    cfg.type = FaultType::gps_dropout;
    q.set(cfg);

    FaultConfig out;
    ASSERT_TRUE(q.try_take(out));
    EXPECT_FALSE(q.try_take(out));
}

TEST(FaultQueue, LatestSetOverwritesPrevious) {
    FaultQueue q;
    FaultConfig first;
    first.type = FaultType::maneuver;
    first.magnitude = 1.0;
    q.set(first);

    FaultConfig second;
    second.type = FaultType::sensor_bias;
    second.magnitude = 2.0;
    q.set(second);

    FaultConfig out;
    ASSERT_TRUE(q.try_take(out));
    EXPECT_EQ(out.type, FaultType::sensor_bias);
    EXPECT_DOUBLE_EQ(out.magnitude, 2.0);
    EXPECT_FALSE(q.try_take(out));
}
