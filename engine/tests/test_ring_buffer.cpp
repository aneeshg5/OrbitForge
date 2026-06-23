#include <gtest/gtest.h>

#include <cstddef>
#include <thread>
#include <vector>

#include "memory/ring_buffer.hpp"

using orbitforge::memory::SPSCRingBuffer;

TEST(RingBuffer, SingleThreadPushPopFIFOOrder) {
    SPSCRingBuffer<int, 16> rb;
    for (int i = 0; i < 10; ++i) {
        ASSERT_TRUE(rb.push(i));
    }
    for (int i = 0; i < 10; ++i) {
        int item = -1;
        ASSERT_TRUE(rb.pop(item));
        EXPECT_EQ(item, i);
    }
}

TEST(RingBuffer, ClearResetsToEmpty) {
    SPSCRingBuffer<int, 8> rb;
    for (int i = 0; i < 5; ++i) ASSERT_TRUE(rb.push(i));
    rb.clear();

    int item = -1;
    EXPECT_FALSE(rb.pop(item));
    EXPECT_EQ(rb.size(), 0u);
    for (int i = 0; i < 8; ++i) ASSERT_TRUE(rb.push(i));
}

TEST(RingBuffer, EmptyBufferPopFails) {
    SPSCRingBuffer<int, 16> rb;
    int item = -1;
    EXPECT_FALSE(rb.pop(item));
}

TEST(RingBuffer, FullBufferRejectsPush) {
    SPSCRingBuffer<int, 8> rb;
    for (int i = 0; i < 8; ++i) {
        ASSERT_TRUE(rb.push(i));
    }
    EXPECT_FALSE(rb.push(99));

    int item = -1;
    ASSERT_TRUE(rb.pop(item));
    EXPECT_EQ(item, 0);
    EXPECT_TRUE(rb.push(99));
}

TEST(RingBuffer, MultiThreadProducerConsumerNoLoss) {
    constexpr size_t k_items = 10'000'000;
    SPSCRingBuffer<size_t, 1024> rb;

    std::vector<size_t> received;
    received.reserve(k_items);

    std::thread producer([&rb] {
        for (size_t i = 0; i < k_items; ++i) {
            while (!rb.push(i)) {
            }
        }
    });

    std::thread consumer([&rb, &received] {
        size_t item = 0;
        while (received.size() < k_items) {
            if (rb.pop(item)) {
                received.push_back(item);
            }
        }
    });

    producer.join();
    consumer.join();

    ASSERT_EQ(received.size(), k_items);
    for (size_t i = 0; i < k_items; ++i) {
        EXPECT_EQ(received[i], i) << "FIFO order violated at index " << i;
    }
}
