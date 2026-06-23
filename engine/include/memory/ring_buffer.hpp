#pragma once

#include <atomic>
#include <cstddef>

namespace orbitforge::memory {

// Lock-free single-producer/single-consumer ring buffer. One thread may call
// push(), a different thread may call pop(), concurrently, without locks.
// Capacity must be a power of 2 so index wraparound is a bitmask, not a
// modulo. write_pos_ and read_pos_ are each on their own cache line — they
// are written by different threads, and without the padding they would
// share a line, so every push would invalidate the consumer's cached copy
// of a line it never touched (false sharing).
template <typename T, size_t N>
class SPSCRingBuffer {
    static_assert((N & (N - 1)) == 0, "capacity must be a power of 2");

public:
    bool push(const T& item) noexcept {
        const size_t w = write_pos_.load(std::memory_order_relaxed);
        const size_t r = read_pos_.load(std::memory_order_acquire);
        if ((w - r) == N) return false;
        buffer_[w & (N - 1)] = item;
        write_pos_.store(w + 1, std::memory_order_release);
        return true;
    }

    bool pop(T& item) noexcept {
        const size_t r = read_pos_.load(std::memory_order_relaxed);
        const size_t w = write_pos_.load(std::memory_order_acquire);
        if (r == w) return false;
        item = buffer_[r & (N - 1)];
        read_pos_.store(r + 1, std::memory_order_release);
        return true;
    }

    // Approximate occupancy. Racy by construction (the producer/consumer may
    // advance between the two loads) — for diagnostics only.
    size_t size() const noexcept {
        const size_t w = write_pos_.load(std::memory_order_acquire);
        const size_t r = read_pos_.load(std::memory_order_acquire);
        return w - r;
    }

    static constexpr size_t capacity() noexcept { return N; }

    // Drops all queued items. Only safe to call when no other thread is
    // concurrently pushing/popping (e.g. simulation reset, with the worker
    // thread already stopped).
    void clear() noexcept {
        write_pos_.store(0, std::memory_order_relaxed);
        read_pos_.store(0, std::memory_order_relaxed);
    }

private:
    static constexpr size_t k_pad = 64 - sizeof(std::atomic<size_t>);

    alignas(64) std::atomic<size_t> write_pos_{0};
    char pad_w_[k_pad]{};
    alignas(64) std::atomic<size_t> read_pos_{0};
    char pad_r_[k_pad]{};
    alignas(64) T buffer_[N];
};

}
