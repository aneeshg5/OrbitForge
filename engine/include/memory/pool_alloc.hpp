#pragma once

#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>

namespace orbitforge::memory {

// Fixed-size block pool allocator. allocate()/deallocate() are O(1), make no
// syscalls, and take no lock — used for working buffers that must be
// returned between simulation ticks without going through malloc/free,
// whose internal locking and fragmentation introduce unpredictable latency.
template <size_t BlockSize, size_t NumBlocks>
class PoolAllocator {
    static_assert(BlockSize % 64 == 0, "blocks must be cache-line aligned");
    static_assert(NumBlocks > 0, "pool must hold at least one block");

public:
    PoolAllocator() noexcept {
        for (size_t i = 0; i < NumBlocks; ++i) {
            std::byte* block = &pool_[i * BlockSize];
            free_list_[i] = block;
#ifndef NDEBUG
            set_free(block, true);
#endif
        }
        free_head_ = NumBlocks;
    }

    // Returns a pointer to a free block, or nullptr if the pool is exhausted.
    void* allocate() noexcept {
        if (free_head_ == 0) return nullptr;
        std::byte* block = free_list_[--free_head_];
#ifndef NDEBUG
        set_free(block, false);
#endif
        return block;
    }

    // Returns a block previously obtained from allocate() to the pool.
    void deallocate(void* p) noexcept {
        if (p == nullptr) return;
        auto* block = static_cast<std::byte*>(p);
        assert(owns(block) && "deallocate() called with pointer outside the pool");
        assert(((block - pool_.data()) % BlockSize) == 0 &&
               "deallocate() called with misaligned block pointer");
        // Double-free must be checked before the headroom check below: a
        // double-free always also looks like free-list overflow (free_head_
        // is already at NumBlocks), so checking overflow first would report
        // the wrong cause.
#ifndef NDEBUG
        assert(!is_free(block) && "double free detected");
        set_free(block, true);
#endif
        assert(free_head_ < NumBlocks && "pool free list overflow");
        free_list_[free_head_++] = block;
    }

    static constexpr size_t block_size() noexcept { return BlockSize; }
    static constexpr size_t num_blocks() noexcept { return NumBlocks; }
    size_t free_count() const noexcept { return free_head_; }

private:
    bool owns(const std::byte* p) const noexcept {
        return p >= pool_.data() && p < pool_.data() + pool_.size();
    }

#ifndef NDEBUG
    void set_free(const std::byte* block, bool is_free_now) noexcept {
        const size_t idx = static_cast<size_t>(block - pool_.data()) / BlockSize;
        if (is_free_now) {
            free_bitmap_[idx / 64] |= (uint64_t{1} << (idx % 64));
        } else {
            free_bitmap_[idx / 64] &= ~(uint64_t{1} << (idx % 64));
        }
    }
    bool is_free(const std::byte* block) const noexcept {
        const size_t idx = static_cast<size_t>(block - pool_.data()) / BlockSize;
        return ((free_bitmap_[idx / 64] >> (idx % 64)) & 1u) != 0;
    }
    std::array<uint64_t, (NumBlocks + 63) / 64> free_bitmap_{};
#endif

    alignas(64) std::array<std::byte, BlockSize * NumBlocks> pool_;
    std::array<std::byte*, NumBlocks> free_list_;
    size_t free_head_;
};

}  // namespace orbitforge::memory
