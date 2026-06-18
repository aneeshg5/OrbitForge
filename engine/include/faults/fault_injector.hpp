#pragma once

#include <atomic>
#include <cstdint>

namespace orbitforge::faults {

enum class FaultType : uint8_t {
    none = 0,
    gps_spike,
    gps_dropout,
    maneuver,
    drag_coeff_error,
    sensor_bias,
};

struct FaultConfig {
    FaultType type      = FaultType::none;
    double    onset_t   = 0.0;   // simulation time of fault onset, seconds
    double    duration  = 0.0;   // seconds; 0 = instantaneous
    double    magnitude = 0.0;   // fault-specific parameter
};

// Thread-safe single-element mailbox: the UI thread calls
// set() to queue a fault config; the simulation worker calls try_take()
// once per tick to pick it up. Only the most recently set() fault survives
// if set() is called again before the worker reads it — this is a mailbox,
// not a queue. The release-store/acquire-load pair on `pending_` is what
// makes the worker's read of `staged_` safe without a separate lock: the
// store to staged_ happens-before the load that observes pending_ == true.
class FaultQueue {
public:
    void set(const FaultConfig& cfg) noexcept {
        staged_ = cfg;
        pending_.store(true, std::memory_order_release);
    }

    // Returns true and fills `out` if a fault was pending; clears the
    // pending flag in that case. Returns false (leaving `out` untouched)
    // if nothing was pending.
    bool try_take(FaultConfig& out) noexcept {
        if (!pending_.load(std::memory_order_acquire)) return false;
        out = staged_;
        pending_.store(false, std::memory_order_release);
        return true;
    }

private:
    FaultConfig staged_;
    std::atomic<bool> pending_{false};
};

}  // namespace orbitforge::faults
