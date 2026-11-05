#pragma once

#include <chrono>

// A delayed DDS sample must not stretch the actor's 10-frame history by 4x.
// Commands use the latest feedback; the hardware freshness watchdog still
// rejects old samples. Missed deadlines are skipped instead of replayed.
class ControlTicker {
public:
    using Clock = std::chrono::steady_clock;
    static constexpr auto kPeriod = std::chrono::milliseconds(5);

    explicit ControlTicker(Clock::time_point start) : next_(start) {}

    bool Due(Clock::time_point now) {
        if (now < next_) return false;
        next_ += kPeriod;
        if (next_ <= now) next_ = now + kPeriod;
        return true;
    }

    Clock::time_point Next() const { return next_; }

private:
    Clock::time_point next_;
};
