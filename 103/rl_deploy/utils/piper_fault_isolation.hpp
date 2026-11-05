#pragma once

#include <chrono>
#include <cstdint>
#include <exception>
#include <iostream>

// Runtime boundary for OPTIONAL arm writes only. Never use this around S10
// publishing, policy inference, IMU freshness or the robot's tilt protection.
class PiperFaultIsolation {
    using Clock = std::chrono::steady_clock;
    Clock::time_point last_log_{};
    bool failed_ = false;
    uint64_t errors_ = 0;

    void Failed(const char* reason) {
        failed_ = true;
        ++errors_;
        const auto now = Clock::now();
        if (last_log_.time_since_epoch().count() == 0 ||
                now-last_log_ >= std::chrono::seconds(1)) {
            std::cerr << "[Piper/ISOLATED] " << reason
                      << "; S10 leg/wheel publication and RL continue. "
                      << "Robot tilt/feedback protections remain active.\n";
            last_log_ = now;
        }
    }
public:
    template<class Action> bool Run(Action&& action) {
        try {
            action();
            if (failed_) {
                std::cerr << "[Piper] Optional arm call recovered; "
                             "S10 control was not interrupted.\n";
                failed_ = false;
            }
            return true;
        } catch (const std::exception& error) {
            Failed(error.what());
        } catch (...) {
            Failed("unknown optional arm exception");
        }
        return false;
    }
    uint64_t ErrorCount() const { return errors_; }
};
