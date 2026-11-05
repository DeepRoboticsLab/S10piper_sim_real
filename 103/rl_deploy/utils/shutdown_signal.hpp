#pragma once

#include <csignal>

namespace shutdown_signal {

inline volatile std::sig_atomic_t requested = 0;

inline void Handle(int) {
    requested = 1;
}

inline bool Requested() {
    return requested != 0;
}

}  // namespace shutdown_signal
