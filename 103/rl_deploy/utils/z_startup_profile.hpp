#pragma once
#include <algorithm>
namespace z_startup_profile {
inline constexpr float kPreviousOriginalArmSpeedRadS=0.4f;
inline constexpr float kArmSpeedRadS=2.0f*kPreviousOriginalArmSpeedRadS;
inline float ArmFrameStep(float dt) {
    // No catch-up bursts: at most one 50 Hz step after a delayed tick.
    return kArmSpeedRadS*std::clamp(dt,0.0f,0.020f);
}
}
