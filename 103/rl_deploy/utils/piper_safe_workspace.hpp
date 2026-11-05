#pragma once

#include "common_types.h"
#include <algorithm>

namespace piper_safe_workspace {

// S10/Piper operator workspace: extended lateral command range, shared by VR and
// keyboard. Orin opts into the same named box; training normalization stays fixed.
inline constexpr float kHomeX = 0.42f;
inline constexpr float kHomeY = 0.00f;
inline constexpr float kHomeZ = 0.05f;
inline constexpr float kMinX = -0.42f;
inline constexpr float kMaxX =  0.58f;
inline constexpr float kMinY = -0.50f;
inline constexpr float kMaxY =  0.50f;
inline constexpr float kMinZ = -0.30f;
inline constexpr float kMaxZ =  0.42f;
inline constexpr const char* kProfile = "lateral_box";

inline types::Vec3f Home() {
    return {kHomeX, kHomeY, kHomeZ};
}

inline types::Vec3f Clamp(const types::Vec3f& requested) {
    if (!requested.allFinite()) return Home();
    return {
        std::clamp(requested.x(), kMinX, kMaxX),
        std::clamp(requested.y(), kMinY, kMaxY),
        std::clamp(requested.z(), kMinZ, kMaxZ),
    };
}

}  // namespace piper_safe_workspace
