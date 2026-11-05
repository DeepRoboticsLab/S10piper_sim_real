#pragma once

#include "piper_safe_workspace.hpp"

// Physical arm commands and whole-body task observations are different
// channels. X/Y still drive Piper, but only Z varies in the policy task.
struct LayeredArmTargets {
    types::Vec3f policy_xyz;
    types::Vec3f piper_offset;
};

inline LayeredArmTargets SplitLayeredArmTargets(
        const types::Vec3f& requested, bool home_reset) {
    const auto home = piper_safe_workspace::Home();
    const auto physical = home_reset ? home :
        piper_safe_workspace::Clamp(requested);
    return {types::Vec3f(home.x(), home.y(), physical.z()), physical - home};
}
