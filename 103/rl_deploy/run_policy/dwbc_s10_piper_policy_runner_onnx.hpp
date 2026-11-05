/**
 * @file dwbc_s10_piper_policy_runner_onnx.hpp
 * @brief ONNX adapter matching the dwbcS10piper training contract.
 *
 * Simulator order: [S10 leg3 + wheel] x 4, then Piper joint1..joint6.
 * Actor contract: 836 observations (76 current + 10 history), 22 actions.
 */
#pragma once

#include "policy_runner_base.hpp"
#include "piper_safe_workspace.hpp"
#include <onnxruntime_cxx_api.h>
#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <filesystem>
#include <iostream>
#include <memory>
#include <cstdlib>
#include <vector>

using namespace types;

#ifndef S10_POLICY_FILE
#define S10_POLICY_FILE "policy.onnx"
#endif

class DwbcS10PiperPolicyRunnerONNX : public PolicyRunnerBase {
private:
    static constexpr int kRobotDof = 22;
    static constexpr int kActionDim = 22;
    static constexpr int kSingleObs = 76;
    static constexpr int kHistoryLen = 10;
    static constexpr int kInputDim = kSingleObs * (kHistoryLen + 1);
#ifdef S10_K8_RECOVERY
    static constexpr float kLegWheelEmaAlpha = 0.40f;
    static constexpr float kStandingLegMaxDeltaRad = 0.08f;
#elif defined(S10_PIPER_V24) || defined(S10_PIPER_V25) || defined(S10_K8_LOW035)
    static constexpr float kLegWheelEmaAlpha = 0.20f;
    static constexpr float kStandingLegMaxDeltaRad = 0.006f;
#elif defined(S10_PIPER_V23)
    static constexpr float kLegWheelEmaAlpha = 0.20f;
    static constexpr float kStandingLegMaxDeltaRad = 0.0f;
#else
    static constexpr float kLegWheelEmaAlpha = 0.20f;
    static constexpr float kStandingLegMaxDeltaRad = 0.003f;
#endif
#ifdef S10_PIPER_V23_STABLE
    // Deployment-only smoothing for the twelve position-controlled leg joints.
    // Keep the wheel EMA and the policy's raw-action observation unchanged.
    static constexpr float kStableLegEmaAlpha = 0.08f;
    static constexpr float kStableLegDeadbandRad = 0.002f;
    static constexpr float kStableLegStandingMaxDeltaRad = 0.006f;
    static constexpr float kStableLegMovingMaxDeltaRad = 0.020f;
#endif
#ifdef S10_K8_ARM
    // The K8 wrist amplifies small policy target changes at its tip.
    // This checkpoint was trained with joint6 at -1.08 rad. Deployment holds
    // it at zero and maps that fixed pose to the policy's neutral observation.
#if defined(S10_K8_LOW035) || defined(S10_K8_RECOVERY_V7)
    static constexpr float kK8Joint6TargetRad = -1.08f;
#else
    static constexpr float kK8Joint6TargetRad = 0.0f;
#endif
#ifdef S10_K8_RECOVERY
    static constexpr float kArmEmaAlpha = 0.50f;
    static constexpr float kArmMaxDeltaRad = 0.08f;
    static constexpr float kArmTargetDeadbandRad = 0.0f;
#elif defined(S10_K8_LOW035)
    static constexpr float kArmEmaAlpha = 0.20f;
    static constexpr float kArmMaxDeltaRad = 0.020f;
    static constexpr float kArmTargetDeadbandRad = 0.0f;
#else
    static constexpr float kArmEmaAlpha = 0.05f;
    static constexpr float kArmMaxDeltaRad = 0.006f;
    static constexpr float kArmTargetDeadbandRad = 0.005f;
#endif
    static constexpr int kArmSettleSteps = 200;  // 4 s at 50 Hz.
#else
    static constexpr float kArmEmaAlpha = 0.20f;
    static constexpr float kArmMaxDeltaRad = 0.020f;
#endif

    Ort::Env env_;
    Ort::SessionOptions session_options_;
    std::unique_ptr<Ort::Session> session_;
    Ort::MemoryInfo memory_info_;
    std::vector<const char*> input_names_{"obs"};
    std::vector<const char*> output_names_{"actions"};
    bool model_ready_ = false;

    VecXf current_obs_ = VecXf::Zero(kSingleObs);
    VecXf input_obs_ = VecXf::Zero(kInputDim);
    VecXf last_raw_action_ = VecXf::Zero(kActionDim);
    VecXf filtered_leg_action_ = VecXf::Zero(16);
#ifdef S10_PIPER_V23_LEG_HOLD
    VecXf locked_leg_pos_ = VecXf::Zero(16);
    bool leg_hold_initialized_ = false;
#endif
#ifdef S10_PIPER_V23_STABLE
    VecXf smoothed_leg_action_ = VecXf::Zero(16);
#endif
    VecXf arm_target_ = VecXf::Zero(6);
    std::deque<VecXf> history_;
    RobotAction action_;

    Vec3f velocity_command_ = Vec3f::Zero();
    Vec3f arm_command_{piper_safe_workspace::Home()};
#ifdef S10_K8_ARM
    Vec3f previous_arm_command_{0.42f, 0.0f, 0.05f};
    int arm_settle_steps_ = 0;
    bool arm_target_hold_ = false;
    bool hold_initial_home_ = true;
#endif
    bool pure_x_leg_hold_active_ = false;
#ifdef S10_V26_NEGATIVE_YAW_MIRROR
    bool yaw_mirror_initialized_ = false;
    bool yaw_mirror_active_ = false;
#endif

    const std::array<float, 16> dog_default_{{
#ifdef S10_PIPER_LOW035
         0.08f, -0.7269239136f,  1.4538478271f, 0.0f,
        -0.08f, -0.7269239136f,  1.4538478271f, 0.0f,
         0.08f,  0.7269239136f, -1.4538478271f, 0.0f,
        -0.08f,  0.7269239136f, -1.4538478271f, 0.0f
#else
         0.08f, -0.45f,  0.90f, 0.0f,
        -0.08f, -0.45f,  0.90f, 0.0f,
         0.08f,  0.45f, -0.90f, 0.0f,
        -0.08f,  0.45f, -0.90f, 0.0f
#endif
    }};
    const std::array<float, 16> dog_action_scale_{{
#ifdef S10_K8_RECOVERY_V9
        0.40f, 0.45f, 0.45f, 14.0f,
        0.40f, 0.45f, 0.45f, 14.0f,
        0.40f, 0.45f, 0.45f, 14.0f,
        0.40f, 0.45f, 0.45f, 14.0f
#else
        0.40f, 0.45f, 0.45f, 10.0f,
        0.40f, 0.45f, 0.45f, 10.0f,
        0.40f, 0.45f, 0.45f, 10.0f,
        0.40f, 0.45f, 0.45f, 10.0f
#endif
    }};
#ifdef S10_K8_ARM
#ifdef S10_K8_RECOVERY
    const std::array<float, 6> arm_default_{{
        0.0f, -2.45f, 1.09f, 0.0f, 0.82f, kK8Joint6TargetRad
    }};
#else
    const std::array<float, 6> arm_default_{{
        0.0f, -2.45f, 1.09f, 0.0f, 0.82f, -1.08f
    }};
#endif
    const std::array<float, 6> arm_lower_{{
        -2.525f, -3.192f, 0.0f, -2.014f, -1.644f, -2.685f
    }};
    const std::array<float, 6> arm_upper_{{
        3.216f, 0.0f, 3.161f, 2.784f, 1.57f, 2.754f
    }};
#else
    const std::array<float, 6> arm_default_{{
        0.0f, 2.54f, -1.26f, 0.0f, -1.22f, 0.0f
    }};
    const std::array<float, 6> arm_lower_{{
        -2.618f, 0.0f, -2.967f, -1.745f, -1.22f, -2.0944f
    }};
    const std::array<float, 6> arm_upper_{{
        2.618f, 3.14f, 0.0f, 1.745f, 1.22f, 2.0944f
    }};
#endif

    static bool IsWheel(int i) {
        return i == 3 || i == 7 || i == 11 || i == 15;
    }

#ifdef S10_V26_NEGATIVE_YAW_MIRROR
    static int MirrorDogJointIndex(int index) {
        if (index < 4) return index + 4;
        if (index < 8) return index - 4;
        if (index < 12) return index + 4;
        return index - 4;
    }

    static float MirrorJointSign(int index) {
        if (index < 16) {
            // Sagittal reflection: hip-X changes sign; hip-Y, knee and wheel
            // retain their generalized-coordinate signs.
            return index % 4 == 0 ? -1.0f : 1.0f;
        }
        const int arm_joint = index - 16;
        return (arm_joint == 0 || arm_joint == 3 || arm_joint == 5)
            ? -1.0f : 1.0f;
    }

    static VecXf MirrorJointVector(const VecXf& source) {
        VecXf mirrored = source;
        if (source.size() < kRobotDof) return mirrored;
        for (int index = 0; index < 16; ++index) {
            mirrored(index) = MirrorJointSign(index) *
                source(MirrorDogJointIndex(index));
        }
        for (int index = 16; index < kRobotDof; ++index) {
            mirrored(index) = MirrorJointSign(index) * source(index);
        }
        return mirrored;
    }

    static RobotBasicState MirrorRobotState(const RobotBasicState& source) {
        RobotBasicState mirrored = source;
        Mat3f reflection = Mat3f::Identity();
        reflection(1, 1) = -1.0f;
        mirrored.base_rot_mat = reflection * source.base_rot_mat * reflection;
        mirrored.base_rpy = Vec3f(
            -source.base_rpy.x(), source.base_rpy.y(), -source.base_rpy.z());
        mirrored.base_omega = Vec3f(
            -source.base_omega.x(), source.base_omega.y(),
            -source.base_omega.z());
        mirrored.base_acc = reflection * source.base_acc;
        mirrored.projected_gravity = reflection * source.projected_gravity;
        mirrored.cmd_vel_normlized = Vec3f(
            source.cmd_vel_normlized.x(), -source.cmd_vel_normlized.y(),
            -source.cmd_vel_normlized.z());
        mirrored.joint_pos = MirrorJointVector(source.joint_pos);
        mirrored.joint_vel = MirrorJointVector(source.joint_vel);
        mirrored.joint_tau = MirrorJointVector(source.joint_tau);
        return mirrored;
    }

    bool ShouldMirrorNegativeYaw() const {
        return velocity_command_.z() < -0.10f &&
            std::abs(velocity_command_.x()) <= 1.0e-4f &&
            std::abs(velocity_command_.y()) <= 1.0e-4f;
    }
#endif

    bool IsPureXHomeMode() const {
#if defined(S10_K8_ARM) || defined(S10_PIPER_V23) || defined(S10_PIPER_V24) || defined(S10_PIPER_V25) || defined(S10_PIPER_LOW035)
        // The S10+K8 policy is trained with the K8-best full-RL contract.
        return false;
#else
        const bool pure_x =
            std::abs(velocity_command_(0)) > 0.20f &&
            std::abs(velocity_command_(1)) <= 0.20f &&
            std::abs(velocity_command_(2)) <= 0.10f;
        const Vec3f home(0.42f, 0.0f, 0.05f);
        return pure_x && (arm_command_ - home).norm() <= 0.02f;
#endif
    }

    static Eigen::Vector2f RollPitch(const Mat3f& rot) {
        const float roll = std::atan2(rot(2, 1), rot(2, 2));
        const float pitch = std::atan2(
            -rot(2, 0),
            std::sqrt(rot(0, 0) * rot(0, 0) + rot(1, 0) * rot(1, 0)));
        return {roll, pitch};
    }

    void InitializeAction() {
        action_.goal_joint_pos = VecXf::Zero(kRobotDof);
        action_.goal_joint_vel = VecXf::Zero(kRobotDof);
        action_.kp = VecXf::Zero(kRobotDof);
        action_.kd = VecXf::Zero(kRobotDof);
        action_.tau_ff = VecXf::Zero(kRobotDof);

        for (int leg = 0; leg < 4; ++leg) {
            const int base = leg * 4;
            action_.kp.segment<4>(base) << 60.0f, 60.0f, 60.0f, 0.0f;
            action_.kd.segment<4>(base) << 1.0f, 1.0f, 1.0f, 0.4f;
            for (int j = 0; j < 4; ++j) {
                action_.goal_joint_pos(base + j) = dog_default_[base + j];
            }
        }

#ifdef S10_K8_ARM
        const std::array<float, 6> arm_kp{{200.0f, 200.0f, 200.0f, 200.0f, 200.0f, 100.0f}};
        const std::array<float, 6> arm_kd{{6.0f, 6.0f, 6.0f, 6.0f, 6.0f, 4.0f}};
#else
        const std::array<float, 6> arm_kp{{50.0f, 60.0f, 80.0f, 30.0f, 30.0f, 20.0f}};
        const std::array<float, 6> arm_kd{{6.0f, 6.0f, 8.0f, 4.0f, 4.0f, 2.0f}};
#endif
        for (int i = 0; i < 6; ++i) {
#ifdef S10_K8_ARM
            const float initial_target = i == 5 ? kK8Joint6TargetRad : arm_default_[i];
#else
            const float initial_target = arm_default_[i];
#endif
            arm_target_(i) = initial_target;
            action_.goal_joint_pos(16 + i) = initial_target;
            action_.kp(16 + i) = arm_kp[i];
            action_.kd(16 + i) = arm_kd[i];
        }
    }

    void LoadModel() {
        std::filesystem::path model_path;
        if (const char* override_path = std::getenv("S10_POLICY_PATH");
            override_path && *override_path) {
            model_path = override_path;
        } else {
            model_path = std::filesystem::path(__FILE__).parent_path().parent_path() /
                         "policy" / "ppo" / S10_POLICY_FILE;
        }
        if (!std::filesystem::exists(model_path)) {
            std::cerr << "[S10/DWBC] ONNX not found: " << model_path << "\n"
                      << "[S10/DWBC] Idle/StandUp remain usable; RL mode holds "
                      << "the training default until the policy is installed.\n";
            return;
        }

        session_options_.SetIntraOpNumThreads(1);
        session_options_.SetGraphOptimizationLevel(
            GraphOptimizationLevel::ORT_ENABLE_ALL);
        session_ = std::make_unique<Ort::Session>(
            env_, model_path.c_str(), session_options_);

        const auto input_shape = session_->GetInputTypeInfo(0)
            .GetTensorTypeAndShapeInfo().GetShape();
        const auto output_shape = session_->GetOutputTypeInfo(0)
            .GetTensorTypeAndShapeInfo().GetShape();
        if (input_shape.size() != 2 ||
            (input_shape[1] > 0 && input_shape[1] != kInputDim) ||
            output_shape.size() != 2 ||
            (output_shape[1] > 0 && output_shape[1] != kActionDim)) {
            std::cerr << "[S10/DWBC] Invalid ONNX dimensions; expected [N,836] -> [N,22].\n";
            session_.reset();
            return;
        }
        model_ready_ = true;
        std::cout << "[S10/DWBC] Loaded ONNX: " << model_path << "\n";
    }

    void BuildObservation(const RobotBasicState& state) {
        current_obs_.setZero();
        int offset = 0;
        current_obs_.segment<2>(offset) = RollPitch(state.base_rot_mat);
        offset += 2;
        current_obs_.segment<3>(offset) = state.base_omega;
        offset += 3;

        // Training stores arm position first, followed by 12 non-wheel joints.
        for (int i = 0; i < 6; ++i) {
#ifdef S10_K8_ARM
            // Deployed joint6 is fixed at 0 rad. Treat that pose as the
            // policy's neutral wrist pose so the old -1.08 rad training
            // reference does not look like a new arm command on C entry.
            const float reference = i == 5 ? kK8Joint6TargetRad : arm_default_[i];
#else
            const float reference = arm_default_[i];
#endif
            current_obs_(offset++) = state.joint_pos(16 + i) - reference;
        }
        for (int i = 0; i < 16; ++i) {
            if (!IsWheel(i)) {
                current_obs_(offset++) = state.joint_pos(i) - dog_default_[i];
            }
        }

        // Velocity order is arm6 followed by all 16 S10 joints.
        for (int i = 0; i < 6; ++i) {
            current_obs_(offset++) = state.joint_vel(16 + i) * 0.05f;
        }
        for (int i = 0; i < 16; ++i) {
            current_obs_(offset++) = state.joint_vel(i) * 0.05f;
        }

        current_obs_.segment(offset, kActionDim) = last_raw_action_;
        offset += kActionDim;
        current_obs_.segment<3>(offset) = velocity_command_;
        offset += 3;
        current_obs_.segment<3>(offset) = arm_command_;
        offset += 3;
        current_obs_.segment<3>(offset).setZero();
        offset += 3;

        if (offset != kSingleObs) {
            throw std::runtime_error("S10 observation layout is not 76 values");
        }
        for (int i = 0; i < kSingleObs; ++i) {
            if (!std::isfinite(current_obs_(i))) current_obs_(i) = 0.0f;
            current_obs_(i) = std::clamp(current_obs_(i), -100.0f, 100.0f);
        }
    }

    void InitializeHistoryIfNeeded() {
        if (!history_.empty()) return;
        for (int i = 0; i < kHistoryLen; ++i) history_.push_back(current_obs_);
    }

    VecXf Infer() {
        if (!model_ready_) return VecXf::Zero(kActionDim);
        input_obs_.segment(0, kSingleObs) = current_obs_;
        int offset = kSingleObs;
        for (const auto& obs : history_) {
            input_obs_.segment(offset, kSingleObs) = obs;
            offset += kSingleObs;
        }

        std::array<int64_t, 2> shape{{1, kInputDim}};
        Ort::Value input = Ort::Value::CreateTensor<float>(
            memory_info_, input_obs_.data(), input_obs_.size(),
            shape.data(), shape.size());
        std::array<Ort::Value, 1> inputs{{std::move(input)}};
        auto outputs = session_->Run(
            Ort::RunOptions{nullptr}, input_names_.data(), inputs.data(), 1,
            output_names_.data(), 1);

        const float* output = outputs.front().GetTensorData<float>();
        VecXf result(kActionDim);
        for (int i = 0; i < kActionDim; ++i) {
            result(i) = std::isfinite(output[i])
                ? std::clamp(output[i], -100.0f, 100.0f) : 0.0f;
        }
        return result;
    }

    float ArmRawTarget(int i, float normalized_action) const {
#if defined(S10_K8_ARM) && !defined(S10_K8_RECOVERY)
        if (i == 5) return kK8Joint6TargetRad;
#endif
        const float a = std::clamp(normalized_action, -1.0f, 1.0f);
        const float safe_lower = arm_default_[i] +
            0.95f * (arm_lower_[i] - arm_default_[i]);
        const float safe_upper = arm_default_[i] +
            0.95f * (arm_upper_[i] - arm_default_[i]);
        float target = a >= 0.0f
            ? arm_default_[i] + a * (safe_upper - arm_default_[i])
            : arm_default_[i] + a * (arm_default_[i] - safe_lower);
#ifdef S10_K8_RECOVERY
        if (i == 3 || i == 4) {
            target = std::clamp(
                arm_default_[i] + a, safe_lower, safe_upper);
        }
#else
        if (i == 3 || i == 5) target = arm_default_[i];
        if (i == 4) {
            target = std::clamp(
                arm_default_[i] + 0.5f * a, safe_lower, safe_upper);
        }
#endif
        return std::clamp(target, safe_lower, safe_upper);
    }

#ifdef S10_K8_RECOVERY
    void SyncTargetsToImpactPose(const RobotBasicState& state) {
        // Isaac Gym initializes the policy filters at the measured impact
        // pose. Do the same when C is entered after a free fall in MuJoCo.
        for (int i = 0; i < 16; ++i) {
            if (IsWheel(i)) {
                filtered_leg_action_(i) = 0.0f;
                continue;
            }
            filtered_leg_action_(i) = std::clamp(
                (state.joint_pos(i) - dog_default_[i]) / dog_action_scale_[i],
                -1.0f, 1.0f);
            action_.goal_joint_pos(i) = dog_default_[i] +
                dog_action_scale_[i] * filtered_leg_action_(i);
        }
        for (int i = 0; i < 6; ++i) {
            arm_target_(i) = std::clamp(
                state.joint_pos(16 + i), arm_lower_[i], arm_upper_[i]);
            action_.goal_joint_pos(16 + i) = arm_target_(i);
        }
    }
#endif

    void DecodeAction(const VecXf& raw_action) {
        const VecXf previous_leg_action = filtered_leg_action_;
        VecXf candidate_leg_action = previous_leg_action + kLegWheelEmaAlpha *
            (raw_action.head(16) - previous_leg_action);

        // Match training's standstill-only 0.003 rad/policy-step limiter.
        // Non-zero X/Y/yaw commands retain full leg authority; wheels are
        // velocity controlled and are never rate-limited by this block.
        const bool walking =
            std::abs(velocity_command_(0)) > 0.20f ||
            std::abs(velocity_command_(1)) > 0.20f ||
            std::abs(velocity_command_(2)) > 0.10f;
#ifdef S10_PIPER_V23_STABLE
        for (int i = 0; i < 16; ++i) {
            if (IsWheel(i)) continue;
            const float previous = previous_leg_action(i);
            smoothed_leg_action_(i) += kStableLegEmaAlpha *
                (raw_action(i) - smoothed_leg_action_(i));
            const float desired = smoothed_leg_action_(i);
            const float physical_delta =
                dog_action_scale_[i] * (desired - previous);
            if (std::abs(physical_delta) <= kStableLegDeadbandRad) {
                candidate_leg_action(i) = previous;
                continue;
            }
            const float max_delta = walking
                ? kStableLegMovingMaxDeltaRad
                : kStableLegStandingMaxDeltaRad;
            candidate_leg_action(i) = previous + std::clamp(
                physical_delta, -max_delta, max_delta) /
                dog_action_scale_[i];
        }
#else
        if (kStandingLegMaxDeltaRad > 0.0f &&
            (!walking || kStandingLegMaxDeltaRad >= 0.08f)) {
            for (int i = 0; i < 16; ++i) {
                if (IsWheel(i)) continue;
                const float normalized_limit =
                    kStandingLegMaxDeltaRad / dog_action_scale_[i];
                candidate_leg_action(i) = std::clamp(
                    candidate_leg_action(i),
                    previous_leg_action(i) - normalized_limit,
                    previous_leg_action(i) + normalized_limit);
            }
        }
#endif
        filtered_leg_action_ = candidate_leg_action;

        // The V16 checkpoint still emits large leg residuals for pure X,
        // despite the wheel-only shaping reward.  Enforce the intended
        // deployment contract only for neutral-arm forward/reverse driving:
        // all 12 position joints stay at the training default while the four
        // wheel actions remain fully RL-controlled.  Yaw and every non-home
        // whole-body arm posture bypass this guard and retain full leg output.
        pure_x_leg_hold_active_ = IsPureXHomeMode();
        if (pure_x_leg_hold_active_) {
            for (int i = 0; i < 16; ++i) {
                if (!IsWheel(i)) filtered_leg_action_(i) = 0.0f;
            }
        }

        for (int leg = 0; leg < 4; ++leg) {
            const int base = leg * 4;
            for (int j = 0; j < 3; ++j) {
                action_.goal_joint_pos(base + j) =
                    dog_default_[base + j] +
                    dog_action_scale_[base + j] * filtered_leg_action_(base + j);
                action_.goal_joint_vel(base + j) = 0.0f;
            }
            action_.goal_joint_pos(base + 3) = 0.0f;
            action_.goal_joint_vel(base + 3) =
                dog_action_scale_[base + 3] * filtered_leg_action_(base + 3);
        }

        for (int i = 0; i < 6; ++i) {
#ifdef S10_K8_ARM
            if (arm_target_hold_) {
                action_.goal_joint_pos(16 + i) = arm_target_(i);
                action_.goal_joint_vel(16 + i) = 0.0f;
                continue;
            }
#endif
            const float desired = ArmRawTarget(i, raw_action(16 + i));
#ifdef S10_K8_ARM
            if (std::abs(desired - arm_target_(i)) <=
                kArmTargetDeadbandRad) {
                action_.goal_joint_pos(16 + i) = arm_target_(i);
                action_.goal_joint_vel(16 + i) = 0.0f;
                continue;
            }
#endif
            const float candidate = arm_target_(i) +
                kArmEmaAlpha * (desired - arm_target_(i));
            arm_target_(i) += std::clamp(
                candidate - arm_target_(i),
                -kArmMaxDeltaRad, kArmMaxDeltaRad);
#if defined(S10_K8_ARM) && !defined(S10_K8_RECOVERY)
            if (i == 5) arm_target_(i) = kK8Joint6TargetRad;
#elif !defined(S10_K8_RECOVERY)
            if (i == 5) arm_target_(i) = arm_default_[i];
#endif
            action_.goal_joint_pos(16 + i) = arm_target_(i);
            action_.goal_joint_vel(16 + i) = 0.0f;
        }
    }

public:
    explicit DwbcS10PiperPolicyRunnerONNX(const std::string& policy_name)
        : PolicyRunnerBase(policy_name),
          env_(ORT_LOGGING_LEVEL_WARNING, "dwbcS10"),
          memory_info_(Ort::MemoryInfo::CreateCpu(
              OrtArenaAllocator, OrtMemTypeDefault)) {
        decimation_ = 4;  // MuJoCo 200 Hz, policy 50 Hz.
        InitializeAction();
        LoadModel();
    }

    void SetVelocityCommand(float vx, float vy, float wz) {
        (void)vy;
#ifdef S10_K8_RECOVERY
        velocity_command_ <<
#ifdef S10_K8_RECOVERY_V9
            std::clamp(vx, -1.0f, 1.0f),
#else
            std::clamp(vx, -0.5f, 0.5f),
#endif
            0.0f,
            std::clamp(wz, -0.6f, 0.6f);
#else
        velocity_command_ <<
            std::clamp(vx, -0.8f, 0.8f),
            0.0f,
            std::clamp(wz, -1.0f, 1.0f);
#endif
    }

    void SetArmCommand(float x, float y, float z) {
#ifdef S10_K8_RECOVERY
        (void)x; (void)y; (void)z;
        arm_command_ = piper_safe_workspace::Home();
#else
        // This value is used both by the actor observation and by the Orin
        // link6 command, so the policy never sees an unreachable raw target.
        arm_command_ = piper_safe_workspace::Clamp(Vec3f(x, y, z));
#endif
    }

    // Read-only view of the same clamped task passed to the actor.
    // Orin uses link6 training Home as its fixed zero-offset reference.
    Vec3f GetArmPositionOffset() const {
        return arm_command_ - piper_safe_workspace::Home();
    }

    void SetHomeResetCommand(bool requested) {
        if (requested) arm_command_ = piper_safe_workspace::Home();
    }

    void DisplayPolicyInfo() override {
        std::cout << policy_name_
                  << ": 836 obs, 22 actions, 50 Hz, S10 gains 60/1, wheel Kd 0.4; "
                  << "Lite3 leg scales 0.40/0.45/0.45; "
#ifdef S10_K8_ARM
#ifdef S10_K8_RECOVERY
                  << "K8 recovery: free wrist, arm EMA 0.5/0.08 rad, "
                  << "leg EMA 0.4/0.08 rad, no tilt exit; "
#elif defined(S10_K8_LOW035)
                  << "K8 low035 V3: full RL, arm EMA 0.2/0.02 rad, "
                  << "leg EMA 0.2/0.006 rad standing limit; "
                  << "joint6=-1.08, no stationary arm latch; "
#else
                  << "K8 full-RL pure-X; K8 arm EMA 0.05, 0.006 rad/step, "
                  << "0.005 rad deadband, 4 s stationary target hold; "
                  << "joint6=0, C holds Z posture until user motion; "
#endif
#else
#ifdef S10_PIPER_LOW035
                  << "Piper low035: training-matched neutral hip=0.726924 rad, full-RL legs; "
#endif
#ifdef S10_PIPER_V25
                  << "Piper V25 full-body control; standstill leg target "
                  << "limit 0.006 rad/step; "
#elif defined(S10_PIPER_V24)
                  << "Piper V24 full-body control; standstill leg target "
                  << "limit 0.006 rad/step (training matched); "
#elif defined(S10_PIPER_LOW035)
                  << "standstill leg target limit 0.003 rad/step; "
#elif defined(S10_PIPER_V23)
#ifdef S10_PIPER_V23_LEG_HOLD
                  << "Piper V23: 12 leg joints held at C-entry measured pose; "
                  << "wheels and Piper arm remain policy-controlled; "
#else
#ifdef S10_PIPER_V23_STABLE
                  << "Piper V23 stable legs: EMA 0.08, 0.002 rad deadband, "
                  << "0.006/0.020 rad step limit (standing/moving); "
#else
                  << "Piper V23 full-body control; no standstill leg rate limit; "
#endif
#endif
#else
                  << "pure-X Home keeps 12 leg joints fixed; "
#endif
                  << "Piper EMA 0.2 and 0.02 rad/step; "
#endif
#ifdef S10_PIPER_SAFE_LOW_GOAL
                  << "low EE goal z >= -0.27 m; "
#endif
                  << "S10/Piper operator workspace X[-0.42,0.58] Y[-0.50,0.50] "
                  << "Z[-0.30,0.42] m; constrained J1-J3 IK -> MOVE_J, J4-J6 locked; policy task Z only; "
                  << "gravity compensation OFF; "
                  << (model_ready_ ? "ONNX ready" : "safe default hold (ONNX missing)")
                  << "\n";
    }

    void OnEnter() override {
        run_cnt_ = 0;
        history_.clear();
        last_raw_action_.setZero();
        filtered_leg_action_.setZero();
#ifdef S10_PIPER_V23_LEG_HOLD
        leg_hold_initialized_ = false;
#endif
#ifdef S10_PIPER_V23_STABLE
        smoothed_leg_action_.setZero();
#endif
        pure_x_leg_hold_active_ = false;
#ifdef S10_V26_NEGATIVE_YAW_MIRROR
        yaw_mirror_initialized_ = false;
        yaw_mirror_active_ = false;
#endif
        velocity_command_.setZero();
        arm_command_ = piper_safe_workspace::Home();
#ifdef S10_K8_ARM
        previous_arm_command_ = arm_command_;
        arm_settle_steps_ = 0;
        arm_target_hold_ = false;
        hold_initial_home_ = true;
#if defined(S10_K8_RECOVERY) || defined(S10_K8_LOW035)
        hold_initial_home_ = false;
#endif
#endif
        InitializeAction();
        DisplayPolicyInfo();
    }

    RobotAction GetRobotAction(const RobotBasicState& state) override {
        if (state.joint_pos.size() < kRobotDof ||
            state.joint_vel.size() < kRobotDof) {
            return action_;
        }
#ifdef S10_PIPER_V23_LEG_HOLD
        if (!leg_hold_initialized_) {
            for (int i = 0; i < 16; ++i) {
                if (!IsWheel(i)) locked_leg_pos_(i) = state.joint_pos(i);
            }
            leg_hold_initialized_ = true;
        }
#endif
        if (run_cnt_ % decimation_ == 0) {
#ifdef S10_K8_RECOVERY
            if (run_cnt_ == 0) SyncTargetsToImpactPose(state);
#endif
#ifdef S10_K8_ARM
            const Vec3f home(0.42f, 0.0f, 0.05f);
            if (velocity_command_.norm() > 1.0e-4f ||
                (arm_command_ - home).norm() > 1.0e-5f) {
                hold_initial_home_ = false;
            }
            if (velocity_command_.norm() > 1.0e-4f ||
                (arm_command_ - previous_arm_command_).norm() > 1.0e-5f) {
                arm_settle_steps_ = 0;
                arm_target_hold_ = false;
#if !defined(S10_K8_RECOVERY) && !defined(S10_K8_LOW035)
            } else if (!arm_target_hold_ &&
                       ++arm_settle_steps_ >= kArmSettleSteps) {
                arm_target_hold_ = true;
#endif
            }
            previous_arm_command_ = arm_command_;
#endif
#ifdef S10_V26_NEGATIVE_YAW_MIRROR
            const bool mirror_yaw = ShouldMirrorNegativeYaw();
            if (!yaw_mirror_initialized_ || mirror_yaw != yaw_mirror_active_) {
                yaw_mirror_initialized_ = true;
                yaw_mirror_active_ = mirror_yaw;
                history_.clear();
                std::cout << "[V26 yawfix] E negative-yaw mirror "
                          << (mirror_yaw ? "ON" : "OFF")
                          << "; observation history rebuilt.\n";
            }
            if (mirror_yaw) {
                const RobotBasicState mirrored_state = MirrorRobotState(state);
                const Vec3f physical_velocity = velocity_command_;
                const Vec3f physical_arm = arm_command_;
                const VecXf physical_last_action = last_raw_action_;
                velocity_command_ = Vec3f(
                    physical_velocity.x(), -physical_velocity.y(),
                    -physical_velocity.z());
                arm_command_ = Vec3f(
                    physical_arm.x(), -physical_arm.y(), physical_arm.z());
                last_raw_action_ = MirrorJointVector(physical_last_action);
                BuildObservation(mirrored_state);
                velocity_command_ = physical_velocity;
                arm_command_ = physical_arm;
                last_raw_action_ = physical_last_action;
            } else {
                BuildObservation(state);
            }
#else
            BuildObservation(state);
#endif
            InitializeHistoryIfNeeded();
#ifdef S10_K8_ARM
            if (hold_initial_home_) {
                // Z and the first C tick command the same posture. Keep it
                // until the operator requests motion, while filling history.
                last_raw_action_.setZero();
                history_.pop_front();
                history_.push_back(current_obs_);
                ++run_cnt_;
                return action_;
            }
#endif
            VecXf raw_action = Infer();
#ifdef S10_V26_NEGATIVE_YAW_MIRROR
            if (mirror_yaw) raw_action = MirrorJointVector(raw_action);
#endif
            DecodeAction(raw_action);
            last_raw_action_ = raw_action;
#ifdef S10_PIPER_V23_LEG_HOLD
            for (int i = 0; i < 16; ++i) {
                if (IsWheel(i)) continue;
                action_.goal_joint_pos(i) = locked_leg_pos_(i);
                action_.goal_joint_vel(i) = 0.0f;
                last_raw_action_(i) = std::clamp(
                    (locked_leg_pos_(i) - dog_default_[i]) /
                        dog_action_scale_[i], -1.0f, 1.0f);
            }
#endif
            if (pure_x_leg_hold_active_) {
                for (int i = 0; i < 16; ++i) {
                    if (!IsWheel(i)) last_raw_action_(i) = 0.0f;
                }
            }
            history_.pop_front();
            history_.push_back(current_obs_);
        }
        ++run_cnt_;
        return action_;
    }
};
