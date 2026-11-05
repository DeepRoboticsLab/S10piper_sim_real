#pragma once

#include "state_base.h"
#include "dwbc_s10_piper_policy_runner_onnx.hpp"
#include "piper_fault_isolation.hpp"
#include "layered_arm_targets.hpp"
#include <iostream>
#include <chrono>
#include <Eigen/Geometry> 

#ifndef S10_POLICY_NAME
#define S10_POLICY_NAME "policy"
#endif

class RLControlStateONNX : public StateBase {
private:
    std::shared_ptr<DwbcS10PiperPolicyRunnerONNX> policy_runner_ptr_;
    long long run_ticks_ = 0;
    using Clock = std::chrono::steady_clock;
    Clock::time_point timing_start_{};
    long long timing_control_frames_ = 0;
    long long timing_policy_frames_ = 0;
    long long timing_feedback_frames_ = 0;
    double timing_last_feedback_ = -1.0;
    PiperFaultIsolation piper_fault_isolation_;

public:
    RLControlStateONNX(RobotType robot_type, std::string state_name, std::shared_ptr<ControllerData> data_ptr)
        : StateBase(robot_type, state_name, data_ptr) {
        
        policy_runner_ptr_ = std::make_shared<DwbcS10PiperPolicyRunnerONNX>(
            S10_POLICY_NAME);
    }

    virtual void OnEnter() override {
        std::cout << "---------------------------------------" << std::endl;
        std::cout << "    Enter RL Control State (ONNX)      " << std::endl;
        std::cout << "---------------------------------------" << std::endl;
        policy_runner_ptr_->OnEnter();
        // C changes target ownership, not the arm's MOVE_J firmware mode.
        std::cout << "[Layered] Piper: constrained J1-J3 XYZ IK -> MOVE_J / JointCtrl; J4-J6 locked at training Home; "
                     "policy EE task: X/Y fixed at Home, Z only. "
                     "Measured arm q/dq remain in observation.\n";
        StateBase::msfb_.UpdateCurrentState(RobotMotionState::RLControlMode);
        data_ptr_->uc_ptr->SetMotionStateFeedback(StateBase::msfb_);
        run_ticks_ = 0;
        data_ptr_->ri_ptr->SetPiperPolicyControl(true);
        timing_start_ = Clock::now();
        timing_control_frames_ = timing_policy_frames_ = timing_feedback_frames_ = 0;
        timing_last_feedback_ = -1.0;
    }

    virtual void Run() override {
        RobotBasicState robot_state;

        // 1. 获取底层状态数据
        robot_state.joint_pos = data_ptr_->ri_ptr->GetJointPosition();
        robot_state.joint_vel = data_ptr_->ri_ptr->GetJointVelocity();
        
        // 🌟 核心修复1：补充读取关节力矩！给观测提供足底接触判断依据
        robot_state.joint_tau = data_ptr_->ri_ptr->GetJointTorque(); 

        // 🔥【致命错误拦截】检查是否收到了有效的底层数据
        if (robot_state.joint_pos.size() < 22) {
            if (run_ticks_ % 200 == 0) {
                std::cerr << "\033[1;31m[WARNING] 关节数据异常！当前维度: " 
                          << robot_state.joint_pos.size() 
                          << " (预期 22)。请检查 Python 仿真器或底层通讯是否正常运行！\033[0m" << std::endl;
            }
            data_ptr_->ri_ptr->SetJointCommand(MatXf::Zero(22, 5));
            run_ticks_++;
            return;
        }

        // 2. 获取 IMU 数据并转换为旋转矩阵
        Vec3f rpy = data_ptr_->ri_ptr->GetImuRpy();
        robot_state.base_rot_mat = (Eigen::AngleAxisf(rpy[2], Eigen::Vector3f::UnitZ()) *
                                    Eigen::AngleAxisf(rpy[1], Eigen::Vector3f::UnitY()) *
                                    Eigen::AngleAxisf(rpy[0], Eigen::Vector3f::UnitX())).toRotationMatrix();

        robot_state.base_omega = data_ptr_->ri_ptr->GetImuOmega();
        
        // 3. 桥接：获取并设置用户键盘指令
        UserCommand cmd = data_ptr_->uc_ptr->GetUserCommand();
        
        // 设置底盘移动指令 (VX, VY, WZ)。该策略的横移效果不可靠，
        // 状态机版本先固定 VY=0，只开放前后和原地转向。
        policy_runner_ptr_->SetVelocityCommand(
            cmd.forward_vel_scale, 0.0f, cmd.turnning_vel_scale);

        // Keep full XYZ for Piper, but vary ONLY Z in the policy EE task.
        const auto arm_targets = SplitLayeredArmTargets(
            Vec3f(cmd.arm_pos_des[0], cmd.arm_pos_des[1], cmd.arm_pos_des[2]),
            cmd.soft_stop_flag);
        policy_runner_ptr_->SetArmCommand(
            arm_targets.policy_xyz.x(), arm_targets.policy_xyz.y(),
            arm_targets.policy_xyz.z());
        policy_runner_ptr_->SetHomeResetCommand(cmd.soft_stop_flag);

        // 兼容旧框架逻辑
        robot_state.cmd_vel_normlized <<
            cmd.forward_vel_scale, 0.0f, cmd.turnning_vel_scale;

        // 4. 执行 ONNX 策略推理
        RobotAction action = policy_runner_ptr_->GetRobotAction(robot_state);

        // 5. 组装并发送动作
        MatXf cmd_mat = MatXf::Zero(22, 5); 
        cmd_mat.col(0) = action.kp;
        cmd_mat.col(1) = action.goal_joint_pos;
        cmd_mat.col(2) = action.kd;
        cmd_mat.col(3) = action.goal_joint_vel;
        cmd_mat.col(4) = action.tau_ff;

        data_ptr_->ri_ptr->SetJointCommand(cmd_mat);

        // Publish mandatory S10 outputs FIRST. Piper is an optional subsystem:
        // neither a gripper failure nor an IK/TCP write exception may unwind
        // the whole-body state machine. Hardware freshness/tilt faults are
        // deliberately outside this catch boundary and keep their protections.
        piper_fault_isolation_.Run([&] {
            data_ptr_->ri_ptr->SetPiperGripperCommand(cmd.piper_gripper_command);
            data_ptr_->ri_ptr->SetPiperCartesianCommand(
                arm_targets.piper_offset,
                cmd.piper_wrist_yaw_rad, cmd.soft_stop_flag);
        });

        ++timing_control_frames_;
        if (run_ticks_ % policy_runner_ptr_->decimation_ == 0) {
            ++timing_policy_frames_;
        }
        const double feedback_time = data_ptr_->ri_ptr->GetInterfaceTimeStamp();
        if (feedback_time != timing_last_feedback_) {
            ++timing_feedback_frames_;
            timing_last_feedback_ = feedback_time;
        }
        const float elapsed = std::chrono::duration<float>(
            Clock::now() - timing_start_).count();
        if (elapsed >= 2.0f) {
            std::cout << "[Timing] control=" << timing_control_frames_ / elapsed
                      << " Hz, policy=" << timing_policy_frames_ / elapsed
                      << " Hz, consumed DDS=" << timing_feedback_frames_ / elapsed
                      << " Hz\n";
            timing_start_ = Clock::now();
            timing_control_frames_ = timing_policy_frames_ = timing_feedback_frames_ = 0;
        }

        run_ticks_++;
    }

    virtual void OnExit() override {
        data_ptr_->ri_ptr->SetPiperPolicyControl(false);
        std::cout << "Exit RL Control State" << std::endl;
    }

    virtual StateName GetNextStateName() override {
        if (data_ptr_->uc_ptr->GetUserCommand().target_mode == int(RobotMotionState::JointDamping)) {
            std::cout << "[SAFE] Keyboard damping request -> joint_damping; "
                         "not a Piper timeout.\n";
            return StateName::kJointDamping;
        }
        return StateName::kRLControl; 
    }

    virtual bool LoseControlJudge() override {
        Vec3f rpy = data_ptr_->ri_ptr->GetImuRpy();
        if (std::abs(rpy[0]) > 1.0f || std::abs(rpy[1]) > 1.0f) {
            std::cout << "\033[1;31m[ERROR] Robot Tilted! Lose Control! "
                      << "roll=" << rpy[0] * 57.295779513f
                      << " deg, pitch=" << rpy[1] * 57.295779513f
                      << " deg -> joint_damping (not Piper timeout).\033[0m"
                      << std::endl;
            return true; 
        }
        return false; 
    }
};
