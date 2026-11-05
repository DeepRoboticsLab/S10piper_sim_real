/**
 * @file standup_state.hpp
 * @brief Smooth stand-up state for 22-DOF S10+Piper control.
 */
#pragma once

#include "state_base.h"
#include <algorithm>

class StandUpState : public StateBase{
private:
    VecXf init_joint_pos_, init_joint_vel_, current_joint_pos_, current_joint_vel_;
    float time_stamp_record_, run_time_;
    VecXf goal_joint_pos_, policy_default_pos_, idle_kp_, idle_kd_, kp_, kd_;
    MatXf joint_cmd_;
    float stand_duration_ = 2.;
    
    // 机械臂的目标位置
    VecXf arm_target_pos_;

    void GetRobotJointValue(){
        current_joint_pos_ = ri_ptr_->GetJointPosition();
        current_joint_vel_ = ri_ptr_->GetJointVelocity();
        run_time_ = ri_ptr_->GetInterfaceTimeStamp();
    }

    void RecordJointData(){
        init_joint_pos_ = current_joint_pos_;
        init_joint_vel_ = current_joint_vel_;
        time_stamp_record_ = run_time_;
    }

    float GetCubicSplinePos(float x0, float v0, float xf, float vf, float t, float T){
        if(t >= T) return xf;
        float a, b, c, d;
        d = x0; c = v0;
        a = (vf*T - 2*xf + v0*T + 2*x0) / pow(T, 3);
        b = (3*xf - vf*T - 2*v0*T - 3*x0) / pow(T, 2);
        return a*pow(t, 3)+b*pow(t, 2)+c*t+d;
    }
    float GetCubicSplineVel(float x0, float v0, float xf, float vf, float t, float T){
        if(t >= T) return 0;
        float a, b, c;
        c = v0;
        a = (vf*T - 2*xf + v0*T + 2*x0) / pow(T, 3);
        b = (3*xf - vf*T - 2*v0*T - 3*x0) / pow(T, 2);
        return 3.*a*pow(t, 2) + 2.*b*t + c;
    }

    float GetHipYPosByHeight(float h){
        float l1 = cp_ptr_->thigh_len_;
        float l2 = cp_ptr_->shank_len_;
        if(fabs(h) >= l1 + l2) return 0;
        float theta = -acos((l1*l1+h*h-l2*l2)/(2.*h*l1));
        theta = LimitNumber(theta, cp_ptr_->fl_joint_lower_(1), cp_ptr_->fl_joint_upper_(1));
        return theta;
    }

    float GetKneePosByHeight(float h){
        float l1 = cp_ptr_->thigh_len_;
        float l2 = cp_ptr_->shank_len_;
        if(fabs(h) >= l1 + l2) return 0;
        float theta = M_PI-acos((l1*l1+l2*l2-h*h)/(2*l1*l2));
        theta = LimitNumber(theta, cp_ptr_->fl_joint_lower_(2), cp_ptr_->fl_joint_upper_(2));
        return theta;
    }

public:
    StandUpState(const RobotType& robot_type, const std::string& state_name, 
        std::shared_ptr<ControllerData> data_ptr):StateBase(robot_type, state_name, data_ptr){
            
            goal_joint_pos_ = VecXf::Zero(22);
            policy_default_pos_ = VecXf::Zero(22);
            
            // Stand up to the deployment target used on the first C tick.
            arm_target_pos_.resize(6);
#ifdef S10_K8_ARM
#ifdef S10_K8_LOW035
            float arm_def[] = {0.0f, -2.45f, 1.09f, 0.0f, 0.82f, -1.08f};
#else
            float arm_def[] = {0.0f, -2.45f, 1.09f, 0.0f, 0.82f, 0.0f};
#endif
#else
            float arm_def[] = {0.0f, 2.54f, -1.26f, 0.0f, -1.22f, 0.0f};
#endif
            for(int i=0; i<6; ++i) {
                arm_target_pos_(i) = arm_def[i];
            }

            kp_ = VecXf::Zero(22);
            kd_ = VecXf::Zero(22);     
            idle_kp_ = VecXf::Zero(22);
            idle_kd_ = VecXf::Zero(22);
            
            // Match the dwbcS10piper training gains. Wheels remain velocity-damped,
            // not position locked, so switching to C does not change dynamics.
            Eigen::Vector4f stand_kp(60.0f, 60.0f, 60.0f, 0.0f);
            Eigen::Vector4f stand_kd(1.0f, 1.0f, 1.0f, 0.4f);
            kp_.head(16) = stand_kp.replicate(4, 1);
            kd_.head(16) = stand_kd.replicate(4, 1);
            Eigen::Vector4f idle_leg_kp = Eigen::Vector4f::Zero();
            Eigen::Vector4f idle_leg_kd = Eigen::Vector4f::Zero();
            idle_kp_.head(16) = idle_leg_kp.replicate(4, 1);
            idle_kd_.head(16) = idle_leg_kd.replicate(4, 1);

#ifdef S10_K8_ARM
            float arm_kp[] = {200.0f, 200.0f, 200.0f, 200.0f, 200.0f, 100.0f};
            float arm_kd[] = {6.0f, 6.0f, 6.0f, 6.0f, 6.0f, 4.0f};
#else
            float arm_kp[] = {50.0f, 60.0f, 80.0f, 30.0f, 30.0f, 20.0f};
            float arm_kd[] = {6.0f, 6.0f, 8.0f, 4.0f, 4.0f, 2.0f};
#endif
            for (int i = 0; i < 6; ++i) {
                kp_(16 + i) = arm_kp[i];
                kd_(16 + i) = arm_kd[i];
            }
            for (int i = 0; i < 6; ++i) {
                idle_kp_(16 + i) = 0.0f;
                idle_kd_(16 + i) = 0.0f;
            }

            joint_cmd_ = MatXf::Zero(22, 5);
            stand_duration_ = cp_ptr_->stand_duration_;
            
            // 初始化第一阶段的目标姿态
            float init_hipy = GetHipYPosByHeight(cp_ptr_->pre_height_);
            float init_knee = GetKneePosByHeight(cp_ptr_->pre_height_);
            
            // 前后腿镜像
            Eigen::Vector4f front_leg_goal(0., init_hipy, init_knee, 0.);
            Eigen::Vector4f hind_leg_goal(0., -init_hipy, -init_knee, 0.); 
            
            goal_joint_pos_.segment(0, 4) = front_leg_goal;
            goal_joint_pos_.segment(4, 4) = front_leg_goal;
            goal_joint_pos_.segment(8, 4) = hind_leg_goal;
            goal_joint_pos_.segment(12, 4) = hind_leg_goal;
            goal_joint_pos_.tail(6) = arm_target_pos_;

            // Match the checkpoint's neutral reference before entering C.
#ifdef S10_PIPER_LOW035
            constexpr float policy_hip = 0.7269239136f;
#else
            constexpr float policy_hip = 0.45f;
#endif
            Eigen::Vector4f fl_policy( 0.08f, -policy_hip,  2 * policy_hip, 0.0f);
            Eigen::Vector4f fr_policy(-0.08f, -policy_hip,  2 * policy_hip, 0.0f);
            Eigen::Vector4f hl_policy( 0.08f,  policy_hip, -2 * policy_hip, 0.0f);
            Eigen::Vector4f hr_policy(-0.08f,  policy_hip, -2 * policy_hip, 0.0f);
            policy_default_pos_.segment(0, 4) = fl_policy;
            policy_default_pos_.segment(4, 4) = fr_policy;
            policy_default_pos_.segment(8, 4) = hl_policy;
            policy_default_pos_.segment(12, 4) = hr_policy;
            policy_default_pos_.tail(6) = arm_target_pos_;
        }
    ~StandUpState(){}

    virtual void OnEnter() {
        GetRobotJointValue();
        RecordJointData();
        StateBase::msfb_.UpdateCurrentState(RobotMotionState::StandingUp);
        uc_ptr_->SetMotionStateFeedback(StateBase::msfb_);
        std::cout << "[StandUp] Z: ramp Kp/Kd from zero; Piper Home "
                     "command speed 0.8 rad/s (2x previous original)." << std::endl;
    };
    virtual void OnExit() {
        std::cout << "[StandUp] Exit." << std::endl;
    }
    
    virtual void Run() {
        GetRobotJointValue();
        
        VecXf planning_joint_pos(22); 
        VecXf planning_joint_vel(22); 
        planning_joint_pos.setZero();
        planning_joint_vel.setZero();

        // 阶段 1: 从当前姿态插值到预备姿态
        if(run_time_ - time_stamp_record_ <= stand_duration_){
            for(int i=0; i<22; ++i){ 
                planning_joint_pos(i) = GetCubicSplinePos(init_joint_pos_(i), init_joint_vel_(i), goal_joint_pos_(i), 0, 
                                                run_time_ - time_stamp_record_, stand_duration_);
                planning_joint_vel(i) = GetCubicSplineVel(init_joint_pos_(i), init_joint_vel_(i), goal_joint_pos_(i), 0, 
                                                run_time_ - time_stamp_record_, stand_duration_);
            }
        }
        // Stage 2: move to the deployment standby posture.
        else{
            float new_time = run_time_ - time_stamp_record_ - stand_duration_;
            for (int i = 0; i < 22; ++i) {
                planning_joint_pos(i) = GetCubicSplinePos(
                    goal_joint_pos_(i), 0.0f, policy_default_pos_(i), 0.0f,
                    new_time, stand_duration_);
                planning_joint_vel(i) = GetCubicSplineVel(
                    goal_joint_pos_(i), 0.0f, policy_default_pos_(i), 0.0f,
                    new_time, stand_duration_);
            }
        }

        // Only the arm trajectory duration is halved. The S10 leg trajectory
        // and gain-ramp duration stay unchanged; the arm output also retains
        // a bounded per-frame speed in the hardware adapter.
        const float arm_duration = 0.5f * stand_duration_;
        for (int i = 16; i < 22; ++i) {
            planning_joint_pos(i) = GetCubicSplinePos(
                init_joint_pos_(i), 0.0f, arm_target_pos_(i-16), 0.0f,
                run_time_ - time_stamp_record_, arm_duration);
            planning_joint_vel(i) = GetCubicSplineVel(
                init_joint_pos_(i), 0.0f, arm_target_pos_(i-16), 0.0f,
                run_time_ - time_stamp_record_, arm_duration);
        }

        // Wheel angle is accumulated rotation, not a standing pose target.
        for (int wheel : {3,7,11,15}) {
            planning_joint_pos(wheel)=0.0f;
            planning_joint_vel(wheel)=0.0f;
        }

        // 刚度平滑唤醒 (Cosine Blend)
        float phase = (run_time_ - time_stamp_record_) / stand_duration_;
        phase = std::clamp(phase, 0.0f, 1.0f);
        float blend = 0.5f * (1.0f - std::cos(phase * M_PI)); 

        // Start at the new zero-gain Idle; C retains normal RL gains.
        VecXf current_kp = idle_kp_ + (kp_ - idle_kp_) * blend;
        VecXf current_kd = idle_kd_ + (kd_ - idle_kd_) * blend;

        joint_cmd_.col(0) = current_kp;           
        joint_cmd_.col(1) = planning_joint_pos;   
        joint_cmd_.col(2) = current_kd;           
        joint_cmd_.col(3) = planning_joint_vel;   
        joint_cmd_.col(4).setZero();              

        ri_ptr_->SetJointCommand(joint_cmd_); 
    }

    virtual bool LoseControlJudge() {
        if(uc_ptr_->GetUserCommand().target_mode == int(RobotMotionState::JointDamping)) return true;
        return false;
    }

    virtual StateName GetNextStateName() {
        if(run_time_ - time_stamp_record_ <= 2.*stand_duration_){
            return StateName::kStandUp;
        }else{
            if(uc_ptr_->GetUserCommand().target_mode == int(RobotMotionState::RLControlMode)){
                if (!ri_ptr_->PiperAtTrainingHome()) {
                    return StateName::kStandUp;
                }
                std::cout << "stand up success -> switching to RL" << std::endl;
                return StateName::kRLControl;
            }
        }
        return StateName::kStandUp;
    }
};
