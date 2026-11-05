/**
 * @file idle_state.hpp
 * @brief robot need to confirm sensor input while in idle state
 */
#pragma once

#include "state_base.h"
#include <cmath>

class IdleState : public StateBase{
private:
    bool joint_normal_flag_ = false, imu_normal_flag_ = false;
    bool first_enter_flag_ = true;
    VecXf joint_pos_, joint_vel_, joint_tau_;
    Vec3f rpy_, acc_, omg_;
    double enter_state_time_ = -10000.;

    float last_print_time = 0;
    
    void GetProprioceptiveData(){
        joint_pos_ = ri_ptr_->GetJointPosition();
        joint_vel_ = ri_ptr_->GetJointVelocity();
        joint_tau_ = ri_ptr_->GetJointTorque();
        rpy_ = ri_ptr_->GetImuRpy();
        acc_ = ri_ptr_->GetImuAcc();
        omg_ = ri_ptr_->GetImuOmega();
    }

    bool JointDataNormalCheck(){
        VecXf joint_pos_lower(16), joint_pos_upper(16); 
        
        Eigen::Vector4f fl_lower = cp_ptr_->fl_joint_lower_;
        Eigen::Vector4f fl_upper = cp_ptr_->fl_joint_upper_;
        Eigen::Vector4f fr_lower(-fl_upper(0), fl_lower(1), fl_lower(2), fl_lower(3));
        Eigen::Vector4f fr_upper(-fl_lower(0), fl_upper(1), fl_upper(2), fl_upper(3));
        Eigen::Vector4f hl_lower(fl_lower(0), -fl_upper(1), -fl_upper(2), fl_lower(3));
        Eigen::Vector4f hl_upper(fl_upper(0), -fl_lower(1), -fl_lower(2), fl_upper(3));
        Eigen::Vector4f hr_lower(-fl_upper(0), -fl_upper(1), -fl_upper(2), fl_lower(3));
        Eigen::Vector4f hr_upper(-fl_lower(0), -fl_lower(1), -fl_lower(2), fl_upper(3));

        joint_pos_lower << fl_lower, fr_lower, hl_lower, hr_lower;
        joint_pos_upper << fl_upper, fr_upper, hl_upper, hr_upper;
        
        for(int i=0; i<16; ++i){ 
            if(std::isnan(joint_pos_lower(i)) || joint_pos_(i) > joint_pos_upper(i)+0.1 || joint_pos_(i) < joint_pos_lower(i)-0.1) {
                std::cout << "\033[1;31m[Safety Lock] Joint " << i << " Position Error! Val: " 
                          << joint_pos_(i) << " | Limit: [" << joint_pos_lower(i) << ", " << joint_pos_upper(i) << "]\033[0m\n";
                return false;
            }
            if(std::isnan(joint_vel_(i)) || std::abs(joint_vel_(i)) > cp_ptr_->joint_vel_limit_(i%4) + 0.1) {
                return false;
            }
        }
        return true;
    }

    bool ImuDataNormalCheck(){
        for(int i=0; i<3; ++i){
            if(std::isnan(rpy_(i)) || fabs(rpy_(i)) > M_PI) return false;
            if(std::isnan(omg_(i)) || fabs(omg_(i)) > M_PI) return false;
        }
        if(acc_.norm() < 0.1*gravity || acc_.norm() > 3.0*gravity) return false;
        return true;
    }

    void DisplayProprioceptiveInfo(){
        std::cout << "Joint Data: \npos: " << joint_pos_.transpose() << "\nvel: " << joint_vel_.transpose() << "\ntau: " << joint_tau_.transpose() << std::endl;
        std::cout << "Imu Data: \nrpy: " << rpy_.transpose() << "\nacc: " << acc_.transpose() << "\nomg: " << omg_.transpose() << std::endl;
    }

    void DisplayAxisValue(){
        auto cmd = uc_ptr_->GetUserCommand();
        std::cout << "User Command Input: \naxis value:  " << cmd.forward_vel_scale << " " << cmd.side_vel_scale << " " << cmd.turnning_vel_scale << "\ntarget mode: " << cmd.target_mode << std::endl;
    }

public:
    IdleState(const RobotType& robot_type, const std::string& state_name, std::shared_ptr<ControllerData> data_ptr):StateBase(robot_type, state_name, data_ptr){}
    ~IdleState(){}

    virtual void OnEnter() {
        StateBase::msfb_.UpdateCurrentState(RobotMotionState::WaitingForStand);
        std::cout << "[Idle] Kp/Kd=0; no new Piper motion. Press Z to ramp gains and stand." << std::endl;
        uc_ptr_->SetMotionStateFeedback(StateBase::msfb_);
        enter_state_time_ = ri_ptr_->GetInterfaceTimeStamp();
    };

    virtual void OnExit() { first_enter_flag_ = false; }

    virtual void Run() {
        GetProprioceptiveData();
        joint_normal_flag_ = JointDataNormalCheck();
        imu_normal_flag_ = ImuDataNormalCheck();
        
        if (((ri_ptr_->GetInterfaceTimeStamp() - last_print_time) > 1)) {
            DisplayProprioceptiveInfo();
            DisplayAxisValue();
            last_print_time = ri_ptr_->GetInterfaceTimeStamp();
        }
        
        MatXf cmd = MatXf::Zero(22, 5); 
        
        // Passive until Z: zero Kp, Kd, desired velocity and feed-forward.
        // Position merely follows measurement; zero arm Kp also prevents the
        // hardware adapter from sending MOVE_J targets before Z.
        cmd.col(1) = joint_pos_;

        ri_ptr_->SetJointCommand(cmd); 
    }

    virtual bool LoseControlJudge() { return false; }

    virtual StateName GetNextStateName() {
        if(!joint_normal_flag_ || !imu_normal_flag_) return StateName::kIdle;
        if(first_enter_flag_ && ri_ptr_->GetInterfaceTimeStamp() - enter_state_time_ < 0.5) return StateName::kIdle;
        if(uc_ptr_->GetUserCommand().target_mode == int(RobotMotionState::StandingUp)) return StateName::kStandUp;
        return StateName::kIdle;
    }
};
