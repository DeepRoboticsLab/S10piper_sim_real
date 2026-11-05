/**
 * @file state_machine.hpp
 * @brief Robot State Machine (Final Fix: Correct Path)
 */
#pragma once

#include "state_base.h"
#include "idle_state.hpp"
#include "standup_state.hpp"
#include "joint_damping_state.hpp"
#include "rl_control_state_onnx.hpp"
#include "shutdown_signal.hpp"
#include "control_ticker.hpp"

#ifdef BUILD_HARDWARE
    #include "../interface/robot/hardware/s10_piper_hardware_interface.hpp"
    #include <rclcpp/rclcpp.hpp>
#else
    #error "Use the root simulation build or the S10 DDS hardware build"
#endif

#ifdef S10_VR_CONTROL
    #include "../interface/user_command/vr_command_interface.hpp"
#else
    #include "../interface/user_command/keyboard_interface.hpp"
#endif

#include "data_streaming.hpp"

class StateMachine{
private:
    std::shared_ptr<StateBase> current_controller_;
    std::shared_ptr<StateBase> idle_controller_;
    std::shared_ptr<StateBase> standup_controller_;
    std::shared_ptr<StateBase> rl_controller_;
    std::shared_ptr<StateBase> joint_damping_controller_;

    StateName current_state_name_, next_state_name_;

    std::shared_ptr<UserCommandInterface> uc_ptr_;
    std::shared_ptr<RobotInterface> ri_ptr_; 
    std::shared_ptr<ControlParameters> cp_ptr_;
    std::shared_ptr<DataStreaming> ds_ptr_;
    bool interfaces_stopped_ = false;

    bool KeepRunning() const {
#ifdef BUILD_HARDWARE
        return rclcpp::ok() && !shutdown_signal::Requested();
#else
        return true;
#endif
    }

    void StopInterfaces() {
        if (interfaces_stopped_) return;
        interfaces_stopped_ = true;
        if (ri_ptr_) ri_ptr_->Stop();
        if (uc_ptr_) uc_ptr_->Stop();
    }

    void GetDataStreaming(){
        if(!ri_ptr_) return;
        VecXf pos = ri_ptr_->GetJointPosition();
        VecXf vel = ri_ptr_->GetJointVelocity();
        VecXf tau = ri_ptr_->GetJointTorque();
        Vec3f rpy = ri_ptr_->GetImuRpy();
        Vec3f acc = ri_ptr_->GetImuAcc();
        Vec3f omg = ri_ptr_->GetImuOmega();
        MatXf jc = ri_ptr_->GetJointCommand();

        ds_ptr_->InsertInterfaceTime(ri_ptr_->GetInterfaceTimeStamp());
        ds_ptr_->InsertJointData("q", pos);
        ds_ptr_->InsertJointData("dq", vel);
        ds_ptr_->InsertJointData("tau", tau);
        ds_ptr_->InsertJointData("q_cmd", jc.col(1));
        ds_ptr_->InsertJointData("tau_ff", jc.col(4));
        ds_ptr_->InsertImuData("rpy", rpy);
        ds_ptr_->InsertImuData("acc", acc);
        ds_ptr_->InsertImuData("omg", omg);

        if(!uc_ptr_) return;
        auto cmd = uc_ptr_->GetUserCommand();
        ds_ptr_->InsertCommandData("target_mode", float(cmd.target_mode));
        ds_ptr_->InsertStateData("current_state", StateBase::msfb_.current_state);
        ds_ptr_->SendData();
    }

    std::shared_ptr<StateBase> GetNextStatePtr(StateName state_name){
        switch(state_name){
            case StateName::kInvalid:{ return nullptr; }
            case StateName::kIdle:{ return idle_controller_; }
            case StateName::kStandUp:{ return standup_controller_; }
            case StateName::kRLControl:{ return rl_controller_; }
            case StateName::kJointDamping:{ return joint_damping_controller_; }
            default:{ std::cerr << "error state name" << std::endl; }
        }
        return nullptr;
    }

public:
    StateMachine(RobotType robot_type){
        const std::string activation_key = "~/raisim/activation.raisim";
        std::string urdf_path = "";
        std::string mjcf_path = ""; 
        
#ifdef S10_VR_CONTROL
        uc_ptr_ = std::make_shared<interface::VrCommandInterface>();
#else
        uc_ptr_ = std::make_shared<interface::KeyboardInterface>();
#endif
        if(robot_type == RobotType::S10){
#ifdef BUILD_HARDWARE
            std::cout << ">>> Loading official S10 DDS + virtual/reserved Piper 22-DOF interface <<<" << std::endl;
            ri_ptr_ = std::make_shared<interface::S10PiperHardwareInterface>("S10_Piper");
#else
            #error "The legacy simulator backend is not part of this package"
#endif
            cp_ptr_ = std::make_shared<ControlParameters>(robot_type);
        } else {
            std::cerr << "Unknown robot type error" << std::endl;
        }

        std::shared_ptr<ControllerData> data_ptr = std::make_shared<ControllerData>();
        data_ptr->ri_ptr = ri_ptr_;
        data_ptr->uc_ptr = uc_ptr_;
        data_ptr->cp_ptr = cp_ptr_;
        ds_ptr_ = std::make_shared<DataStreaming>(false, false);
        data_ptr->ds_ptr = ds_ptr_;

        // 3. 初始化状态
        idle_controller_ = std::make_shared<IdleState>(robot_type, "idle_state", data_ptr);
        standup_controller_ = std::make_shared<StandUpState>(robot_type, "standup_state", data_ptr);
        rl_controller_ = std::make_shared<RLControlStateONNX>(robot_type, "rl_control", data_ptr);
        joint_damping_controller_ = std::make_shared<JointDampingState>(robot_type, "joint_damping", data_ptr);

        current_controller_ = idle_controller_;
        current_state_name_ = kIdle;
        next_state_name_ = kIdle;
   
        // 启动接口线程
        ri_ptr_->Start();
        std::cout << "Robot interface started" << std::endl;
        uc_ptr_->Start();
        
        current_controller_->OnEnter();  
    }
    ~StateMachine(){ StopInterfaces(); }

    void Run(){
        static double time_record = 0;
        std::cout << "--- Main Loop Active. Waiting for Data... ---" << std::endl;
#ifdef BUILD_HARDWARE
        ControlTicker control_ticker(ControlTicker::Clock::now());
        std::cout << "[Timing] 200 Hz wall-clock control, 50 Hz policy; "
                     "latest DDS feedback with freshness watchdog." << std::endl;
#endif

        try {
        while(KeepRunning()){
#ifdef BUILD_HARDWARE
            if (!control_ticker.Due(ControlTicker::Clock::now())) {
                std::this_thread::sleep_until(control_ticker.Next());
                continue;
            }
#endif
            double current_ts = ri_ptr_->GetInterfaceTimeStamp();
            
#ifdef BUILD_HARDWARE
            if (current_ts > 0.0) {
#else
            if(current_ts != time_record){
#endif
                if (time_record == 0) std::cout << ">>> Heartbeat Detected! System Live. <<<" << std::endl;

                time_record = current_ts;
                current_controller_ -> Run();
                
                if(current_controller_->LoseControlJudge()) next_state_name_ = StateName::kJointDamping;
                else next_state_name_ = current_controller_ -> GetNextStateName();
                
                if(next_state_name_ != current_state_name_){
                    current_controller_ -> OnExit();
                    std::cout << current_controller_ -> state_name_ << " ------------> ";
                    current_controller_ = GetNextStatePtr(next_state_name_);
                    std::cout << current_controller_ -> state_name_ << std::endl;
                    current_controller_ ->OnEnter();
                    current_state_name_ = next_state_name_; 
                }
                this->GetDataStreaming();
            } 
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
        } catch (...) {
            StopInterfaces();
            throw;
        }
#ifdef BUILD_HARDWARE
        if (shutdown_signal::Requested() && ri_ptr_) {
            std::cout << "[SAFE] Ctrl+C: smoothly crouching S10 before exit."
                      << std::endl;
            ri_ptr_->PrepareShutdown();
        }
#endif
        StopInterfaces();
    }
};
