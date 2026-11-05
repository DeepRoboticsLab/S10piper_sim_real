#pragma once

#include "user_command_interface.h"
#include "piper_safe_workspace.hpp"
#include <cstdio>
#include <functional>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include <mutex> 
#include <sys/select.h>
#include <algorithm> // 用于 std::max, std::min

#define VEL_STEP 0.05
#define YAW_STEP 0.20
#define POS_STEP 0.02 
#define WRIST_YAW_STEP_RAD 0.0872664626f
#define WRIST_YAW_LIMIT_RAD 1.0471975512f

using namespace types;

namespace interface {

class KeyboardInterface : public UserCommandInterface
{
private:
    UserCommand usr_cmd_;
    bool start_thread_flag_;
    std::thread kb_thread_;
    std::mutex cmd_mutex_; 
    MotionStateFeedback msfb_;

    double GetCurrentTimeStamp(){
        static timespec startup_timestamp;
        timespec now_timestamp;
        if (startup_timestamp.tv_sec + startup_timestamp.tv_nsec == 0) {
            clock_gettime(CLOCK_MONOTONIC,&startup_timestamp);
        }
        clock_gettime(CLOCK_MONOTONIC,&now_timestamp);
        return (now_timestamp.tv_sec-startup_timestamp.tv_sec)*1e3 
            + (now_timestamp.tv_nsec-startup_timestamp.tv_nsec)/1e6;
    }

public:
    KeyboardInterface(){
        std::memset(&usr_cmd_, 0, sizeof(usr_cmd_));
        
        // 默认保持所选任务的 home EE，避免刚进入策略就触发机械臂任务。
        const Vec3f arm_home = piper_safe_workspace::Home();
        usr_cmd_.arm_pos_des[0] = arm_home.x();
        usr_cmd_.arm_pos_des[1] = arm_home.y();
        usr_cmd_.arm_pos_des[2] = arm_home.z();
        std::cout << "[Keyboard] S10+Piper Home EE: X=0.42, Y=0.0, Z=0.05" << std::endl;
        std::cout << "[Keyboard] training workspace: X=[-0.42,0.58], "
                  << "Y=[-0.50,0.50], Z=[-0.30,0.42] m" << std::endl;
    }
    ~KeyboardInterface(){}

    virtual void Start(){
        start_thread_flag_ = true;
        kb_thread_ = std::thread(std::bind(&KeyboardInterface::Run, this));
    }
    virtual void Stop(){
        start_thread_flag_ = false;
        if(kb_thread_.joinable()) kb_thread_.join();
    }
    
    virtual UserCommand GetUserCommand() override {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        return usr_cmd_;
    }

    virtual void SetMotionStateFeedback(const MotionStateFeedback& msfb){
        msfb_ = msfb;
    }

    void Run(){
        struct termios oldt, newt;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);

        char input;

        std::cout << "==========================================\n";
        std::cout << "      S10+Piper State-Machine Control\n";
        std::cout << "==========================================\n";
        std::cout << " [W/S]   Base Forward/Backward (Cruise)\n";
        std::cout << " [A/D]   Base Left/Right (disabled for this checkpoint)\n";
        std::cout << " [Q/E]   Base Turn Left/Right (Cruise)\n";
        std::cout << " [SPACE] EMERGENCY BRAKE (Stop Base)\n";
        std::cout << "------------------------------------------\n";
        std::cout << " [I/K]   Arm X: forward/back\n";
        std::cout << " [J/L]   Arm Y: left/right (constrained XYZ IK -> MOVE_J)\n";
        std::cout << " [U/O]   Arm Z: up/down\n";
        std::cout << " [J1-J3] XYZ IK -> JointCtrl; J4/J5/J6 fixed at training Home\n";
        std::cout << " [N/M]   Piper Gripper: fully open / fully close\n";
        std::cout << " [ H ]   Arm Task Target Reset\n"; // 添加菜单提示
        std::cout << " [ Z ]   Stand Up\n";
        std::cout << " [ C ]   Start RL Policy\n";
        std::cout << "==========================================\n";

        while (start_thread_flag_) {
            fd_set read_fds;
            FD_ZERO(&read_fds);
            FD_SET(STDIN_FILENO, &read_fds);
            timeval timeout{0, 50000};
            const int ready = select(
                STDIN_FILENO + 1, &read_fds, nullptr, nullptr, &timeout);
            if (ready > 0 && read(STDIN_FILENO, &input, 1) == 1) {
                std::lock_guard<std::mutex> lock(cmd_mutex_); 
                if (input != 'n' && input != 'm' && input != '[' && input != ']')
                    usr_cmd_.soft_stop_flag = false;

                auto zero_vel_cmd = [&](){
                    usr_cmd_.forward_vel_scale  = 0.0f;
                    usr_cmd_.side_vel_scale     = 0.0f;
                    usr_cmd_.turnning_vel_scale = 0.0f;
                };

                if(input == 'r') { usr_cmd_.target_mode = int(RobotMotionState::JointDamping); zero_vel_cmd(); }
                if(input == 'z') { usr_cmd_.target_mode = int(RobotMotionState::StandingUp);  zero_vel_cmd(); }
                if(input == 'c') { usr_cmd_.target_mode = int(RobotMotionState::RLControlMode); zero_vel_cmd(); }

                if(input == ' ') { 
                    zero_vel_cmd(); 
                    std::cout << "\r[BRAKE] Base Stopped!                             " << std::flush; 
                }

                if (usr_cmd_.target_mode == int(RobotMotionState::RLControlMode)) {
                    
                    // Gripper keys only change the binary gripper target.
                    // Do not reset H or change XYZ/base velocity here.
                    if (input == 'n' || input == 'm') {
                        usr_cmd_.piper_gripper_command = input == 'n' ? 1 : 2;
                        std::cout << (input == 'n'
                            ? "\r[Gripper] N: fully OPEN       "
                            : "\r[Gripper] M: fully CLOSE      ") << std::flush;
                    }

                    // 底盘速度控制 (定速巡航模式)
                    if(input=='w') {
                        // The command-split policy is trained on pure X,
                        // pure yaw, and idle as mutually exclusive modes.
                        usr_cmd_.turnning_vel_scale = 0.0f;
                        usr_cmd_.forward_vel_scale += VEL_STEP;
                    }
                    if(input=='s') {
                        usr_cmd_.turnning_vel_scale = 0.0f;
                        usr_cmd_.forward_vel_scale -= VEL_STEP;
                    }
                    // 横向训练效果暂不可靠；A/D 保留按键位置但不向策略发命令。
                    if(input=='a' || input=='d') {
                        usr_cmd_.side_vel_scale = 0.0f;
                    }
                    // Match the proven no-FSM deployment.  The previous 0.05
                    // rad/s step sat in the training yaw dead zone and often
                    // produced only a rear-leg posture response.
                    if(input=='q') {
                        usr_cmd_.forward_vel_scale = 0.0f;
                        usr_cmd_.turnning_vel_scale += YAW_STEP;
                    }
                    if(input=='e') {
                        usr_cmd_.forward_vel_scale = 0.0f;
                        usr_cmd_.turnning_vel_scale -= YAW_STEP;
                    }

                    // 机械臂笛卡尔坐标目标控制
                    if(input=='i') usr_cmd_.arm_pos_des[0] += POS_STEP; 
                    if(input=='k') usr_cmd_.arm_pos_des[0] -= POS_STEP; 
                    if(input=='j') usr_cmd_.arm_pos_des[1] += POS_STEP; 
                    if(input=='l') usr_cmd_.arm_pos_des[1] -= POS_STEP; 
                    if(input=='u') usr_cmd_.arm_pos_des[2] += POS_STEP; 
                    if(input=='o') usr_cmd_.arm_pos_des[2] -= POS_STEP; // 如果你想把o键改成复位，删掉这行
                    if(input=='[' || input==']') {
                        std::cout << "\r[Arm] J4-J6 locked at training Home; independent wrist key ignored.\n"
                                  << std::flush;
                    }

                    // Clamp the stored keyboard target itself. This avoids a
                    // hidden out-of-range backlog when the operator reverses.
                    const Vec3f requested_arm(
                        usr_cmd_.arm_pos_des[0], usr_cmd_.arm_pos_des[1],
                        usr_cmd_.arm_pos_des[2]);
                    const Vec3f safe_arm =
                        piper_safe_workspace::Clamp(requested_arm);
                    const bool arm_workspace_limited =
                        (safe_arm - requested_arm).cwiseAbs().maxCoeff() >
                        1.0e-6f;
                    usr_cmd_.arm_pos_des[0] = safe_arm.x();
                    usr_cmd_.arm_pos_des[1] = safe_arm.y();
                    usr_cmd_.arm_pos_des[2] = safe_arm.z();

                    // 机械臂任务目标复位。
                    if(input=='h') { 
                        const Vec3f home = piper_safe_workspace::Home();
                        usr_cmd_.arm_pos_des[0] = home.x();
                        usr_cmd_.arm_pos_des[1] = home.y();
                        usr_cmd_.arm_pos_des[2] = home.z();
                        usr_cmd_.piper_wrist_yaw_rad = 0.0f;
                        usr_cmd_.soft_stop_flag = true;
                        // 复位时强制清零底盘速度，防止狗在失稳状态下继续乱跑（可选，推荐保留）
                        zero_vel_cmd(); 
                    }

                    // 安全限幅：防止按键过多导致底盘速度越界飞车
                    // The policy uses a 10 rad/s target (0.81 m/s at the S10
                    // wheel radius of 0.081 m). Preserve
                    // differential yaw authority by keeping X below saturation.
                    usr_cmd_.forward_vel_scale = std::max(-0.8f, std::min(0.8f, usr_cmd_.forward_vel_scale));
                    usr_cmd_.side_vel_scale = 0.0f;
                    usr_cmd_.turnning_vel_scale = std::max(-1.0f, std::min(1.0f, usr_cmd_.turnning_vel_scale));

                    // 终端状态实时打印 (加入 h 键的监听)
                    if(std::string("wasdqeijkluoh []").find(input) != std::string::npos) {
                        if(input == 'h') {
                            printf("\r\033[1;32m[SAFE] S10+Piper returning to training Home.\033[0m      ");
                        } else {
                            printf("\r%s Vel(%.1f, %.1f), Yaw=%.1f | Arm(%.2f, %.2f, %.2f), wrist=%.0f deg      ",
                                arm_workspace_limited ? "[ARM LIMIT]" : "[CMD]",
                                usr_cmd_.forward_vel_scale, usr_cmd_.side_vel_scale,
                                usr_cmd_.turnning_vel_scale,
                                usr_cmd_.arm_pos_des[0], usr_cmd_.arm_pos_des[1], usr_cmd_.arm_pos_des[2],
                                usr_cmd_.piper_wrist_yaw_rad * 57.295779513f);
                        }
                        fflush(stdout);
                    }
                }
            }
        }
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    }
};

}
