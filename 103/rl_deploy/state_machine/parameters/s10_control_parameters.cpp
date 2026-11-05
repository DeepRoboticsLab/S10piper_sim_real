#include "control_parameters.h"

void ControlParameters::GenerateS10Parameters(){
    body_len_x_ = 0.2277 * 2.0;
    body_len_y_ = 0.0574 * 2.0;
    hip_len_ = 0.0779;
    thigh_len_ = 0.18;
    shank_len_ = 0.18;

    pre_height_ = 0.20;
    stand_height_ = 0.324;
    
    // 前三个是腿部增益，第四个是轮子锁车增益
    swing_leg_kp_ << 40.0, 40.0, 40.0, 0.0;
    swing_leg_kd_ << 1.0, 1.0, 1.0, 0.4;

    // Official S10 URDF limits. Mirroring for the other legs is performed by
    // IdleState's safety check.
    fl_joint_lower_ << -0.6109, -2.5307, -2.7227, -1e5;
    fl_joint_upper_ <<  0.6109,  2.5307,  2.7227,  1e5;
    
    joint_vel_limit_ << 30.0, 30.0, 30.0, 65.5;
    torque_limit_ << 50.0, 50.0, 50.0, 14.0;
}
