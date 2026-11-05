/**
 * @file robot_interface.h
 * @brief this file is for applying robot's control interface
 * @author mazunwang
 * @version 1.0
 * @date 2024-04-11
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */
#pragma once

#include "common_types.h"
#include <atomic>

using namespace types;

namespace interface{
class RobotInterface{
private:
    /* data */
public:
    /**
     * @brief Construct a new Robot Interface object
     * @param  robot_name       robot name
     * @param  dof_num          The number of degrees of freedom of the robot.
     */
    RobotInterface(const std::string& robot_name, int dof_num):robot_name_(robot_name), dof_num_(dof_num){
        start_flag_ = true;
    }
    virtual ~RobotInterface(){};

    std::string robot_name_;
    const int dof_num_;
    Eigen::Matrix<float, Eigen::Dynamic, 5> joint_cmd_;
    std::atomic<bool> start_flag_;

    /**
     * @brief Start to get robot's state and send control command
     */
    virtual void Start() = 0;

    /**
     * @brief Stop to control the robot
     */
    virtual void Stop() = 0;

    /** Perform an optional controlled transition before transport shutdown. */
    virtual void PrepareShutdown() {}

    /**
     * @brief Get the time stamp of the robot
     * @return double        time stamp
     */
    virtual double GetInterfaceTimeStamp() = 0;

    /**
     * @brief Get the joint position of the robot
     * @return VecXf        joint position vector
     */
    virtual VecXf GetJointPosition() = 0;

    /**
     * @brief Get the joint velocity of the robot
     * @return VecXf        joint velocity vector
     */
    virtual VecXf GetJointVelocity() = 0;

    /**
     * @brief Get the joint torque of the robot
     * @return VecXf        joint torque vector
     */
    virtual VecXf GetJointTorque() = 0;

    /**
     * @brief Get the roll-pitch-yaw angle of the robot base
     * @return Vec3f        roll-pitch-yaw(unit rad)
     */
    virtual Vec3f GetImuRpy() = 0;

    /**
     * @brief Get the accleration of the robot base
     * @return Vec3f        accleration(unit m*s^-2)
     */
    virtual Vec3f GetImuAcc() = 0;

    /**
     * @brief Get the angular velocity of robot base
     * @return Vec3f        angular velocity in body coordinate(unit rad/s)
     */
    virtual Vec3f GetImuOmega() = 0;

    /**
     * @brief Set the joint command in standard form
     *                  torque = kp * (qDes - q) + kd * (vDes - v) + tff
     * @param  input       a dof_num*5 matrix and each column represent kp, goal_angle_pos, kd, goal_vel, torque_feedforward 
     * (current torque, not last torque, video content slip of the tongue)
     */
    virtual void SetJointCommand(Eigen::Matrix<float, Eigen::Dynamic, 5> input) = 0;

    // Optional layered Piper command.  The policy keeps simulating its six
    // arm joints, while a real Piper follows this Cartesian offset through
    // the independent Orin IK bridge.  Non-Piper/simulation backends keep the
    // no-op default.
    virtual void SetPiperCartesianCommand(
        const Vec3f& position_offset_m,
        float wrist_yaw_rad = 0.0f,
        bool exact_training_home = false) {
        (void)position_offset_m;
        (void)wrist_yaw_rad;
        (void)exact_training_home;
    }

    // Stand-up may outlast the S10 leg trajectory while the separate Piper
    // bridge slews toward its training Home.  Simulation needs no extra wait.
    virtual bool PiperReadyForLayeredRL() const { return true; }

    // Binary gripper command, independent of the Cartesian arm/leg targets.
    // Zero means no operator request, not an implicit close command.
    virtual void SetPiperGripperCommand(int command) { (void)command; }

    // Simulation compatibility: legacy MuJoCo backends expose a normalized
    // gripper target, while the layered real Piper uses the binary API above.
    virtual void SetGripperCommand(float closed) { (void)closed; }

    // Direct-joint backends may remove their Home-transition speed limiter
    // once the actor owns the arm. Simulation requires no special handling.
    virtual void SetPiperPolicyControl(bool active) { (void)active; }

    // Unlike PiperReadyForLayeredRL(), this is a strict physical-state check
    // used while releasing an O-key front-leg hold. A stale real-arm link
    // must not be treated as Home.
    virtual bool PiperAtTrainingHome() const { return true; }

    virtual MatXf GetJointCommand(){
        return joint_cmd_;
    }

    /**
     * @brief Get the contact force if robot have any force sensor
     * @return VecXf        force vector
     */
    virtual VecXf GetContactForce() = 0;

    /**
     * @brief Get the motor temperture 
     * @return VecXf        motor temperture
     */
    virtual VecXf GetMotorTemperture(){
        return VecXf::Zero(dof_num_);
    }

    /**
     * @brief Get the driver temperture 
     * @return VecXf        driver temperture
     */
    virtual VecXf GetDriverTemperture(){
        return VecXf::Zero(dof_num_);
    }
};


};//namespace interface
