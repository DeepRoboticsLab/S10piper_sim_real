#pragma once

// Real-hardware adapter for the trained 22-DoF contract:
//   [S10 FL4, FR4, HL4, HR4, Piper J1..J6].
//
// The first 16 joints use the vendor S10 ROS 2/drdds transport and the exact
// direction/encoder offsets shipped in S10_sdk_deploy 1.4.1.  Piper remains a
// separate TCP subsystem; its six values are never written into /JOINTS_CMD.

#include "robot_interface.h"
#include "json.hpp"
#include "piper_safe_workspace.hpp"
#include "z_startup_profile.hpp"

#include "drdds/msg/battery_data.hpp"
#include "drdds/msg/imu_data.hpp"
#include "drdds/msg/joints_data.hpp"
#include "drdds/msg/joints_data_cmd.hpp"
#include "rclcpp/rclcpp.hpp"

#include <algorithm>
#include <arpa/inet.h>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <fcntl.h>
#include <iostream>
#include <mutex>
#include <netinet/tcp.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/select.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

namespace interface {
namespace s10_real_detail {

constexpr int kS10Dof = 16;
constexpr int kPiperDof = 6;
constexpr int kWholeBodyDof = 22;

constexpr uint16_t kDisable = 1;
constexpr uint16_t kEnable = 2;
constexpr uint16_t kMotorControl = 4;
constexpr uint16_t kErrorReset = 17;
constexpr uint16_t kGetStatus = 23;

inline float DegToRad(float degrees) {
    return degrees * static_cast<float>(M_PI / 180.0);
}

inline std::string EnvOr(const char* name, const char* fallback) {
    const char* value = std::getenv(name);
    return value && *value ? std::string(value) : std::string(fallback);
}

inline int EnvIntOr(const char* name, int fallback) {
    const char* value = std::getenv(name);
    if (!value || !*value) return fallback;
    try {
        return std::stoi(value);
    } catch (...) {
        return fallback;
    }
}

inline bool EnvBoolOr(const char* name, bool fallback) {
    const std::string value = EnvOr(name, fallback ? "1" : "0");
    return value == "1" || value == "true" || value == "TRUE" ||
           value == "yes" || value == "YES";
}

class PiperTcpClient {
private:
    std::string host_;
    int port_;
    std::atomic<bool> running_{false};
    std::atomic<bool> connected_{false};
    std::thread thread_;
    mutable std::mutex state_mutex_;
    std::mutex socket_mutex_;
    std::condition_variable state_cv_;
    int socket_fd_ = -1;
    VecXf position_ = VecXf::Zero(kPiperDof);
    VecXf velocity_ = VecXf::Zero(kPiperDof);
    std::chrono::steady_clock::time_point last_state_{};
    bool state_received_ = false;
    bool enabled_ = false;
    bool feedback_fresh_ = false;
    bool training_ik_supported_ = false;
    std::atomic<bool> shutdown_requested_{false};
    bool home_request_sent_ = false;
    bool home_disable_complete_ = false;
    bool home_disable_ok_ = false;

    void CloseSocketLocked() {
        if (socket_fd_ >= 0) {
            ::shutdown(socket_fd_, SHUT_RDWR);
            ::close(socket_fd_);
            socket_fd_ = -1;
        }
        connected_ = false;
    }

    bool SendTextLocked(const std::string& text) {
        if (socket_fd_ < 0) return false;
        size_t sent = 0;
        while (sent < text.size()) {
            const ssize_t n = ::send(socket_fd_, text.data() + sent,
                                     text.size() - sent,
                                     MSG_NOSIGNAL | MSG_DONTWAIT);
            if (n <= 0) {
                CloseSocketLocked();
                return false;
            }
            sent += static_cast<size_t>(n);
        }
        return true;
    }

    void HandleLine(const std::string& line) {
        try {
            const auto msg = nlohmann::json::parse(line);
            const auto type = msg.value("type", "");
            if (type == "home_disable_complete") {
                std::lock_guard<std::mutex> lock(state_mutex_);
                home_disable_ok_ = msg.value("ok", false);
                home_disable_complete_ = true;
                state_cv_.notify_all();
                return;
            }
            if (type != "state") return;
            const auto q = msg.at("q").get<std::vector<float>>();
            const auto dq = msg.at("dq").get<std::vector<float>>();
            if (q.size() != kPiperDof || dq.size() != kPiperDof) return;

            VecXf next_q(kPiperDof), next_dq(kPiperDof);
            for (int i = 0; i < kPiperDof; ++i) {
                if (!std::isfinite(q[i]) || !std::isfinite(dq[i])) return;
                next_q(i) = q[i];
                next_dq(i) = dq[i];
            }
            std::lock_guard<std::mutex> lock(state_mutex_);
            position_ = next_q;
            velocity_ = next_dq;
            last_state_ = std::chrono::steady_clock::now();
            state_received_ = true;
            enabled_ = msg.value("enabled", true);
            feedback_fresh_ = msg.value("feedback_fresh", true);
#ifdef S10_PIPER_REALTIME
            training_ik_supported_ = false;
            if (msg.contains("training_ik_profiles") &&
                msg.at("training_ik_profiles").is_array()) {
                for (const auto& profile : msg.at("training_ik_profiles")) {
                    if (profile.is_string() && profile.get<std::string>() ==
                            "training_y_joint1_xz_j23_v1") {
                        training_ik_supported_ = true;
                        break;
                    }
                }
            }
#else
            training_ik_supported_ = msg.value("training_ik_profile", "") ==
                "training_j123_locked_j456_v1";
#endif
            state_cv_.notify_all();
        } catch (...) {
            // Newline framing lets a later valid frame recover automatically.
        }
    }

    void Run() {
        while (running_) {
            if (shutdown_requested_ && !connected_) break;
            const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
            if (fd < 0) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            }

            int one = 1;
            ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
            timeval io_timeout{0, 250000};
            ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO,
                         &io_timeout, sizeof(io_timeout));
            ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO,
                         &io_timeout, sizeof(io_timeout));

            sockaddr_in address{};
            address.sin_family = AF_INET;
            address.sin_port = htons(port_);
            const int original_flags = ::fcntl(fd, F_GETFL, 0);
            bool connected = false;
            if (original_flags >= 0 &&
                ::fcntl(fd, F_SETFL, original_flags | O_NONBLOCK) == 0 &&
                ::inet_pton(AF_INET, host_.c_str(), &address.sin_addr) == 1) {
                const int result = ::connect(
                    fd, reinterpret_cast<sockaddr*>(&address), sizeof(address));
                if (result == 0) {
                    connected = true;
                } else if (errno == EINPROGRESS) {
                    fd_set write_fds;
                    FD_ZERO(&write_fds);
                    FD_SET(fd, &write_fds);
                    timeval connect_timeout{0, 500000};
                    if (::select(fd + 1, nullptr, &write_fds, nullptr,
                                 &connect_timeout) > 0) {
                        int socket_error = 0;
                        socklen_t error_size = sizeof(socket_error);
                        connected = ::getsockopt(
                            fd, SOL_SOCKET, SO_ERROR, &socket_error,
                            &error_size) == 0 && socket_error == 0;
                    }
                }
            }
            if (!connected || ::fcntl(fd, F_SETFL, original_flags) != 0) {
                ::close(fd);
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            }

            {
                std::lock_guard<std::mutex> lock(socket_mutex_);
                socket_fd_ = fd;
                connected_ = true;
                SendTextLocked("{\"type\":\"subscribe\"}\n");
            }
            std::cout << "[Piper] connected to " << host_ << ':' << port_
                      << std::endl;

            std::string pending;
            char buffer[2048];
            while (running_ && connected_) {
                const ssize_t n = ::recv(fd, buffer, sizeof(buffer), 0);
                if (n > 0) {
                    pending.append(buffer, static_cast<size_t>(n));
                    size_t newline = 0;
                    while ((newline = pending.find('\n')) != std::string::npos) {
                        HandleLine(pending.substr(0, newline));
                        pending.erase(0, newline + 1);
                    }
                    if (pending.size() > 65536) pending.clear();
                } else if (n == 0) {
                    break;
                } else if (errno != EAGAIN && errno != EWOULDBLOCK &&
                           errno != EINTR) {
                    break;
                }
            }

            {
                std::lock_guard<std::mutex> lock(socket_mutex_);
                if (socket_fd_ == fd) CloseSocketLocked();
            }
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                state_received_ = false;
                enabled_ = false;
                feedback_fresh_ = false;
                training_ik_supported_ = false;
                state_cv_.notify_all();
            }
            // A normal exit must not start an endless reconnect loop while
            // the controller is waiting for the arm to return to zero.
            if (shutdown_requested_) break;
            if (running_) {
                std::cerr << "[Piper] link lost; retrying in 1 s" << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }
    }

public:
    PiperTcpClient(std::string host, int port)
        : host_(std::move(host)), port_(port) {}
    ~PiperTcpClient() { Stop(false); }

    void Start() {
        if (running_.exchange(true)) return;
        thread_ = std::thread(&PiperTcpClient::Run, this);
    }

    bool WaitForState(std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lock(state_mutex_);
        return state_cv_.wait_for(
            lock, timeout, [this] { return state_received_; });
    }

    bool StateFresh(std::chrono::milliseconds max_age =
                        std::chrono::milliseconds(500)) const {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return state_received_ &&
               feedback_fresh_ &&
               std::chrono::steady_clock::now() - last_state_ <= max_age;
    }

    bool Enabled() const {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return state_received_ && enabled_;
    }

    bool TrainingIKSupported() const {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return state_received_ && training_ik_supported_;
    }

    VecXf Position() const {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return position_;
    }

    VecXf Velocity() const {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return velocity_;
    }

    bool SendJointTarget(const Eigen::Ref<const VecXf>& target,
                         bool startup_home = false) {
        if (target.size() != kPiperDof || !target.allFinite()) return false;
        nlohmann::json msg{
            {"type", "joint"},
            {"target", std::vector<float>(target.data(),
                                           target.data() + target.size())}};
        if (startup_home) msg["startup_home"] = true;
        std::lock_guard<std::mutex> lock(socket_mutex_);
        return SendTextLocked(msg.dump() + "\n");
    }

    bool SendGripperCommand(bool fully_open) {
        const nlohmann::json msg{
            {"type", "gripper"}, {"opening_m", fully_open ? 0.080 : 0.0},
            {"binary_only", true}};
        std::lock_guard<std::mutex> lock(socket_mutex_);
        return SendTextLocked(msg.dump() + "\n");
    }

    bool SendTrainingIKOffset(const Vec3f& offset, float wrist_yaw_rad,
                         bool exact_home) {
        (void)wrist_yaw_rad;  // J4-J6 stay at training Home in this profile.
        if (!offset.allFinite()) return false;
        const nlohmann::json msg{
            {"type", "link6_offset"},
            {"xyz_m", std::vector<float>{offset.x(), offset.y(), offset.z()}},
            {"workspace_profile", piper_safe_workspace::kProfile},
#ifdef S10_PIPER_REALTIME
            {"ik_profile", "training_y_joint1_xz_j23_v1"},
#else
            {"ik_profile", "training_j123_locked_j456_v1"},
#endif
            {"allow_all_joints", false},
            {"exact_training_home", exact_home}};
        std::lock_guard<std::mutex> lock(socket_mutex_);
        // Require explicit support; an older bridge must not unlock the wrist.
        if (!TrainingIKSupported()) return false;
        return SendTextLocked(msg.dump() + "\n");
    }

    bool RequestHomeDisable() {
        if (!running_ || !connected_) return false;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            home_disable_complete_ = false;
            home_disable_ok_ = false;
        }
        shutdown_requested_ = true;
        std::lock_guard<std::mutex> lock(socket_mutex_);
        home_request_sent_ = SendTextLocked("{\"type\":\"home_disable\"}\n");
        if (home_request_sent_) {
            std::cout << "[Piper] Ctrl+C: hold 0.5 s, return all six joints "
                         "to zero at 15%, then disable.\n";
        }
        return home_request_sent_;
    }

    void Stop(bool hold) {
        if (!running_) return;
        shutdown_requested_ = true;
        if (home_request_sent_) {
            std::unique_lock<std::mutex> lock(state_mutex_);
            const bool finished = state_cv_.wait_for(
                lock, std::chrono::seconds(45),
                [this] { return home_disable_complete_ || !connected_; });
            if (finished && home_disable_complete_ && home_disable_ok_) {
                std::cout << "[Piper] Six-joint zero verified by Orin; "
                             "arm disabled.\n";
            } else {
                std::cerr << "[Piper] Homing was not confirmed (link lost, "
                             "error or timeout); closing without reconnect.\n";
            }
        }
        if (!running_.exchange(false)) return;
        {
            std::lock_guard<std::mutex> lock(socket_mutex_);
            // Orin owns the entire home/disable sequence once requested.
            // Do not overwrite that request with an immediate disable.
            if (!home_request_sent_) {
                if (hold) SendTextLocked("{\"type\":\"hold\"}\n");
                else SendTextLocked("{\"type\":\"disable\"}\n");
            }
            CloseSocketLocked();
        }
        if (thread_.joinable()) thread_.join();
    }
};

}  // namespace s10_real_detail

class S10PiperHardwareInterface final : public RobotInterface {
private:
    using Clock = std::chrono::steady_clock;
    static constexpr auto kJointFreshness = std::chrono::milliseconds(100);
    static constexpr auto kReadOnlyJointFreshness =
        std::chrono::milliseconds(1500);
    static constexpr auto kImuFreshness = std::chrono::milliseconds(200);
    static constexpr auto kStableWindow = std::chrono::milliseconds(300);

    struct JointConfig {
        float direction = 1.0f;
        float offset = 0.0f;
    };

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<drdds::msg::JointsDataCmd>::SharedPtr joint_cmd_pub_;
    rclcpp::Subscription<drdds::msg::JointsData>::SharedPtr joint_data_sub_;
    rclcpp::Subscription<drdds::msg::ImuData>::SharedPtr imu_data_sub_;
    rclcpp::Subscription<drdds::msg::BatteryData>::SharedPtr battery_sub_;
    std::thread spin_thread_;
    std::atomic<bool> started_{false};

    mutable std::mutex data_mutex_;
    std::condition_variable data_cv_;
    VecXf s10_position_ = VecXf::Zero(s10_real_detail::kS10Dof);
    VecXf s10_velocity_ = VecXf::Zero(s10_real_detail::kS10Dof);
    VecXf s10_torque_ = VecXf::Zero(s10_real_detail::kS10Dof);
    std::array<uint16_t, s10_real_detail::kS10Dof> status_{};
    Vec3f rpy_ = Vec3f::Zero();
    Vec3f acceleration_ = Vec3f::Zero();
    Vec3f angular_velocity_ = Vec3f::Zero();
    std::array<JointConfig, s10_real_detail::kS10Dof> joint_config_{};
    bool joints_ready_ = false;
    bool imu_ready_ = false;
    Clock::time_point last_joint_state_{};
    Clock::time_point last_imu_state_{};
    const Clock::time_point epoch_ = Clock::now();
    double timestamp_ = 0.0;

    s10_real_detail::PiperTcpClient piper_;
    std::string piper_mode_ = "virtual";
    bool piper_real_for_session_ = false;
    bool piper_isolation_logged_ = false;
    bool read_only_ = false;
    bool hold_piper_on_exit_ = false;
    bool piper_activation_requested_ = false;
    bool piper_policy_control_ = false;
    bool piper_recovering_ = true;
    VecXf last_piper_target_ = VecXf::Zero(s10_real_detail::kPiperDof);
    Clock::time_point last_piper_target_time_{};
    Clock::time_point last_piper_cartesian_send_time_{};
    int last_gripper_command_ = 0;
    Clock::time_point last_gripper_send_time_{};
    VecXf virtual_arm_position_ = VecXf::Zero(s10_real_detail::kPiperDof);
    VecXf virtual_arm_velocity_ = VecXf::Zero(s10_real_detail::kPiperDof);
    Clock::time_point last_virtual_arm_update_ = Clock::now();

    void UpdateVirtualArm(const Eigen::Ref<const VecXf>& requested_target) {
        static constexpr std::array<float, s10_real_detail::kPiperDof> kLower{
            -2.618f, 0.0f, -2.967f, -1.745f, -1.22f, -2.0944f};
        static constexpr std::array<float, s10_real_detail::kPiperDof> kUpper{
             2.618f, 3.14f, 0.0f, 1.745f, 1.22f, 2.0944f};
        constexpr float kVirtualJointSpeedRadS = 1.0f;

        const auto now = Clock::now();
        const float dt = std::clamp(
            std::chrono::duration<float>(now - last_virtual_arm_update_).count(),
            0.001f, 0.020f);
        last_virtual_arm_update_ = now;
        const float max_delta = kVirtualJointSpeedRadS * dt;

        for (int i = 0; i < s10_real_detail::kPiperDof; ++i) {
            const float target = std::clamp(
                requested_target(i), kLower[i], kUpper[i]);
            const float delta = std::clamp(
                target - virtual_arm_position_(i), -max_delta, max_delta);
            virtual_arm_position_(i) += delta;
            const float instantaneous_velocity = delta / dt;
            virtual_arm_velocity_(i) =
                0.8f * virtual_arm_velocity_(i) +
                0.2f * instantaneous_velocity;
            if (std::abs(target - virtual_arm_position_(i)) < 1.0e-5f &&
                std::abs(virtual_arm_velocity_(i)) < 1.0e-3f) {
                virtual_arm_velocity_(i) = 0.0f;
            }
        }
    }

    void JointHandler(const drdds::msg::JointsData::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        for (int i = 0; i < s10_real_detail::kS10Dof; ++i) {
            const auto& source = msg->data.joints_data[i];
            s10_position_(i) = source.position * joint_config_[i].direction +
                               joint_config_[i].offset;
            s10_velocity_(i) = source.velocity * joint_config_[i].direction;
            s10_torque_(i) = source.torque * joint_config_[i].direction;
            status_[i] = source.status_word;
        }
        const auto now = Clock::now();
        last_joint_state_ = now;
        timestamp_ = std::chrono::duration<double>(now - epoch_).count();
        joints_ready_ = s10_position_.allFinite() && s10_velocity_.allFinite();
        data_cv_.notify_all();
    }

    void ImuHandler(const drdds::msg::ImuData::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        // DDS RPY is already radians; retain degree conversion for encoders.
        rpy_ << msg->data.roll, msg->data.pitch, msg->data.yaw;
        acceleration_ << msg->data.acc_x, msg->data.acc_y, msg->data.acc_z;
        angular_velocity_ << msg->data.omega_x,
                             msg->data.omega_y,
                             msg->data.omega_z;
        last_imu_state_ = Clock::now();
        imu_ready_ = rpy_.allFinite() && acceleration_.allFinite() &&
                     angular_velocity_.allFinite();
        data_cv_.notify_all();
    }

    void BatteryHandler(const drdds::msg::BatteryData::SharedPtr) {}

    void PublishControlWord(uint16_t control_word) {
        auto msg = drdds::msg::JointsDataCmd();
        for (int i = 0; i < s10_real_detail::kS10Dof; ++i) {
            auto& joint = msg.data.joints_data[i];
            joint.position = 0.0f;
            joint.velocity = 0.0f;
            joint.torque = 0.0f;
            joint.kp = 0.0f;
            joint.kd = 0.0f;
            joint.control_word = control_word;
        }
        joint_cmd_pub_->publish(msg);
    }

    void ResetAndEnableMotors() {
        for (const uint16_t command : {
                 s10_real_detail::kDisable,
                 s10_real_detail::kErrorReset,
                 s10_real_detail::kEnable,
                 s10_real_detail::kGetStatus}) {
            PublishControlWord(command);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void WaitForCommandSubscriber(std::chrono::seconds timeout) const {
        const auto deadline = Clock::now() + timeout;
        while (Clock::now() < deadline) {
            if (joint_cmd_pub_->get_subscription_count() > 0) return;
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        throw std::runtime_error(
            "S10 /JOINTS_CMD has no subscriber; SDK mode/bridge is not ready");
    }

    void WaitForEnabledStatus(std::chrono::seconds timeout) const {
        const auto deadline = Clock::now() + timeout;
        while (Clock::now() < deadline) {
            std::array<uint16_t, s10_real_detail::kS10Dof> status{};
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                status = status_;
            }
            if (std::all_of(status.begin(), status.end(),
                            [](uint16_t value) { return value == 1; })) {
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        std::ostringstream message;
        message << "S10 motors did not report enabled status; status_word=";
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            for (size_t i = 0; i < status_.size(); ++i) {
                if (i) message << ',';
                message << status_[i];
            }
        }
        throw std::runtime_error(message.str());
    }

    bool FeedbackFresh(long long& joint_age_ms, long long& imu_age_ms) const {
        std::lock_guard<std::mutex> lock(data_mutex_);
        const auto now = Clock::now();
        joint_age_ms = joints_ready_
            ? std::chrono::duration_cast<std::chrono::milliseconds>(
                  now - last_joint_state_).count()
            : -1;
        imu_age_ms = imu_ready_
            ? std::chrono::duration_cast<std::chrono::milliseconds>(
                  now - last_imu_state_).count()
            : -1;
        const auto joint_freshness = read_only_
            ? kReadOnlyJointFreshness : kJointFreshness;
        return joints_ready_ && imu_ready_ &&
               now - last_joint_state_ <= joint_freshness &&
               now - last_imu_state_ <= kImuFreshness;
    }

    void WaitForStableFeedback(std::chrono::seconds timeout) const {
        const auto deadline = Clock::now() + timeout;
        Clock::time_point stable_since{};
        long long joint_age_ms = -1;
        long long imu_age_ms = -1;
        while (Clock::now() < deadline) {
            const auto now = Clock::now();
            if (FeedbackFresh(joint_age_ms, imu_age_ms)) {
                if (stable_since == Clock::time_point{}) stable_since = now;
                if (now - stable_since >= kStableWindow) return;
            } else {
                stable_since = {};
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        throw std::runtime_error(
            "S10 feedback is not continuously fresh (joint age=" +
            std::to_string(joint_age_ms) + " ms, IMU age=" +
            std::to_string(imu_age_ms) + " ms)");
    }

    void CorrectWrappedOffsets() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        for (int i = 1; i < s10_real_detail::kS10Dof; i += 4) {
            if (s10_position_(i) < s10_real_detail::DegToRad(-140.0f))
                joint_config_[i].offset += 2.0f * static_cast<float>(M_PI);
            else if (s10_position_(i) > s10_real_detail::DegToRad(140.0f))
                joint_config_[i].offset -= 2.0f * static_cast<float>(M_PI);
        }
        for (int i = 2; i < s10_real_detail::kS10Dof; i += 4) {
            if (s10_position_(i) < s10_real_detail::DegToRad(-164.0f))
                joint_config_[i].offset += 2.0f * static_cast<float>(M_PI);
            else if (s10_position_(i) > s10_real_detail::DegToRad(164.0f))
                joint_config_[i].offset -= 2.0f * static_cast<float>(M_PI);
        }
    }

    void PublishS10(const Eigen::Ref<const MatXf>& input) {
        static constexpr std::array<float, 4> kPositionLower{
            -0.6109f, -2.5307f, -2.7227f, -1.0e6f};
        static constexpr std::array<float, 4> kPositionUpper{
             0.6109f,  2.5307f,  2.7227f,  1.0e6f};
        static constexpr std::array<float, 4> kVelocityLimit{
            30.0f, 30.0f, 30.0f, 65.5f};
        static constexpr std::array<float, 4> kTorqueLimit{
            50.0f, 50.0f, 50.0f, 14.0f};

        auto msg = drdds::msg::JointsDataCmd();
        for (int i = 0; i < s10_real_detail::kS10Dof; ++i) {
            const int joint_in_leg = i % 4;
            auto& joint = msg.data.joints_data[i];
            const float position = std::clamp(
                input(i, 1), kPositionLower[joint_in_leg],
                kPositionUpper[joint_in_leg]);
            const float velocity = std::clamp(
                input(i, 3), -kVelocityLimit[joint_in_leg],
                kVelocityLimit[joint_in_leg]);
            const float torque = std::clamp(
                input(i, 4), -kTorqueLimit[joint_in_leg],
                kTorqueLimit[joint_in_leg]);

            joint.position = (position - joint_config_[i].offset) *
                             joint_config_[i].direction;
            joint.velocity = velocity * joint_config_[i].direction;
            joint.torque = torque * joint_config_[i].direction;
            joint.kp = joint_in_leg == 3
                ? 0.0f : std::clamp(input(i, 0), 0.0f, 200.0f);
            joint.kd = std::clamp(input(i, 2), 0.0f, 20.0f);
            joint.control_word = s10_real_detail::kMotorControl;
        }
        joint_cmd_pub_->publish(msg);
    }

public:
    explicit S10PiperHardwareInterface(const std::string& robot_name)
        : RobotInterface(robot_name, s10_real_detail::kWholeBodyDof),
          node_(std::make_shared<rclcpp::Node>("s10_piper_rl_deploy")),
          piper_(s10_real_detail::EnvOr("PIPER_SERVER_IP", "10.21.41.12"),
                 s10_real_detail::EnvIntOr("PIPER_SERVER_PORT", 9999)) {
        read_only_ = s10_real_detail::EnvBoolOr("S10_REAL_READ_ONLY", false);
        piper_mode_ = s10_real_detail::EnvOr("S10_PIPER_MODE", "virtual");
        if (piper_mode_ != "virtual" && piper_mode_ != "real" &&
            piper_mode_ != "auto") {
            throw std::runtime_error(
                "S10_PIPER_MODE must be virtual, real, or auto");
        }
        hold_piper_on_exit_ =
            s10_real_detail::EnvBoolOr("S10_PIPER_HOLD_ON_EXIT", false);

        virtual_arm_position_ <<
            0.0f, 2.54f, -1.26f, 0.0f, -1.22f, 0.0f;

        // Exact values from the vendor S10Interface in S10_sdk_deploy 1.4.1.
        constexpr float kOffsetDegrees[s10_real_detail::kS10Dof] = {
            -35, -145, 156, 0,
             35, -145, 156, 0,
            -35,  145, -156, 0,
             35,  145, -156, 0};
        constexpr float kDirection[s10_real_detail::kS10Dof] = {
             1,  1, -1,  1,
             1, -1,  1, -1,
            -1,  1, -1,  1,
            -1, -1,  1, -1};
        for (int i = 0; i < s10_real_detail::kS10Dof; ++i) {
            joint_config_[i].direction = kDirection[i];
            joint_config_[i].offset =
                s10_real_detail::DegToRad(kOffsetDegrees[i]);
        }

        joint_cmd_pub_ = node_->create_publisher<drdds::msg::JointsDataCmd>(
            "/JOINTS_CMD", 10);
        joint_data_sub_ = node_->create_subscription<drdds::msg::JointsData>(
            "/JOINTS_DATA", rclcpp::SensorDataQoS(),
            std::bind(&S10PiperHardwareInterface::JointHandler, this,
                      std::placeholders::_1));
        imu_data_sub_ = node_->create_subscription<drdds::msg::ImuData>(
            "/IMU_DATA", rclcpp::SensorDataQoS(),
            std::bind(&S10PiperHardwareInterface::ImuHandler, this,
                      std::placeholders::_1));
        battery_sub_ = node_->create_subscription<drdds::msg::BatteryData>(
            "/BATTERY_DATA", 10,
            std::bind(&S10PiperHardwareInterface::BatteryHandler, this,
                      std::placeholders::_1));
    }

    ~S10PiperHardwareInterface() override { Stop(); }

    void Start() override {
        if (started_.exchange(true)) return;
        spin_thread_ = std::thread([this] { rclcpp::spin(node_); });
        if (piper_mode_ != "virtual") piper_.Start();

        {
            std::unique_lock<std::mutex> lock(data_mutex_);
            if (!data_cv_.wait_for(lock, std::chrono::seconds(10),
                                   [this] {
                                       return joints_ready_ && imu_ready_;
                                   })) {
                throw std::runtime_error(
                    "S10 DDS feedback timeout: check /JOINTS_DATA and /IMU_DATA");
            }
        }
        CorrectWrappedOffsets();
        WaitForStableFeedback(std::chrono::seconds(5));

        if (piper_mode_ != "virtual") {
            piper_real_for_session_ =
                piper_.WaitForState(std::chrono::milliseconds(1500)) &&
                piper_.StateFresh();
        }
        if (!piper_real_for_session_) {
            std::cerr
                << "[S10/Piper/ISOLATED] no fresh Piper state at startup; "
                   "IMU MOVE_J leg/wheel policy and keyboard remain "
                   "active. This session uses rate-limited virtual arm q/dq "
                   "and sends no physical arm motion. Restart this controller "
                   "after Piper communication recovers to reattach the arm."
                << std::endl;
        } else {
            if (!piper_.TrainingIKSupported()) {
                // Piper is optional to S10 locomotion. An unavailable or old
                // bridge must not abort keyboard input or the leg/wheel RL
                // loop. Keep the 22-DoF observation contract alive locally,
                // but never send blind motion to the physical arm.
                piper_real_for_session_ = false;
                std::cerr
                    << "[S10/Piper/ISOLATED] Orin bridge does not advertise "
                       "training_j123_locked_j456_v1; IMU MOVE_J "
                       "leg/wheel policy continues with virtual arm q/dq and "
                       "no physical arm commands. Restart after updating the "
                       "Orin bridge."
                    << std::endl;
            } else {
#ifdef S10_PIPER_REALTIME
                std::cout << "[S10/Piper] realtime decoupled MOVE_J: "
                             "Y controls J1 directly; X/Z use J2/J3; "
                             "J4-J6 locked at training Home."
                          << std::endl;
#else
                std::cout << "[S10/Piper] MOVE_J layered: constrained J1-J3 XYZ IK, J4-J6 locked at training Home; "
                             "measured q/dq enter the policy; legs/wheels use RL, arm uses official JointCtrl."
                          << std::endl;
#endif
            }
        }

        if (!read_only_) {
            WaitForCommandSubscriber(std::chrono::seconds(5));
            ResetAndEnableMotors();
            WaitForEnabledStatus(std::chrono::seconds(2));
            std::cout << "[S10] DDS feedback and command path ready; all 16 "
                         "motor status words are enabled." << std::endl;
        } else {
            std::cout << "[S10] read-only feedback check passed; no command or "
                         "control-word sample will be published." << std::endl;
        }
    }

    void Stop() override {
        if (!started_.exchange(false)) return;
        if (!read_only_) {
            try {
                MatXf damping = MatXf::Zero(s10_real_detail::kS10Dof, 5);
                for (int i = 0; i < s10_real_detail::kS10Dof; ++i)
                    damping(i, 2) = (i % 4 == 3) ? 1.0f : 2.0f;
                PublishS10(damping);
            } catch (...) {
            }
        }
        if (piper_mode_ != "virtual") piper_.Stop(hold_piper_on_exit_);
        if (rclcpp::ok()) rclcpp::shutdown();
        if (spin_thread_.joinable()) spin_thread_.join();
    }

    void PrepareShutdown() override {
        if (!started_ || read_only_) return;

        // Start Piper homing before crouching S10 so both transitions proceed
        // together. Stop() waits for Orin's verified home-disable completion.
        if (piper_real_for_session_ && !piper_.RequestHomeDisable()) {
            std::cerr << "[Piper] Cannot send Ctrl+C homing request; "
                         "check the Orin connection.\n";
        }

        static constexpr std::array<float, s10_real_detail::kS10Dof>
            kCrouchTarget{{
                 0.08f, -1.20f,  2.40f, 0.0f,
                -0.08f, -1.20f,  2.40f, 0.0f,
                 0.08f,  1.20f, -2.40f, 0.0f,
                -0.08f,  1.20f, -2.40f, 0.0f}};
        constexpr int kSteps = 400;       // 2.0 s at 200 Hz
        constexpr int kSettleSteps = 60;  // 0.3 s at low Idle gains

        VecXf initial;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            initial = s10_position_;
        }

        auto make_command = [&](float phase, bool low_gain) {
            MatXf command = MatXf::Zero(
                s10_real_detail::kWholeBodyDof, 5);
            for (int i = 0; i < s10_real_detail::kS10Dof; ++i) {
                const bool wheel = i % 4 == 3;
                if (wheel) {
                    command(i, 0) = 0.0f;
                    command(i, 1) = 0.0f;
                    command(i, 2) = 0.2f;
                    command(i, 3) = 0.0f;
                    continue;
                }
                command(i, 0) = low_gain ? 20.0f : 40.0f;
                command(i, 1) = initial(i) +
                    phase * (kCrouchTarget[i] - initial(i));
                command(i, 2) = low_gain ? 0.5f : 1.0f;
            }
            return command;
        };

        try {
            for (int step = 1; step <= kSteps; ++step) {
                const float alpha = static_cast<float>(step) / kSteps;
                const float phase = 0.5f *
                    (1.0f - std::cos(static_cast<float>(M_PI) * alpha));
                PublishS10(make_command(phase, false).topRows(
                    s10_real_detail::kS10Dof));
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
            for (int step = 0; step < kSettleSteps; ++step) {
                PublishS10(make_command(1.0f, true).topRows(
                    s10_real_detail::kS10Dof));
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
            std::cout << "[SAFE] S10 crouch complete; leaving SDK control."
                      << std::endl;
        } catch (const std::exception& error) {
            std::cerr << "[SAFE] Crouch interrupted: " << error.what()
                      << "; falling back to damping." << std::endl;
        }
    }

    double GetInterfaceTimeStamp() override {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return timestamp_;
    }

    VecXf GetJointPosition() override {
        VecXf result(s10_real_detail::kWholeBodyDof);
        std::lock_guard<std::mutex> lock(data_mutex_);
        result.head(s10_real_detail::kS10Dof) = s10_position_;
        result.tail(s10_real_detail::kPiperDof) = piper_real_for_session_
            ? piper_.Position() : virtual_arm_position_;
        return result;
    }

    VecXf GetJointVelocity() override {
        VecXf result = VecXf::Zero(s10_real_detail::kWholeBodyDof);
        std::lock_guard<std::mutex> lock(data_mutex_);
        result.head(s10_real_detail::kS10Dof) = s10_velocity_;
        if (piper_real_for_session_) {
            if (piper_.StateFresh()) {
                result.tail(s10_real_detail::kPiperDof) = piper_.Velocity();
            }
        } else {
            result.tail(s10_real_detail::kPiperDof) = virtual_arm_velocity_;
        }
        return result;
    }

    VecXf GetJointTorque() override {
        VecXf result = VecXf::Zero(s10_real_detail::kWholeBodyDof);
        std::lock_guard<std::mutex> lock(data_mutex_);
        result.head(s10_real_detail::kS10Dof) = s10_torque_;
        return result;
    }

    Vec3f GetImuRpy() override {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return rpy_;
    }
    Vec3f GetImuAcc() override {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return acceleration_;
    }
    Vec3f GetImuOmega() override {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return angular_velocity_;
    }
    VecXf GetContactForce() override { return VecXf::Zero(4); }

    bool PiperReadyForLayeredRL() const override {
        if (!piper_real_for_session_) return true;
        // An optional Piper fault never prevents S10 leg/wheel inference.
        // Direct joint control resumes from measured feedback after recovery.
        if (!piper_.StateFresh()) return true;

        return PiperAtTrainingHome();
    }

    void SetPiperPolicyControl(bool active) override {
        piper_policy_control_ = active;
        if (!active) {
            // A later Z/reconnect must start MOVE_J from current measurement,
            // never from an old target preceding Cartesian IK.
            piper_activation_requested_ = false;
            piper_recovering_ = true;
            last_piper_cartesian_send_time_ = Clock::time_point{};
        }
    }

    void SetPiperCartesianCommand(const Vec3f& offset,
                                  float wrist_yaw_rad = 0.0f,
                                  bool exact_home = false) override {
        if (!offset.allFinite() || !std::isfinite(wrist_yaw_rad))
            throw std::runtime_error("Invalid layered Cartesian offset");
        if (read_only_ || !piper_real_for_session_ || !piper_policy_control_)
            return;
        if (!piper_.StateFresh()) {
            if (!piper_isolation_logged_) {
                std::cerr << "[S10/Piper] link unavailable: MOVE_J IK arm isolated; "
                             "S10 legs/wheels continue.\n";
                piper_isolation_logged_ = true;
            }
            return;
        }
        if (!piper_.TrainingIKSupported()) {
            if (!piper_isolation_logged_) {
                std::cerr << "[Piper] Training J1-J3 wrist-lock profile absent; "
                             "arm targets blocked. Restart the updated Orin service.\n";
                piper_isolation_logged_ = true;
            }
            return;
        }
        const auto now = Clock::now();
#ifdef S10_PIPER_REALTIME
        constexpr auto kPiperCartesianSendPeriod =
            std::chrono::milliseconds(10);
#else
        constexpr auto kPiperCartesianSendPeriod =
            std::chrono::milliseconds(20);
#endif
        if (last_piper_cartesian_send_time_.time_since_epoch().count() != 0 &&
            now-last_piper_cartesian_send_time_ < kPiperCartesianSendPeriod)
            return;
        if (!piper_.SendTrainingIKOffset(offset, wrist_yaw_rad, exact_home)) {
            piper_isolation_logged_ = true;
            return;
        }
        last_piper_cartesian_send_time_ = now;
        piper_isolation_logged_ = false;
    }

    void SetPiperGripperCommand(int command) override {
        if (command == 0) return;  // Never move the gripper without a keypress.
        if (command != 1 && command != 2)
            throw std::runtime_error("Invalid binary Piper gripper command");
        if (read_only_ || !piper_real_for_session_ || !piper_policy_control_ ||
            !piper_.StateFresh()) return;
        const auto now = Clock::now();
        if (command == last_gripper_command_ &&
            now-last_gripper_send_time_ < std::chrono::milliseconds(200)) return;
        if (piper_.SendGripperCommand(command == 1)) {
            last_gripper_command_ = command;
            last_gripper_send_time_ = now;
        }
    }

    bool PiperAtTrainingHome() const override {
        // Virtual/auto-fallback sessions have no physical Piper to wait for.
        if (!piper_real_for_session_) return true;
        // Stale feedback cannot prove the joint-space Home transition settled.
        if (!piper_.StateFresh()) return false;

        static constexpr std::array<float, 6> kTrainingHome{
            0.0f, 2.54f, -1.26f, 0.0f, -1.22f, 0.0f};
        constexpr float kHomeToleranceRad =
            5.0f * static_cast<float>(M_PI / 180.0);
        constexpr float kSettledVelocityRadS = 0.12f;
        const VecXf q = piper_.Position();
        const VecXf dq = piper_.Velocity();
        for (int i = 0; i < s10_real_detail::kPiperDof; ++i) {
            if (!std::isfinite(q(i)) || !std::isfinite(dq(i)) ||
                std::abs(q(i) - kTrainingHome[i]) > kHomeToleranceRad ||
                std::abs(dq(i)) > kSettledVelocityRadS) {
                return false;
            }
        }
        return true;
    }

    void SetJointCommand(
        Eigen::Matrix<float, Eigen::Dynamic, 5> input) override {
        if (input.rows() != s10_real_detail::kWholeBodyDof ||
            input.cols() != 5 || !input.allFinite()) {
            throw std::runtime_error("invalid S10+Piper 22x5 command");
        }
        joint_cmd_ = input;
        if (read_only_) return;

        long long joint_age_ms = -1;
        long long imu_age_ms = -1;
        if (!FeedbackFresh(joint_age_ms, imu_age_ms)) {
            throw std::runtime_error(
                "S10 feedback stale (joint age=" +
                std::to_string(joint_age_ms) + " ms, IMU age=" +
                std::to_string(imu_age_ms) + " ms)");
        }

        PublishS10(input.topRows(s10_real_detail::kS10Dof));

        const auto arm = input.bottomRows(s10_real_detail::kPiperDof);
        const bool arm_requested =
            arm.col(0).cwiseAbs().maxCoeff() > 1.0e-4f;
        if (!piper_real_for_session_) {
            if (arm_requested) {
                std::lock_guard<std::mutex> lock(data_mutex_);
                UpdateVirtualArm(arm.col(1));
            }
            return;
        }
        // During RL the real arm has ONE owner: SetPiperCartesianCommand.
        // Never send actor joint actions after a constrained-IK MOVE_J packet.
        // Z/startup and shutdown retain the original MOVE_J/home paths.
        if (piper_policy_control_) return;
        if (!piper_.StateFresh()) {
            piper_activation_requested_ = false;
            piper_recovering_ = true;
            if (!piper_isolation_logged_) {
                std::cerr << "[S10/Piper] Piper feedback stale; arm isolated "
                             "while S10 continues." << std::endl;
                piper_isolation_logged_ = true;
            }
            return;
        }
        if (!arm_requested) return;

        static constexpr std::array<float, 6> kLower{
            -2.618f, 0.0f, -2.967f, -1.745f, -1.22f, -2.0944f};
        static constexpr std::array<float, 6> kUpper{
             2.618f, 3.14f, 0.0f, 1.745f, 1.22f, 2.0944f};
        const auto now = Clock::now();
        if (!piper_activation_requested_) {
            // The first MOVE_J packet holds measured joints, not a stale
            // policy target. This also applies after a transport interruption.
            VecXf measured = piper_.Position();
            for (int i = 0; i < s10_real_detail::kPiperDof; ++i) {
                if (!std::isfinite(measured(i)) ||
                    measured(i) < kLower[i] - 0.02f ||
                    measured(i) > kUpper[i] + 0.02f) {
                    if (!piper_isolation_logged_) {
                        std::cerr << "[S10/Piper] measured joints outside "
                                     "policy limits; run normal Piper homing."
                                  << std::endl;
                        piper_isolation_logged_ = true;
                    }
                    return;
                }
                measured(i) = std::clamp(measured(i), kLower[i], kUpper[i]);
            }
            if (!piper_.SendJointTarget(measured, true)) return;
            last_piper_target_ = measured;
            last_piper_target_time_ = now;
            piper_activation_requested_ = true;
            piper_recovering_ = true;
            piper_isolation_logged_ = false;
            return;
        }
        if (!piper_.Enabled()) {
            if (now - last_piper_target_time_ > std::chrono::seconds(2)) {
                piper_activation_requested_ = false;
            }
            return;
        }

        constexpr auto kSendPeriod = std::chrono::milliseconds(20);
        if (now - last_piper_target_time_ < kSendPeriod) return;
        const float dt = std::clamp(
            std::chrono::duration<float>(now - last_piper_target_time_).count(),
            0.001f, 0.040f);
        // Z only: 2x the previous original candidate's 0.4 rad/s.
        // At 50 Hz, <=0.016 rad is below the existing Orin startup cap
        // of 0.024 rad/frame. Runtime IK and exit-zero speeds are unchanged.
        const float frame_step = z_startup_profile::ArmFrameStep(dt);
        VecXf target = last_piper_target_;
        const bool transition = !piper_policy_control_ || piper_recovering_;
        float largest_gap = 0.0f;
        for (int i = 0; i < s10_real_detail::kPiperDof; ++i) {
            const float requested = std::clamp(arm(i, 1), kLower[i], kUpper[i]);
            largest_gap = std::max(largest_gap, std::abs(requested - target(i)));
            if (transition) {
                target(i) += std::clamp(requested - target(i),
                    -frame_step, frame_step);
            } else {
                // V26 already applies EMA 0.2 and 0.020 rad/policy-step.
                // Pass this decoded target without a second speed filter.
                target(i) = requested;
            }
        }
        if (!piper_.SendJointTarget(target, true)) {
            piper_activation_requested_ = false;
            return;
        }
        piper_isolation_logged_ = false;
        if (piper_recovering_ && largest_gap <= 0.020f) {
            piper_recovering_ = false;
        }
        last_piper_target_ = target;
        last_piper_target_time_ = now;
    }
};

}  // namespace interface
