#pragma once

#include "user_command_interface.h"
#include "piper_safe_workspace.hpp"

#include <algorithm>
#include <arpa/inet.h>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <mutex>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <string>
#include <sys/select.h>
#include <sys/socket.h>
#include <termios.h>
#include <thread>
#include <unistd.h>

using namespace types;

namespace interface {

// Wire-compatible with orin/pico_bridge/s10_piper_vr_bridge.py. The bridge receives
// Pico input directly from PXREA; no browser or XLeVR service is involved.
// struct.pack("<7fi", vx, vy, wz, arm_x, arm_y, arm_z, gripper, mode).
#pragma pack(push, 1)
struct VRCommandPacket {
    float vx;
    float vy;
    float wz;
    float arm_x;
    float arm_y;
    float arm_z;
    float gripper;
    int32_t target_mode;
};
#pragma pack(pop)
static_assert(sizeof(VRCommandPacket) == 32, "VR packet must be 32 bytes");

class VrCommandInterface final : public UserCommandInterface {
private:
    static constexpr float kHomeX = 0.42f;
    static constexpr float kHomeY = 0.0f;
    static constexpr float kHomeZ = 0.05f;
    static constexpr float kGripperMaxM = 0.080f;
    // VR base safety: keep the existing +/-0.8 maximum speed, but prevent an
    // instantaneous forward/reverse command.  Braking is intentionally faster
    // than acceleration; a sign change must reach zero before accelerating in
    // the opposite direction.
    static constexpr float kForwardAccelPerSecond = 0.8f;
    static constexpr float kForwardDecelPerSecond = 1.6f;
    static constexpr float kNominalVrPacketSeconds = 0.01f;
    static constexpr float kMaxVrPacketSeconds = 0.05f;
    static constexpr int32_t kVrHomeRequest = -1001;
    static constexpr auto kVrTimeout = std::chrono::milliseconds(300);

    UserCommand usr_cmd_{};
    std::atomic<bool> running_{false};
    std::thread vr_thread_;
    std::thread keyboard_thread_;
    std::mutex command_mutex_;
    std::atomic<int> socket_fd_{-1};
    std::atomic<int> client_fd_{-1};
    bool vr_online_ = false;
    bool vr_base_enabled_ = false;
    bool vr_mode_enabled_ = false;
    bool vr_home_active_ = false;
    std::chrono::steady_clock::time_point home_until_{};
    std::chrono::steady_clock::time_point last_packet_{};
    std::chrono::steady_clock::time_point last_forward_update_{};

    static int EnvPort() {
        const char* value = std::getenv("S10_PIPER_VR_PORT");
        if (!value || !*value) return 8089;
        try {
            const int port = std::stoi(value);
            return port > 0 && port <= 65535 ? port : 8089;
        } catch (...) {
            return 8089;
        }
    }

    static bool EnvFlag(const char* name) {
        const char* value = std::getenv(name);
        return value && (std::string(value) == "1" ||
                         std::string(value) == "true" ||
                         std::string(value) == "TRUE");
    }

    static bool FinitePacket(const VRCommandPacket& packet) {
        return std::isfinite(packet.vx) && std::isfinite(packet.vy) &&
               std::isfinite(packet.wz) && std::isfinite(packet.arm_x) &&
               std::isfinite(packet.arm_y) && std::isfinite(packet.arm_z) &&
               std::isfinite(packet.gripper);
    }

    static float DecodeGripperMetres(float value) {
        // The reference bridge sends legacy micrometre-style values
        // (0..70000). Also accept a direct SI opening in metres (0..0.08).
        if (value > 1.0f) value *= 1.0e-6f;
        return std::clamp(value, 0.0f, kGripperMaxM);
    }

    void ZeroBaseLocked() {
        usr_cmd_.forward_vel_scale = 0.0f;
        usr_cmd_.side_vel_scale = 0.0f;
        usr_cmd_.turnning_vel_scale = 0.0f;
        last_forward_update_ = {};
    }

    float LimitForwardVelocityLocked(
        float requested, std::chrono::steady_clock::time_point now) {
        requested = std::clamp(requested, -0.8f, 0.8f);
        float current = std::clamp(usr_cmd_.forward_vel_scale, -0.8f, 0.8f);

        float dt = kNominalVrPacketSeconds;
        if (last_forward_update_.time_since_epoch().count() != 0) {
            dt = std::chrono::duration<float>(now - last_forward_update_).count();
            dt = std::clamp(dt, 0.0f, kMaxVrPacketSeconds);
        }
        last_forward_update_ = now;

        // Do not cross zero in one update.  A requested reversal first becomes
        // a braking request to exactly zero; only a later packet may accelerate
        // in the opposite direction.
        const bool reversing = current * requested < 0.0f;
        const float target = reversing ? 0.0f : requested;
        const bool braking = std::abs(target) < std::abs(current);
        const float rate = braking ? kForwardDecelPerSecond
                                   : kForwardAccelPerSecond;
        const float max_step = rate * dt;
        const float delta = std::clamp(target - current, -max_step, max_step);
        float limited = current + delta;
        if ((target - current > 0.0f && limited > target) ||
            (target - current < 0.0f && limited < target)) {
            limited = target;
        }
        if (std::abs(limited) < 1.0e-6f) limited = 0.0f;
        return limited;
    }

    void ApplyPacket(const VRCommandPacket& packet) {
        std::lock_guard<std::mutex> lock(command_mutex_);
        vr_online_ = true;
        const auto now = std::chrono::steady_clock::now();
        last_packet_ = now;

        // S10/Piper operator workspace: Y +/-0.50 m, synchronized with Orin.
        const auto safe = piper_safe_workspace::Clamp(
            Vec3f(packet.arm_x, packet.arm_y, packet.arm_z));
        usr_cmd_.arm_pos_des[0] = safe.x();
        usr_cmd_.arm_pos_des[1] = safe.y();
        usr_cmd_.arm_pos_des[2] = safe.z();
        usr_cmd_.piper_gripper_command =
            DecodeGripperMetres(packet.gripper) >= 0.04f ? 1 : 2;

        if (vr_base_enabled_) {
            usr_cmd_.forward_vel_scale =
                LimitForwardVelocityLocked(packet.vx, now);
            usr_cmd_.side_vel_scale = 0.0f;
            usr_cmd_.turnning_vel_scale = std::clamp(packet.wz, -1.0f, 1.0f);
        }
        const bool home_requested = packet.target_mode == kVrHomeRequest;
        if (home_requested)
            home_until_ = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(200);
        if (home_requested) {
            ZeroBaseLocked();
            usr_cmd_.arm_pos_des[0] = kHomeX;
            usr_cmd_.arm_pos_des[1] = kHomeY;
            usr_cmd_.arm_pos_des[2] = kHomeZ;
            if (!vr_home_active_) {
                std::cout << "\n[VR/Pico] B: arm Home requested" << std::endl;
            }
        }
        vr_home_active_ = home_requested;
        if (vr_mode_enabled_ &&
            (packet.target_mode == RobotMotionState::StandingUp ||
             packet.target_mode == RobotMotionState::RLControlMode ||
             packet.target_mode == RobotMotionState::JointDamping)) {
            usr_cmd_.target_mode = packet.target_mode;
        }
    }

    void MarkDisconnected(const char* reason) {
        std::lock_guard<std::mutex> lock(command_mutex_);
        if (!vr_online_) return;
        vr_online_ = false;
        vr_home_active_ = false;
        if (vr_base_enabled_) ZeroBaseLocked();
        // Freeze the last arm target. Returning Home on a network dropout
        // would itself create an unwanted motion.
        std::cerr << "\n[VR] " << reason
                  << ": arm target frozen; VR base command stopped"
                  << std::endl;
    }

    void ReceiveLoop() {
        socket_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (socket_fd_ < 0) {
            std::cerr << "[VR] TCP socket creation failed: "
                      << std::strerror(errno) << std::endl;
            running_ = false;
            return;
        }

        int reuse = 1;
        setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_ANY);
        address.sin_port = htons(static_cast<uint16_t>(EnvPort()));
        if (::bind(socket_fd_, reinterpret_cast<sockaddr*>(&address),
                   sizeof(address)) != 0) {
            std::cerr << "[VR] TCP bind failed on port " << EnvPort() << ": "
                      << std::strerror(errno) << std::endl;
            ::close(socket_fd_);
            socket_fd_ = -1;
            running_ = false;
            return;
        }
        if (::listen(socket_fd_, 1) != 0) {
            std::cerr << "[VR] TCP listen failed: " << std::strerror(errno)
                      << std::endl;
            ::close(socket_fd_);
            socket_fd_ = -1;
            running_ = false;
            return;
        }

        while (running_) {
            fd_set listener_set;
            FD_ZERO(&listener_set);
            FD_SET(socket_fd_, &listener_set);
            timeval accept_timeout{0, 100000};
            const int ready = select(socket_fd_ + 1, &listener_set, nullptr,
                                     nullptr, &accept_timeout);
            if (!running_) break;
            if (ready <= 0) continue;

            sockaddr_in source{};
            socklen_t source_length = sizeof(source);
            client_fd_ = ::accept(
                socket_fd_, reinterpret_cast<sockaddr*>(&source), &source_length);
            if (client_fd_ < 0) continue;
            int no_delay = 1;
            setsockopt(client_fd_, IPPROTO_TCP, TCP_NODELAY,
                       &no_delay, sizeof(no_delay));
            timeval receive_timeout{0, 100000};
            setsockopt(client_fd_, SOL_SOCKET, SO_RCVTIMEO,
                       &receive_timeout, sizeof(receive_timeout));
            char source_ip[INET_ADDRSTRLEN]{};
            inet_ntop(AF_INET, &source.sin_addr, source_ip, sizeof(source_ip));
            std::cout << "\n[VR] TCP connected from " << source_ip << ':'
                      << ntohs(source.sin_port) << std::endl;

            VRCommandPacket packet{};
            size_t received = 0;
            auto last_valid_packet = std::chrono::steady_clock::now();
            while (running_) {
                const ssize_t count = recv(
                    client_fd_, reinterpret_cast<char*>(&packet) + received,
                    sizeof(packet) - received, 0);
                const auto now = std::chrono::steady_clock::now();
                if (now - last_valid_packet > kVrTimeout)
                    MarkDisconnected("stream timeout (>300 ms)");
                if (count > 0) {
                    received += static_cast<size_t>(count);
                    if (received == sizeof(packet)) {
                        if (FinitePacket(packet)) {
                            ApplyPacket(packet);
                            last_valid_packet = now;
                        }
                        else std::cerr << "\n[VR] ignored non-finite packet"
                                       << std::endl;
                        received = 0;
                    }
                    continue;
                }
                if (count == 0) {
                    MarkDisconnected("TCP disconnected");
                    break;
                }
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    continue;
                }
                MarkDisconnected("TCP receive error");
                break;
            }
            ::close(client_fd_);
            client_fd_ = -1;
        }
    }

    void KeyboardLoop() {
        termios old_term{};
        if (tcgetattr(STDIN_FILENO, &old_term) != 0) return;
        termios raw_term = old_term;
        raw_term.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &raw_term);

        while (running_) {
            fd_set fds;
            FD_ZERO(&fds);
            FD_SET(STDIN_FILENO, &fds);
            timeval timeout{0, 50000};
            if (select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &timeout) <= 0)
                continue;
            char key = 0;
            if (read(STDIN_FILENO, &key, 1) != 1) continue;
            if (key >= 'A' && key <= 'Z') key += ('a' - 'A');

            std::lock_guard<std::mutex> lock(command_mutex_);
            usr_cmd_.soft_stop_flag = false;
            if (key == 'z') {
                ZeroBaseLocked();
                usr_cmd_.target_mode = RobotMotionState::StandingUp;
                std::cout << "\n[VR/KB] Z: Stand Up requested" << std::endl;
            } else if (key == 'c') {
                ZeroBaseLocked();
                usr_cmd_.target_mode = RobotMotionState::RLControlMode;
                std::cout << "\n[VR/KB] C: RL requested" << std::endl;
            } else if (key == 'r') {
                ZeroBaseLocked();
                usr_cmd_.target_mode = RobotMotionState::JointDamping;
                std::cout << "\n[VR/KB] R: joint damping requested" << std::endl;
            } else if (key == ' ') {
                ZeroBaseLocked();
                std::cout << "\r[VR/KB] base stopped                    "
                          << std::flush;
            } else if (key == 'h') {
                ZeroBaseLocked();
                usr_cmd_.arm_pos_des[0] = kHomeX;
                usr_cmd_.arm_pos_des[1] = kHomeY;
                usr_cmd_.arm_pos_des[2] = kHomeZ;
                home_until_ = std::chrono::steady_clock::now() +
                              std::chrono::milliseconds(200);
                std::cout << "\n[VR/KB] H: arm Home requested; recenter the "
                             "right controller before resuming"
                          << std::endl;
            } else if (key == 'n') {
                usr_cmd_.piper_gripper_command = 1;
            } else if (key == 'm') {
                usr_cmd_.piper_gripper_command = 2;
            } else if (!vr_base_enabled_ &&
                       usr_cmd_.target_mode == RobotMotionState::RLControlMode) {
                if (key == 'w') {
                    usr_cmd_.turnning_vel_scale = 0.0f;
                    usr_cmd_.forward_vel_scale = std::min(
                        0.8f, usr_cmd_.forward_vel_scale + 0.05f);
                } else if (key == 's') {
                    usr_cmd_.turnning_vel_scale = 0.0f;
                    usr_cmd_.forward_vel_scale = std::max(
                        -0.8f, usr_cmd_.forward_vel_scale - 0.05f);
                } else if (key == 'q') {
                    usr_cmd_.forward_vel_scale = 0.0f;
                    usr_cmd_.turnning_vel_scale = std::min(
                        1.0f, usr_cmd_.turnning_vel_scale + 0.2f);
                } else if (key == 'e') {
                    usr_cmd_.forward_vel_scale = 0.0f;
                    usr_cmd_.turnning_vel_scale = std::max(
                        -1.0f, usr_cmd_.turnning_vel_scale - 0.2f);
                }
            }
        }
        tcsetattr(STDIN_FILENO, TCSANOW, &old_term);
    }

public:
    VrCommandInterface() {
        usr_cmd_ = UserCommand{};
        usr_cmd_.arm_pos_des[0] = kHomeX;
        usr_cmd_.arm_pos_des[1] = kHomeY;
        usr_cmd_.arm_pos_des[2] = kHomeZ;
        usr_cmd_.piper_wrist_yaw_rad = 0.0f;
        vr_base_enabled_ = EnvFlag("S10_PIPER_VR_BASE");
        vr_mode_enabled_ = EnvFlag("S10_PIPER_VR_MODE");
    }

    ~VrCommandInterface() { Stop(); }

    void Start() override {
        if (running_.exchange(true)) return;
        vr_thread_ = std::thread(&VrCommandInterface::ReceiveLoop, this);
        keyboard_thread_ = std::thread(&VrCommandInterface::KeyboardLoop, this);
        std::cout << "[VR] listening TCP 0.0.0.0:" << EnvPort()
                  << "; Orin PXREA -> S10/Piper layered XYZ target; Y +/-0.50 m"
                  << (vr_base_enabled_ ? "; VR base ENABLED" :
                      "; base remains keyboard W/S/Q/E")
                  << std::endl;
        if (vr_base_enabled_) {
            std::cout << "[VR] base X slew limit: accel "
                      << kForwardAccelPerSecond << "/s, decel "
                      << kForwardDecelPerSecond
                      << "/s; reversal must pass through zero"
                      << std::endl;
        }
        std::cout << "[VR] keyboard: Z stand, C RL, R damping, H Home, "
                     "N/M gripper, SPACE base stop"
                  << std::endl;
    }

    void Stop() override {
        running_ = false;
        const int client = client_fd_.load();
        if (client >= 0) {
            // Tell the Orin bridge that this 103 strategy session has ended.
            // The following shutdown also lets the bridge detect an EOF if
            // this best-effort byte cannot be sent.
            const char reset_session = 'R';
            ::send(client, &reset_session, 1, MSG_DONTWAIT | MSG_NOSIGNAL);
            ::shutdown(client, SHUT_RDWR);
        }
        if (socket_fd_ >= 0) {
            ::shutdown(socket_fd_, SHUT_RDWR);
        }
        if (vr_thread_.joinable()) vr_thread_.join();
        if (keyboard_thread_.joinable()) keyboard_thread_.join();
        const int listener = socket_fd_.exchange(-1);
        if (listener >= 0) ::close(listener);
    }

    UserCommand GetUserCommand() override {
        std::lock_guard<std::mutex> lock(command_mutex_);
        usr_cmd_.soft_stop_flag = std::chrono::steady_clock::now() < home_until_;
        return usr_cmd_;
    }

    void SetMotionStateFeedback(const MotionStateFeedback& feedback) override {
        std::lock_guard<std::mutex> lock(command_mutex_);
        msfb_ = feedback;
    }
};

}  // namespace interface
