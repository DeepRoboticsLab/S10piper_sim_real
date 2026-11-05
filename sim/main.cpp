#include "state_machine.hpp"
#include <csignal>
#include <cstdlib>

MotionStateFeedback StateBase::msfb_ = MotionStateFeedback();

int main() {
    // Unconditional overrides protect against inherited real-robot settings.
    setenv("PIPER_SERVER_IP", "127.0.0.1", 1);
    setenv("S10_PIPER_MODE", "real", 1); // real protocol, simulated device ONLY
    setenv("S10_REAL_READ_ONLY", "0", 1);
    std::signal(SIGINT, shutdown_signal::Handle);
    std::signal(SIGTERM, shutdown_signal::Handle);
    std::cout << "[SIM ONLY] Unchanged S10/Piper controller; DDS is LOCAL UDP emulation; "
                 "Piper is loopback simulation. No robot service/CAN access.\n";
    try {
        StateMachine machine(RobotType::S10);
        machine.Run();
        rclcpp::shutdown();
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "[SIM FATAL] " << error.what() << '\n';
        rclcpp::shutdown();
        return 1;
    }
}
