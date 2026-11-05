
#include "state_machine.hpp"
#include "shutdown_signal.hpp"
#include <csignal>
#ifdef BUILD_HARDWARE
    #include <rclcpp/rclcpp.hpp>
#endif
#if defined(BUILD_SIMULATION) && defined(ENABLE_BACKWARD_STACKTRACE) && \
    __has_include("backward.hpp")
    #define BACKWARD_HAS_DW 1
    #include "backward.hpp"
    namespace backward{
        backward::SignalHandling sh;
    }
#endif
using namespace types;

MotionStateFeedback StateBase::msfb_ = MotionStateFeedback();

int main(){
#ifdef BUILD_HARDWARE
    rclcpp::init(0, nullptr, rclcpp::InitOptions(),
                 rclcpp::SignalHandlerOptions::None);
    std::signal(SIGINT, shutdown_signal::Handle);
    std::signal(SIGTERM, shutdown_signal::Handle);
#endif
    try {
        StateMachine state_machine(RobotType::S10);
        state_machine.Run();
#ifdef BUILD_HARDWARE
        if (rclcpp::ok()) rclcpp::shutdown();
#endif
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "[FATAL] " << error.what() << std::endl;
#ifdef BUILD_HARDWARE
        if (rclcpp::ok()) rclcpp::shutdown();
#endif
        return 1;
    }
}
