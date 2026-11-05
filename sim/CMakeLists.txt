cmake_minimum_required(VERSION 3.16)
project(rl_deploy LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
get_filename_component(PACKAGE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
set(DEPLOY_SOURCE "${PACKAGE_ROOT}/103/rl_deploy")
set(ORT "${DEPLOY_SOURCE}/third_party/onnxruntime/x86")
add_executable(rl_deploy main.cpp "${DEPLOY_SOURCE}/state_machine/parameters/s10_control_parameters.cpp")
# BUILD_HARDWARE selects the UNCHANGED real S10/Piper code path. All drdds/rclcpp
# includes below are local UDP shims, not ROS; no robot/ROS library is linked.
target_include_directories(rl_deploy BEFORE PRIVATE
    simulation/local_ros_shim "${DEPLOY_SOURCE}/third_party/eigen" "${ORT}/include"
    "${DEPLOY_SOURCE}/types" "${DEPLOY_SOURCE}/utils" "${DEPLOY_SOURCE}/interface/robot"
    "${DEPLOY_SOURCE}/interface/user_command" "${DEPLOY_SOURCE}/state_machine"
    "${DEPLOY_SOURCE}/state_machine/parameters" "${DEPLOY_SOURCE}/run_policy")
target_compile_definitions(rl_deploy PRIVATE BUILD_HARDWARE S10_PIPER_REALTIME
    S10_PIPER_LOW035 S10_PIPER_V25
    S10_POLICY_FILE="policy.onnx"
    S10_POLICY_NAME="policy")
target_link_libraries(rl_deploy PRIVATE "${ORT}/lib/libonnxruntime.so" pthread dl)
set_target_properties(rl_deploy PROPERTIES
    BUILD_RPATH "$ORIGIN/../../103/rl_deploy/third_party/onnxruntime/x86/lib"
    INSTALL_RPATH "$ORIGIN/../../103/rl_deploy/third_party/onnxruntime/x86/lib")
install(TARGETS rl_deploy RUNTIME DESTINATION bin)
message(STATUS "S10/Piper layered SIM ONLY: loopback DDS shim; no ROS, SDK mode or CAN backend")
