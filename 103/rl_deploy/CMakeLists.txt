cmake_minimum_required(VERSION 3.10)
project(rl_deploy)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# GCC 10+ emits verbose ABI compatibility notes for Eigen vector return values
# on aarch64. They are informational (not warnings or errors), but colcon counts
# any stderr output as a noisy build. Suppress only that diagnostic category.
add_compile_options(-fPIC -Wno-psabi)

set(BUILD_PLATFORM "x86" CACHE STRING "ONNX Runtime platform")
option(BUILD_HARDWARE "Build the official S10 ROS 2/DDS hardware backend" ON)
if(NOT BUILD_HARDWARE)
    message(FATAL_ERROR "Build simulation with sim/build.sh from the project root")
endif()
option(S10_PIPER_REALTIME "Send layered Piper Cartesian targets at 100 Hz" OFF)
option(S10_VR_CONTROL "Use the S10-compatible Orin PXREA TCP relay" OFF)
set(S10_POLICY_FILE "policy.onnx" CACHE STRING
    "ONNX file under policy/ppo")
set(S10_POLICY_NAME "policy" CACHE STRING
    "Policy name displayed by the state machine")

get_filename_component(WORKSPACE_DIR . ABSOLUTE)
set(THIRD_PARTY ${WORKSPACE_DIR}/third_party)
set(ONNX_RUNTIME_ROOT "${THIRD_PARTY}/onnxruntime/${BUILD_PLATFORM}")

if(NOT EXISTS "${ONNX_RUNTIME_ROOT}/include/onnxruntime_cxx_api.h")
    message(FATAL_ERROR "ONNX Runtime not found under ${ONNX_RUNTIME_ROOT}")
endif()

include_directories(
    ${THIRD_PARTY}/eigen
    ${ONNX_RUNTIME_ROOT}/include
    types
    utils
    interface/robot
    interface/user_command
    state_machine
    state_machine/parameters
    run_policy
)
link_directories(${ONNX_RUNTIME_ROOT}/lib)

if(BUILD_HARDWARE)
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(drdds REQUIRED)
endif()

add_subdirectory(interface)

file(GLOB_RECURSE STATE_MACHINE_SRC "state_machine/*.cpp")
file(GLOB_RECURSE RUN_POLICY_SRC "run_policy/*.cpp")
add_executable(rl_deploy main.cpp ${STATE_MACHINE_SRC} ${RUN_POLICY_SRC})
target_compile_definitions(rl_deploy PRIVATE
    S10_POLICY_FILE="${S10_POLICY_FILE}"
    S10_POLICY_NAME="${S10_POLICY_NAME}"
    S10_PIPER_LOW035
    S10_PIPER_V25
)
if(S10_PIPER_REALTIME)
    target_compile_definitions(rl_deploy PRIVATE S10_PIPER_REALTIME)
endif()
if(S10_VR_CONTROL)
    target_compile_definitions(rl_deploy PRIVATE S10_VR_CONTROL)
endif()
if(BUILD_HARDWARE)
    target_compile_definitions(rl_deploy PRIVATE BUILD_HARDWARE)
    ament_target_dependencies(rl_deploy rclcpp drdds)
else()
    target_compile_definitions(rl_deploy PRIVATE BUILD_SIMULATION)
endif()
target_link_libraries(rl_deploy
    interface pthread m rt dl stdc++ onnxruntime
)
if(NOT BUILD_HARDWARE)
    target_link_libraries(rl_deploy dw)
endif()
set(S10_RUNTIME_RPATH "${ONNX_RUNTIME_ROOT}/lib")
set_target_properties(rl_deploy PROPERTIES
    BUILD_RPATH "${S10_RUNTIME_RPATH}"
    INSTALL_RPATH "\$ORIGIN;${S10_RUNTIME_RPATH}"
)

message(STATUS "S10 policy: ${S10_POLICY_NAME} (${S10_POLICY_FILE})")
if(S10_PIPER_REALTIME)
    message(STATUS "Piper layered target transport: realtime 100 Hz")
else()
    message(STATUS "Piper layered target transport: standard 50 Hz")
endif()
if(S10_VR_CONTROL)
    message(STATUS "User command: S10-compatible Orin PXREA TCP relay")
else()
    message(STATUS "User command: keyboard")
endif()
if(BUILD_HARDWARE)
    message(STATUS "Backend: real S10 ROS 2/DDS + optional Piper TCP")
    install(TARGETS rl_deploy DESTINATION lib/${PROJECT_NAME})
    install(DIRECTORY policy/ DESTINATION share/${PROJECT_NAME}/policy)
    ament_package()
else()
    message(STATUS "Backend: S10+Piper MuJoCo UDP simulation")
endif()
