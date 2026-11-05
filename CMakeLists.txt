cmake_minimum_required(VERSION 3.16)
project(rl_deploy LANGUAGES CXX)
# Root CMake is simulation-only. Hardware is built on 103 via ./build.sh real.
# It never initializes a robot SDK or connects to physical devices.
add_subdirectory(sim)
