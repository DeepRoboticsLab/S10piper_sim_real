cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-fPIC)

include_directories(./robot ./user_command)

# User-command interfaces are header-only. The arm is handled by Orin TCP;
# no local arm SDK or gamepad implementation is linked here.
add_library(interface INTERFACE)
target_link_libraries(interface INTERFACE pthread m rt dl stdc++)
