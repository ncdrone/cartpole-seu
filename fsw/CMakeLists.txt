cmake_minimum_required(VERSION 3.10)
project(cartpole_fsw CXX)
set(CMAKE_CXX_STANDARD 14)   # F´ v4 requires C++14; the core is also C++11-clean
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# Determinism contract (see research/02, 03, 04): no fast-math, no FMA contraction, explicit -O2.
# Same source + same arch + same libm  ==>  bit-identical CSV. Cross-libm runs differ at ULP level only.
add_compile_options(-Wall -Wextra -Wpedantic -Werror -fno-exceptions -fno-rtti -ffp-contract=off)
set(CMAKE_CXX_FLAGS_RELEASE "-O2")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

add_library(fsw STATIC src/Controller.cpp src/Plant.cpp src/Lqr.cpp src/Protect.cpp)
target_include_directories(fsw PUBLIC include)

# One harness source, two builds: protected (FSW_PROTECT=1) and baseline (FSW_PROTECT=0).
add_executable(cartpole_demo src/main.cpp src/Runtime.cpp)
target_compile_definitions(cartpole_demo PRIVATE FSW_PROTECT=1)
target_link_libraries(cartpole_demo fsw)
add_executable(cartpole_baseline src/main.cpp src/Runtime.cpp)
target_compile_definitions(cartpole_baseline PRIVATE FSW_PROTECT=0)
target_link_libraries(cartpole_baseline fsw)

add_executable(test_controller tests/test_controller.cpp src/Runtime.cpp)
target_compile_definitions(test_controller PRIVATE FSW_PROTECT=1)
target_link_libraries(test_controller fsw)

add_executable(test_fdir tests/test_fdir.cpp src/Runtime.cpp)
target_compile_definitions(test_fdir PRIVATE FSW_PROTECT=1)
target_link_libraries(test_fdir fsw)

add_executable(test_protect tests/test_protect.cpp)
target_link_libraries(test_protect fsw)
