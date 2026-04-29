cmake_minimum_required(VERSION 3.16)
project(mujoco_pos_debug C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

add_executable(pos_debug
    pos_debug.c
    pos_debug.h
)

target_link_libraries(pos_debug PRIVATE m glfw OpenGL::GL)

if(DEFINED ENV{MUJOCO_ROOT})
    target_include_directories(pos_debug PRIVATE "$ENV{MUJOCO_ROOT}/include")
    target_link_directories(pos_debug PRIVATE "$ENV{MUJOCO_ROOT}/lib")
    target_link_libraries(pos_debug PRIVATE mujoco)
endif()
