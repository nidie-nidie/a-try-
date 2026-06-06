cmake_minimum_required(VERSION 3.16)
project(mujoco_control_leg_debug C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

include("${CMAKE_CURRENT_LIST_DIR}/../cmake/ThirdParty.cmake")

find_package(OpenGL REQUIRED)

set(CONTROL_ROOT "${CMAKE_CURRENT_LIST_DIR}/../mujoco_control_extract")

add_library(leg_debug_controller STATIC
    "${CONTROL_ROOT}/Application/Task/Src/Chassis_Task.c"
    "${CONTROL_ROOT}/Application/Task/Src/ChassisL_Task.c"
    "${CONTROL_ROOT}/Application/Task/Src/ChassisR_Task.c"
    "${CONTROL_ROOT}/Components/Algorithm/Src/VMC_Calc.c"
    "${CONTROL_ROOT}/Components/Algorithm/Src/LPF.c"
    "${CONTROL_ROOT}/Components/Controller/Src/PID.c"
    "${CONTROL_ROOT}/Components/Device/Src/Motor.c"
    "${CONTROL_ROOT}/Components/Lib/Src/User_Lib.c"
    "${CONTROL_ROOT}/sim/sim_adapter.c"
    "${CONTROL_ROOT}/sim/port/sim_platform.c"
)

target_compile_definitions(leg_debug_controller PRIVATE SIM_MUJOCO)
target_link_libraries(leg_debug_controller PUBLIC m)
target_include_directories(leg_debug_controller PUBLIC
    "${CONTROL_ROOT}/sim"
    "${CONTROL_ROOT}/sim/port"
    "${CONTROL_ROOT}/Application/Task/Inc"
    "${CONTROL_ROOT}/Application/RobotParam/Inc"
    "${CONTROL_ROOT}/Application/TypeDef/Inc"
    "${CONTROL_ROOT}/Components/Algorithm/Inc"
    "${CONTROL_ROOT}/Components/Controller/Inc"
    "${CONTROL_ROOT}/Components/Device/Inc"
    "${CONTROL_ROOT}/Components/Lib/Inc"
)

add_executable(leg_debug
    src/main.c
    src/debug_draw.c
    src/control_task_space.c
    src/control_joint_space.c
    src/model_helpers.c
    src/logging.c
)

target_compile_definitions(leg_debug PRIVATE
    SIM_MUJOCO
    LEG_DEBUG_DEFAULT_MODEL_PATH="${CMAKE_CURRENT_LIST_DIR}/models/wheel_leg_debug.xml"
    LEG_DEBUG_DEFAULT_COLLISION_MODEL_PATH="${CMAKE_CURRENT_LIST_DIR}/models/wheel_leg_debug_collision.xml"
)

target_include_directories(leg_debug PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/include"
    "${CONTROL_ROOT}/sim"
    "${CONTROL_ROOT}/sim/port"
    "${CONTROL_ROOT}/Application/Task/Inc"
    "${CONTROL_ROOT}/Application/RobotParam/Inc"
    "${CONTROL_ROOT}/Application/TypeDef/Inc"
    "${CONTROL_ROOT}/Components/Algorithm/Inc"
    "${CONTROL_ROOT}/Components/Controller/Inc"
    "${CONTROL_ROOT}/Components/Device/Inc"
    "${CONTROL_ROOT}/Components/Lib/Inc"
)

target_link_libraries(leg_debug PRIVATE leg_debug_controller m OpenGL::GL)
rm_target_link_glfw(leg_debug)
rm_target_link_mujoco(leg_debug)
