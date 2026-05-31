#ifndef MUJOCO_CONTROL_LEG_DEBUG_LEG_DEBUG_H
#define MUJOCO_CONTROL_LEG_DEBUG_LEG_DEBUG_H

#include <mujoco/mujoco.h>

#include "VMC_Calc.h"

enum
{
    LEG_DEBUG_LEG_JOINT_COUNT = 4,
    LEG_DEBUG_WHEEL_COUNT = 2,
    LEG_DEBUG_ACTUATOR_COUNT = 6,
    LEG_DEBUG_BASE_QPOS_SIZE = 7,
    LEG_DEBUG_BASE_QVEL_SIZE = 6,
    LEG_DEBUG_INPUT_COMMAND_SIZE = 192,
    LEG_DEBUG_INIT_KEY_SIZE = 64,
};

/*
 * Local copy of the robot-side parameters used by the MuJoCo leg debug viewer.
 * Keep edits here if you want to tune the viewer without touching
 * mujoco_control_extract/Application/RobotParam/Inc/robot_param.h.
 */
#define LEG_DEBUG_PARAM_FIRMWARE_POS_KP            (20.0)
#define LEG_DEBUG_PARAM_FIRMWARE_POS_KD            (1.0)
#define LEG_DEBUG_PARAM_FIRMWARE_MAX_TORQUE        (20.0)

#define LEG_DEBUG_PARAM_MIN_LEG_LENGTH             (0.01)
#define LEG_DEBUG_PARAM_MAX_LEG_LENGTH             (0.35)

#define LEG_DEBUG_PARAM_MIN_J0_ANGLE               (-0.6)
#define LEG_DEBUG_PARAM_MIN_J1_ANGLE               (-1.8)
#define LEG_DEBUG_PARAM_MIN_J2_ANGLE               (-1.8)
#define LEG_DEBUG_PARAM_MIN_J3_ANGLE               (0.0)
#define LEG_DEBUG_PARAM_MAX_J0_ANGLE               (1.8)
#define LEG_DEBUG_PARAM_MAX_J1_ANGLE               (0.0)
#define LEG_DEBUG_PARAM_MAX_J2_ANGLE               (0.6)
#define LEG_DEBUG_PARAM_MAX_J3_ANGLE               (1.8)

#define LEG_DEBUG_PARAM_J0_ANGLE_OFFSET            (-0.19163715)
#define LEG_DEBUG_PARAM_J1_ANGLE_OFFSET            (0.19163715 + 3.14159265358979323846)
#define LEG_DEBUG_PARAM_J2_ANGLE_OFFSET            (0.19163715 + 3.14159265358979323846)
#define LEG_DEBUG_PARAM_J3_ANGLE_OFFSET            (-0.19163715)

#define LEG_DEBUG_PARAM_J0_DIRECTION               (1)
#define LEG_DEBUG_PARAM_J1_DIRECTION               (1)
#define LEG_DEBUG_PARAM_J2_DIRECTION               (1)
#define LEG_DEBUG_PARAM_J3_DIRECTION               (1)
#define LEG_DEBUG_PARAM_W0_DIRECTION               (1)
#define LEG_DEBUG_PARAM_W1_DIRECTION               (1)

typedef enum
{
    LEG_DEBUG_CONTROL_TASK_SPACE = 0,
    LEG_DEBUG_CONTROL_JOINT_SPACE = 1,
} LegDebugControlMode;

typedef enum
{
    LEG_DEBUG_JOINT_LEFT_FRONT = 0,
    LEG_DEBUG_JOINT_LEFT_REAR = 1,
    LEG_DEBUG_JOINT_RIGHT_REAR = 2,
    LEG_DEBUG_JOINT_RIGHT_FRONT = 3,
} LegDebugJointIndex;

typedef struct
{
    int qpos;
    int qvel;
} LegDebugJointRef;

typedef struct
{
    LegDebugJointRef joint[LEG_DEBUG_LEG_JOINT_COUNT];
    int joint_id[LEG_DEBUG_LEG_JOINT_COUNT];
    LegDebugJointRef wheel[LEG_DEBUG_WHEEL_COUNT];
    int wheel_joint_id[LEG_DEBUG_WHEEL_COUNT];
    int actuator[LEG_DEBUG_ACTUATOR_COUNT];
    int base_body;
    int base_freejoint;
} LegDebugModelMap;

typedef struct
{
    double hip_mid[3];
    double wheel_axis[3];
    double vmc_c[3];
    double l0_world;
} LegDebugLegWorld;

typedef struct
{
    LegDebugControlMode control_mode;
    int control_enabled;
    int paused;
    int scripted_demo;
    int show_debug_draw;
    int use_gravity_ff;
    int selected_joint;

    double target_q[LEG_DEBUG_LEG_JOINT_COUNT];
    double home_target_q[LEG_DEBUG_LEG_JOINT_COUNT];
    double q[LEG_DEBUG_LEG_JOINT_COUNT];
    double qd[LEG_DEBUG_LEG_JOINT_COUNT];
    double last_tau_pd[LEG_DEBUG_LEG_JOINT_COUNT];
    double last_tau_ff[LEG_DEBUG_LEG_JOINT_COUNT];
    double last_tau[LEG_DEBUG_LEG_JOINT_COUNT];
    double joint_ctrl_sign[LEG_DEBUG_LEG_JOINT_COUNT];

    double left_l0_cmd;
    double right_l0_cmd;
    double home_left_l0_cmd;
    double home_right_l0_cmd;
    double left_phi0_cmd;
    double right_phi0_cmd;
    double home_left_phi0_cmd;
    double home_right_phi0_cmd;

    double measured_left_l0_vmc;
    double measured_right_l0_vmc;
    double measured_left_phi0_vmc;
    double measured_right_phi0_vmc;
    double measured_left_theta;
    double measured_right_theta;
    double measured_left_dl0;
    double measured_right_dl0;
    double measured_left_dphi0;
    double measured_right_dphi0;
    double target_left_l0_vmc;
    double target_right_l0_vmc;
    double target_left_phi0_vmc;
    double target_right_phi0_vmc;

    LegDebugLegWorld left_world;
    LegDebugLegWorld right_world;
    vmc_leg_t left_vmc;
    vmc_leg_t right_vmc;

    double joint_kp;
    double joint_kd;
    double max_joint_torque;
    double hold_ff_period;
    double last_hold_ff_time;
    double min_leg_length;
    double max_leg_length;
    double keyboard_length_rate;
    double keyboard_phi0_rate;
    double keyboard_joint_rate;
    double print_period;

    char init_key_name[LEG_DEBUG_INIT_KEY_SIZE];
    char input_command[LEG_DEBUG_INPUT_COMMAND_SIZE];
} LegDebugState;

extern const char *const kLegDebugSlotName[LEG_DEBUG_ACTUATOR_COUNT];
extern const char *const kLegDebugJointShortName[LEG_DEBUG_LEG_JOINT_COUNT];

static const double kLegDebugPi = 3.14159265358979323846;
static const double kLegDebugKeyboardDt = 1.0 / 60.0;
static const double kLegDebugDefaultPrintPeriod = 0.25;
static const double kLegDebugDefaultMaxLegLength = 0.42;
static const double kLegDebugDefaultJointKp = 80.0;
static const double kLegDebugDefaultJointKd = 4.0;
static const double kLegDebugDefaultMaxJointTorque = 80.0;
static const double kLegDebugDefaultHoldFfPeriod = 0.01;
static const double kLegDebugDefaultMaxTp = 6.0;
static const double kLegDebugDefaultLengthRate = 0.20;
static const double kLegDebugDefaultPhi0Rate = 1.20;
static const double kLegDebugDefaultJointRate = 1.50;
static const int kLegDebugViewerWidth = 1200;
static const int kLegDebugViewerHeight = 900;
static const int kLegDebugSceneMaxGeom = 2200;

void leg_debug_clear_input_command(LegDebugState *state);
void leg_debug_append_input_command(LegDebugState *state, const char *command);
const char *leg_debug_control_mode_name(LegDebugControlMode mode);
void leg_debug_log_state(const mjModel *m, const mjData *d, const LegDebugModelMap *map, const LegDebugState *state);

#endif
