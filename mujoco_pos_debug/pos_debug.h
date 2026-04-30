#ifndef MUJOCO_POS_DEBUG_POS_DEBUG_H
#define MUJOCO_POS_DEBUG_POS_DEBUG_H

enum
{
    POS_DEBUG_LEG_JOINT_COUNT = 4,
    POS_DEBUG_WHEEL_COUNT = 2,
    POS_DEBUG_ACTUATOR_COUNT = 6,
    POS_DEBUG_BASE_QPOS_SIZE = 7,
    POS_DEBUG_BASE_QVEL_SIZE = 6,
    POS_DEBUG_INPUT_COMMAND_SIZE = 160,
    POS_DEBUG_INIT_KEY_SIZE = 64,
};

typedef struct
{
    int qpos;
    int qvel;
} JointRef;

typedef struct
{
    JointRef joint[POS_DEBUG_LEG_JOINT_COUNT];
    int joint_id[POS_DEBUG_LEG_JOINT_COUNT];
    JointRef wheel[POS_DEBUG_WHEEL_COUNT];
    int actuator[POS_DEBUG_ACTUATOR_COUNT];
    int base_body;
    int base_freejoint;
} ModelMap;

typedef struct
{
    double target[POS_DEBUG_LEG_JOINT_COUNT];
    double last_tau[POS_DEBUG_LEG_JOINT_COUNT];
    double left_leg_length;
    double right_leg_length;
    double visual_left_leg_length;
    double visual_right_leg_length;
    double left_l0_pitch;
    double right_l0_pitch;
    double kp;
    double kd;
    double max_torque;
    double min_leg_length;
    double max_leg_length;
    double min_l0_pitch;
    double max_l0_pitch;
    double hang_z;
    double print_period;
    double saved_base_qpos[POS_DEBUG_BASE_QPOS_SIZE];
    char init_key_name[POS_DEBUG_INIT_KEY_SIZE];
    char input_command[POS_DEBUG_INPUT_COMMAND_SIZE];
} PosDebugState;

enum
{
    JOINT_LEFT_REAR = 0,
    JOINT_LEFT_FRONT = 1,
    JOINT_RIGHT_FRONT = 2,
    JOINT_RIGHT_REAR = 3,
};

static const char *kDefaultModelPath = "/home/shun/MuJoCoBin/rm_control/MJCF/env.xml";
static const char *kWindowTitle = "mujoco position debug";
static const char *kDefaultInitKeyName = "pos_debug_hang";

static const double kPi = 3.14159265358979323846;
static const double kLegL1 = 0.215;
static const double kLegL2 = 0.258;
static const double kLegL3 = 0.258;
static const double kLegL4 = 0.215;
static const double kJ0Offset = -0.19163715;
static const double kJ1Offset = 0.19163715 + 3.14159265358979323846;
static const double kJ2Offset = 0.19163715 + 3.14159265358979323846;
static const double kJ3Offset = -0.19163715;
static const int kJ0Direction = 1;
static const int kJ1Direction = 1;
static const int kJ2Direction = 1;
static const int kJ3Direction = 1;

static const double kDefaultSimTime = 5.0;
static const double kDefaultKp = 45.0;
static const double kDefaultKd = 4.0;
static const double kDefaultMaxTorque = 18.0;
static const double kDefaultMinLegLength = 0.11;
static const double kDefaultMaxLegLength = 0.35;
static const double kDefaultLegSet = 0.20;
static const double kDefaultL0Pitch = 0.700;
static const double kDefaultMinL0Pitch = -1.00;
static const double kDefaultMaxL0Pitch = 1.00;
static const double kResetLegSet = 0.20;
static const double kResetL0Pitch = 0.700;
static const double kDefaultHangZ = 0.55;
static const double kDefaultPrintPeriod = 0.25;

static const double kActiveJointRangeMin = -1.8;
static const double kActiveJointRangeMax = 1.8;
static const double kKeyboardLengthRate = 0.12;
static const double kKeyboardPitchRate = 0.8;
static const double kKeyboardControlDt = 1.0 / 60.0;

static const int kViewerWidth = 1200;
static const int kViewerHeight = 900;
static const double kCameraDistance = 2.0;
static const double kCameraAzimuth = 135.0;
static const double kCameraElevation = -20.0;
static const double kCameraLookatX = 0.0;
static const double kCameraLookatY = 0.0;
static const double kCameraLookatZ = 0.35;
static const int kSceneMaxGeom = 2000;

#endif
