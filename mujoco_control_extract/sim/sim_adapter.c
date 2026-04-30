#include "sim_adapter.h"

#include <string.h>

#include "ChassisL_Task.h"
#include "ChassisR_Task.h"
#include "Chassis_Task.h"
#include "INS_Task.h"
#include "Motor.h"
#include "robot_param.h"

extern INS_t INS;

static float sim_dt_s = 0.003f;

static float sim_mit_torque(float position_set, float velocity_set, float kp, float kd, float torque_ff,
                            const DM_Motor_Info_Typedef *motor)
{
    return kp * (position_set - motor->Data.Position) +
           kd * (velocity_set - motor->Data.Velocity) +
           torque_ff;
}

void SimController_Init(void)
{
    memset(&INS, 0, sizeof(INS));

    INS.ins_flag = 1;
    chassis_move.start_flag = 1;
    chassis_move.mode = CHASSIS_STAND_UP;
    chassis_move.leg_set = INIT_LEG_LENGTH;
    chassis_move.last_leg_set = INIT_LEG_LENGTH;
    chassis_move.roll_set = INIT_ROLL;
    chassis_move.turn_set = 0.0f;
    chassis_move.x_set = 0.0f;
    chassis_move.v_set = 0.0f;

    ChassisL_init();
    ChassisR_init();
    Pensation_init();
}

void SimController_SetState(const SimControllerState *state)
{
    if (state == 0)
    {
        return;
    }

    for (int i = 0; i < 4; ++i)
    {
        DM_8009_Motor[i].Data.Position = state->joint_pos[i];
        DM_8009_Motor[i].Data.Velocity = state->joint_vel[i];
        DM_8009_Motor[i].Data.Torque = state->joint_torque_fdb[i];
    }

    for (int i = 0; i < 2; ++i)
    {
        LK_9025_Motor[i].Data.Velocity = state->wheel_vel[i];
    }

    INS.Roll = state->roll;
    INS.Pitch = state->pitch;
    INS.Yaw = state->yaw;
    INS.YawTotalAngle = state->yaw;

    for (int i = 0; i < 3; ++i)
    {
        INS.Gyro[i] = state->gyro[i];
        INS.MotionAccel_b[i] = state->accel_body[i];
        INS.MotionAccel_n[i] = state->accel_world[i];
    }

    chassis_move.x_filter = state->body_x;
    chassis_move.v_filter = state->body_v;
}

void SimController_SetCommand(float v_set, float x_set, float leg_set, float roll_set, float yaw_set)
{
    chassis_move.v_set = v_set;
    chassis_move.x_set = x_set;
    chassis_move.leg_set = leg_set;
    chassis_move.roll_set = roll_set;
    chassis_move.turn_set = yaw_set;
}

void SimController_SetMode(int mode)
{
    chassis_move.mode = (ChassisMode_e)mode;
}

void SimController_Step(float dt)
{
    sim_dt_s = dt;
    (void)sim_dt_s;

    ChassisL_feedback_update();
    ChassisR_feedback_update();

    ChassisR_control_loop();
    ChassisL_control_loop();

    ChassisConsole();
}

void SimController_GetOutput(SimControllerOutput *output)
{
    if (output == 0)
    {
        return;
    }

    memset(output, 0, sizeof(*output));

    if (chassis_move.start_flag != 1)
    {
        return;
    }

    switch (chassis_move.mode)
    {
    case CHASSIS_STAND_UP:
        output->joint_torque[0] = sim_mit_torque(left.position_set[0], 0.0f, DEBUG_POS_KP, DEBUG_POS_KD, 0.0f,
                                                 chassis_move.joint_motor[0]);
        output->joint_torque[1] = sim_mit_torque(left.position_set[1], 0.0f, DEBUG_POS_KP, DEBUG_POS_KD, 0.0f,
                                                 chassis_move.joint_motor[1]);
        output->joint_torque[2] = sim_mit_torque(right.position_set[0], 0.0f, DEBUG_POS_KP, DEBUG_POS_KD, 0.0f,
                                                 chassis_move.joint_motor[2]);
        output->joint_torque[3] = sim_mit_torque(right.position_set[1], 0.0f, DEBUG_POS_KP, DEBUG_POS_KD, 0.0f,
                                                 chassis_move.joint_motor[3]);
        break;

    case CHASSIS_CALIBRATE:
        output->joint_torque[0] = sim_mit_torque(0.0f, left.velocity_set[0], 0.0f, CALIBRATE_VEL_KD, 0.0f,
                                                 chassis_move.joint_motor[0]);
        output->joint_torque[1] = sim_mit_torque(0.0f, left.velocity_set[1], 0.0f, CALIBRATE_VEL_KD, 0.0f,
                                                 chassis_move.joint_motor[1]);
        output->joint_torque[2] = sim_mit_torque(0.0f, right.velocity_set[0], 0.0f, CALIBRATE_VEL_KD, 0.0f,
                                                 chassis_move.joint_motor[2]);
        output->joint_torque[3] = sim_mit_torque(0.0f, right.velocity_set[1], 0.0f, CALIBRATE_VEL_KD, 0.0f,
                                                 chassis_move.joint_motor[3]);
        break;

    case CHASSIS_SAFE:
        output->joint_torque[0] = sim_mit_torque(0.0f, 0.0f, NORMAL_POS_KP, NORMAL_POS_KD, -left.torque_set[1],
                                                 chassis_move.joint_motor[0]);
        output->joint_torque[1] = sim_mit_torque(0.0f, 0.0f, NORMAL_POS_KP, NORMAL_POS_KD, -left.torque_set[0],
                                                 chassis_move.joint_motor[1]);
        output->joint_torque[2] = sim_mit_torque(0.0f, 0.0f, NORMAL_POS_KP, NORMAL_POS_KD, -right.torque_set[0],
                                                 chassis_move.joint_motor[2]);
        output->joint_torque[3] = sim_mit_torque(0.0f, 0.0f, NORMAL_POS_KP, NORMAL_POS_KD, -right.torque_set[1],
                                                 chassis_move.joint_motor[3]);
        break;

    case CHASSIS_OFF:
    default:
        output->joint_torque[0] = sim_mit_torque(0.0f, 0.0f, 0.0f, ZERO_FORCE_VEL_KD, 0.0f,
                                                 chassis_move.joint_motor[0]);
        output->joint_torque[1] = sim_mit_torque(0.0f, 0.0f, 0.0f, ZERO_FORCE_VEL_KD, 0.0f,
                                                 chassis_move.joint_motor[1]);
        output->joint_torque[2] = sim_mit_torque(0.0f, 0.0f, 0.0f, ZERO_FORCE_VEL_KD, 0.0f,
                                                 chassis_move.joint_motor[2]);
        output->joint_torque[3] = sim_mit_torque(0.0f, 0.0f, 0.0f, ZERO_FORCE_VEL_KD, 0.0f,
                                                 chassis_move.joint_motor[3]);
        break;
    }

    if (chassis_move.mode == CHASSIS_STAND_UP ||
        chassis_move.mode == CHASSIS_SAFE ||
        chassis_move.mode == CHASSIS_CALIBRATE)
    {
        output->wheel_torque[0] = left.wheel_T;
        output->wheel_torque[1] = right.wheel_T;
    }
}
