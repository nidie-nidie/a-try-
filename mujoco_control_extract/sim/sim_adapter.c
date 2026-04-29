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

void SimController_Init(void)
{
    memset(&INS, 0, sizeof(INS));

    INS.ins_flag = 1;
    chassis_move.start_flag = 1;
    chassis_move.mode = CHASSIS_SAFE;
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

void SimController_Step(float dt)
{
    sim_dt_s = dt;
    (void)sim_dt_s;

    ChassisL_feedback_update();
    ChassisR_feedback_update();

    ChassisR_control_loop();
    ChassisL_control_loop();
}

void SimController_GetOutput(SimControllerOutput *output)
{
    if (output == 0)
    {
        return;
    }

    // Match the signs used by ChassisL_task/ChassisR_task when sending CAN torque.
    output->joint_torque[0] = -left.torque_set[1];
    output->joint_torque[1] = -left.torque_set[0];
    output->joint_torque[2] = -right.torque_set[0];
    output->joint_torque[3] = -right.torque_set[1];

    output->wheel_torque[0] = left.wheel_T;
    output->wheel_torque[1] = right.wheel_T;
}
