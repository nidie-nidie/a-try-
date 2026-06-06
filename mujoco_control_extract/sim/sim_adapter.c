#include "sim_adapter.h"

#include <math.h>
#include <string.h>

#include "ChassisL_Task.h"
#include "ChassisR_Task.h"
#include "Chassis_Task.h"
#include "INS_Task.h"
#include "Motor.h"
#include "robot_param.h"

extern INS_t INS;

static float sim_dt_s = 0.003f;
static float sim_stand_l0_pitch = INIT_L0_PITCH;
static int sim_drive_forward = 0;
static float sim_position_hold_blend = 1.0f;
static int sim_yaw_lock = 1;
static int sim_airborne = 0;
static int sim_airborne_pose_valid = 0;
static int sim_jump_compression_enabled = 1;
static int sim_jump_flight_active = 0;
static int sim_jump_has_been_airborne = 0;
static float sim_jump_compress_elapsed = 0.0f;
static float sim_jump_compress_hold_elapsed = 0.0f;
static float sim_airborne_joint_target[4];
static float sim_airborne_pose_kp = MUJOCO_JUMP_TUCK_KP;
static float sim_airborne_pose_kd = MUJOCO_JUMP_TUCK_KD;
static float sim_airborne_pose_torque_limit = MUJOCO_JUMP_TUCK_TORQUE_LIMIT;

static float sim_mit_torque(float position_set, float velocity_set, float kp, float kd, float torque_ff,
                            const DM_Motor_Info_Typedef *motor)
{
    return kp * (position_set - motor->Data.Position) +
           kd * (velocity_set - motor->Data.Velocity) +
           torque_ff;
}

static float sim_clamp_float(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static float sim_airborne_pose_torque(int joint_index)
{
    const float torque = sim_mit_torque(sim_airborne_joint_target[joint_index],
                                        0.0f,
                                        sim_airborne_pose_kp,
                                        sim_airborne_pose_kd,
                                        0.0f,
                                        chassis_move.joint_motor[joint_index]);
    return sim_clamp_float(torque,
                           -sim_airborne_pose_torque_limit,
                           sim_airborne_pose_torque_limit);
}

static void sim_update_jump_compression(float dt)
{
    if (chassis_move.jump_flag != 1 || chassis_move.jump_flag2 != 1)
    {
        return;
    }

    sim_jump_compress_elapsed += dt;
    mujoco_jump_compress_l0_set -= mujoco_jump_compress_rate * dt;
    if (mujoco_jump_compress_l0_set < mujoco_jump_compress_target)
    {
        mujoco_jump_compress_l0_set = mujoco_jump_compress_target;
    }

    if (mujoco_jump_compress_l0_set <= mujoco_jump_compress_target + 1.0e-4f &&
        left.L0 <= mujoco_jump_compress_target + mujoco_jump_compress_tolerance &&
        right.L0 <= mujoco_jump_compress_target + mujoco_jump_compress_tolerance)
    {
        sim_jump_compress_hold_elapsed += dt;
    }
    else
    {
        sim_jump_compress_hold_elapsed = 0.0f;
    }

    if (sim_jump_compress_hold_elapsed >= mujoco_jump_compress_hold_time)
    {
        jump_time_l = 0;
        jump_time_r = 0;
        chassis_move.jump_flag = 2;
        chassis_move.jump_flag2 = 2;
    }
    else if (sim_jump_compress_elapsed >= mujoco_jump_compress_timeout)
    {
        jump_time_l = 0;
        jump_time_r = 0;
        chassis_move.jump_flag = 0;
        chassis_move.jump_flag2 = 0;
        sim_jump_flight_active = 0;
        sim_jump_has_been_airborne = 0;
        chassis_move.leg_set = INIT_LEG_LENGTH;
        chassis_move.last_leg_set = INIT_LEG_LENGTH;
    }
}

static void sim_update_jump_takeoff(void)
{
    if (sim_airborne &&
        (chassis_move.jump_flag == 2 || chassis_move.jump_flag2 == 2))
    {
        jump_time_l = 0;
        jump_time_r = 0;
        chassis_move.jump_flag = 3;
        chassis_move.jump_flag2 = 3;
    }
}

static void sim_apply_native_wheel_balance(SimControllerOutput *output)
{
    const float pitch_term = MUJOCO_WHEEL_BALANCE_PITCH_KP * (INS.Pitch - MUJOCO_WHEEL_BALANCE_PITCH_TARGET) +
                             MUJOCO_WHEEL_BALANCE_PITCH_KD * INS.Gyro[1];
    const float pos_term = sim_drive_forward
                               ? 0.0f
                               : MUJOCO_WHEEL_BALANCE_POS_KP * sim_position_hold_blend *
                                     (chassis_move.x_filter - chassis_move.x_set);
    const float vel_term = MUJOCO_WHEEL_BALANCE_VEL_KD * (chassis_move.v_filter - chassis_move.v_set);
    const float drive_term = MUJOCO_WHEEL_BALANCE_DRIVE_KFF * chassis_move.v_set;
    const float yaw_term = sim_yaw_lock
                               ? MUJOCO_WHEEL_BALANCE_YAW_KP * (INS.Yaw - chassis_move.turn_set) +
                                     MUJOCO_WHEEL_BALANCE_YAW_KD * INS.Gyro[2]
                               : 0.0f;
    const float common = sim_clamp_float(pitch_term + pos_term + vel_term + drive_term,
                                         -MUJOCO_WHEEL_BALANCE_LIMIT,
                                         MUJOCO_WHEEL_BALANCE_LIMIT);

    output->wheel_torque[0] = common - yaw_term;
    output->wheel_torque[1] = common + yaw_term;
}

static void sim_apply_stand_joint_targets(void)
{
    float phi1_phi4_l[2];
    float phi1_phi4_r[2];

    CalcPhi1AndPhi4(sim_stand_l0_pitch, chassis_move.leg_set, phi1_phi4_l);
    CalcPhi1AndPhi4(sim_stand_l0_pitch, chassis_move.leg_set, phi1_phi4_r);

    if (isnan(phi1_phi4_l[0]) || isnan(phi1_phi4_l[1]) ||
        isnan(phi1_phi4_r[0]) || isnan(phi1_phi4_r[1]))
    {
        return;
    }

    left.position_set[0] = -theta_transform(phi1_phi4_l[1], -J0_ANGLE_OFFSET, J0_DIRECTION, 1);
    left.position_set[1] = -theta_transform(phi1_phi4_l[0], -J1_ANGLE_OFFSET, J1_DIRECTION, 1);
    right.position_set[0] = -theta_transform(phi1_phi4_r[0], -J2_ANGLE_OFFSET, J2_DIRECTION, 1);
    right.position_set[1] = -theta_transform(phi1_phi4_r[1], -J3_ANGLE_OFFSET, J3_DIRECTION, 1);
}

void SimController_SetStandL0Pitch(float l0_pitch)
{
    sim_stand_l0_pitch = l0_pitch;
}

void SimController_Init(void)
{
    memset(&INS, 0, sizeof(INS));

    sim_airborne = 0;
    sim_jump_flight_active = 0;
    sim_jump_has_been_airborne = 0;
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
    ConsoleStandUp();
    sim_apply_stand_joint_targets();
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

void SimController_SetDriveContext(int drive_forward, float position_hold_blend, int yaw_lock)
{
    sim_drive_forward = drive_forward != 0;
    sim_position_hold_blend = sim_clamp_float(position_hold_blend, 0.0f, 1.0f);
    sim_yaw_lock = yaw_lock != 0;
}

void SimController_SetJumpThrust(float thrust_ff)
{
    mujoco_jump_thrust_ff = thrust_ff;
}

void SimController_SetJumpPitchTp(float target, float kp, float kd, float limit)
{
    mujoco_jump_pitch_target = target;
    mujoco_jump_pitch_tp_kp = kp;
    mujoco_jump_pitch_tp_kd = kd;
    mujoco_jump_pitch_tp_limit = limit;
}

void SimController_SetJumpCompression(int enabled,
                                      float target,
                                      float rate,
                                      float support_scale,
                                      float tolerance,
                                      float hold_time,
                                      float timeout)
{
    sim_jump_compression_enabled = enabled != 0;
    mujoco_jump_compress_target = sim_clamp_float(target,
                                                   MIN_LEG_LENGTH,
                                                   INIT_LEG_LENGTH);
    mujoco_jump_compress_rate = fabsf(rate);
    mujoco_jump_compress_support_scale = sim_clamp_float(support_scale, 0.0f, 1.0f);
    mujoco_jump_compress_tolerance = fabsf(tolerance);
    mujoco_jump_compress_hold_time = fmaxf(0.0f, hold_time);
    mujoco_jump_compress_timeout = fmaxf(mujoco_jump_compress_hold_time, timeout);
}

void SimController_SetJumpLegSwing(float offset, float kp, float kd, float limit)
{
    mujoco_jump_leg_swing_offset = offset;
    mujoco_jump_leg_swing_kp = kp;
    mujoco_jump_leg_swing_kd = kd;
    mujoco_jump_leg_swing_limit = fabsf(limit);
}

void SimController_SetJumpExtendEndMargin(float margin)
{
    mujoco_jump_extend_end_margin = margin;
}

void SimController_SetAirbornePoseTarget(const float joint_pos[4])
{
    if (joint_pos == 0)
    {
        sim_airborne_pose_valid = 0;
        return;
    }

    memcpy(sim_airborne_joint_target, joint_pos, sizeof(sim_airborne_joint_target));
    sim_airborne_pose_valid = 1;
}

void SimController_SetAirbornePoseGains(float kp, float kd, float torque_limit)
{
    sim_airborne_pose_kp = kp;
    sim_airborne_pose_kd = kd;
    sim_airborne_pose_torque_limit = fabsf(torque_limit);
}

void SimController_SetAirborne(int airborne)
{
    sim_airborne = airborne != 0;
    if (sim_jump_flight_active && sim_airborne)
    {
        sim_jump_has_been_airborne = 1;
    }
    else if (sim_jump_flight_active &&
             sim_jump_has_been_airborne &&
             chassis_move.jump_flag == 0 &&
             chassis_move.jump_flag2 == 0)
    {
        sim_jump_flight_active = 0;
        sim_jump_has_been_airborne = 0;
    }
}

void SimController_SetMode(int mode)
{
    chassis_move.mode = (ChassisMode_e)mode;
}

int SimController_RequestJump(void)
{
    if (sim_jump_flight_active ||
        chassis_move.jump_flag != 0 ||
        chassis_move.jump_flag2 != 0)
    {
        return 0;
    }

    jump_time_l = 0;
    jump_time_r = 0;
    sim_jump_flight_active = 1;
    sim_jump_has_been_airborne = 0;
    sim_jump_compress_elapsed = 0.0f;
    sim_jump_compress_hold_elapsed = 0.0f;
    mujoco_jump_compress_l0_set =
        fmaxf(mujoco_jump_compress_target, fmaxf(left.L0, right.L0));
    chassis_move.jump_flag = sim_jump_compression_enabled ? 1 : 2;
    chassis_move.jump_flag2 = sim_jump_compression_enabled ? 1 : 2;
    return 1;
}

int SimController_IsJumping(void)
{
    return sim_jump_flight_active ||
           chassis_move.jump_flag != 0 ||
           chassis_move.jump_flag2 != 0;
}

void SimController_Step(float dt)
{
    sim_dt_s = dt;
    (void)sim_dt_s;

    ChassisL_feedback_update();
    ChassisR_feedback_update();
    sim_update_jump_compression(dt);
    sim_update_jump_takeoff();

    ChassisR_control_loop();
    ChassisL_control_loop();

    ChassisConsole();
    sim_apply_stand_joint_targets();
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
        // MuJoCo ground-init is already upright. During the timed STAND_UP
        // window, use normal VMC support so the closed-chain leg does not
        // drift into the folded branch before switching to SAFE.
        output->joint_torque[0] = sim_mit_torque(0.0f, 0.0f, NORMAL_POS_KP, NORMAL_POS_KD, -left.torque_set[1],
                                                 chassis_move.joint_motor[0]);
        output->joint_torque[1] = sim_mit_torque(0.0f, 0.0f, NORMAL_POS_KP, NORMAL_POS_KD, -left.torque_set[0],
                                                 chassis_move.joint_motor[1]);
        output->joint_torque[2] = sim_mit_torque(0.0f, 0.0f, NORMAL_POS_KP, NORMAL_POS_KD, -right.torque_set[0],
                                                 chassis_move.joint_motor[2]);
        output->joint_torque[3] = sim_mit_torque(0.0f, 0.0f, NORMAL_POS_KP, NORMAL_POS_KD, -right.torque_set[1],
                                                 chassis_move.joint_motor[3]);
        output->wheel_torque[0] = 0.0f;
        output->wheel_torque[1] = 0.0f;
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
        // Mirror the real control task: MIT zero pose/velocity targets with
        // torque feedforward from the VMC output.
        output->joint_torque[0] = sim_mit_torque(0.0f, 0.0f, NORMAL_POS_KP, NORMAL_POS_KD, -left.torque_set[1],
                                                 chassis_move.joint_motor[0]);
        output->joint_torque[1] = sim_mit_torque(0.0f, 0.0f, NORMAL_POS_KP, NORMAL_POS_KD, -left.torque_set[0],
                                                 chassis_move.joint_motor[1]);
        output->joint_torque[2] = sim_mit_torque(0.0f, 0.0f, NORMAL_POS_KP, NORMAL_POS_KD, -right.torque_set[0],
                                                 chassis_move.joint_motor[2]);
        output->joint_torque[3] = sim_mit_torque(0.0f, 0.0f, NORMAL_POS_KP, NORMAL_POS_KD, -right.torque_set[1],
                                                 chassis_move.joint_motor[3]);
        output->wheel_torque[0] = left.wheel_T;
        output->wheel_torque[1] = right.wheel_T;
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

    if (sim_airborne_pose_valid &&
        (sim_airborne || chassis_move.jump_flag == 3 || chassis_move.jump_flag2 == 3))
    {
        for (int i = 0; i < 4; ++i)
        {
            output->joint_torque[i] = sim_airborne_pose_torque(i);
        }
    }

    if (chassis_move.mode == CHASSIS_CALIBRATE)
    {
        output->wheel_torque[0] = left.wheel_T;
        output->wheel_torque[1] = right.wheel_T;
    }
    else if (chassis_move.mode == CHASSIS_STAND_UP || chassis_move.mode == CHASSIS_SAFE)
    {
        sim_apply_native_wheel_balance(output);
    }
}
