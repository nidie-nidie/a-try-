#include "sim_adapter.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rm_third_party/glfw.h"
#include "rm_third_party/mujoco.h"

#include "Chassis_Task.h"
#include "robot_param.h"

typedef struct
{
    int id;
    int qpos;
    int qvel;
} JointRef;

typedef struct
{
    JointRef joint[4];
    JointRef wheel[2];
    int actuator[6];
    int base_body;
    int base_freejoint;
    int rotate_control_frame;
} ModelMap;

typedef enum
{
    DRIVE_STAND = 0,
    DRIVE_FORWARD = 1,
} DriveMode;

typedef struct
{
    DriveMode mode;
    float leg_set;
    float roll_set;
    float forward_speed;
    float current_speed;
    float target_x;
    float position_hold_blend;
    float yaw_hold;
    int yaw_lock;
    int hold_position_pending;
    int hold_yaw_pending;
} DriveCommand;

static mjModel *g_model = 0;
static mjData *g_data = 0;
static mjvCamera g_camera;
static mjvOption g_option;
static mjvScene g_scene;
static mjrContext g_context;
static int g_button_left = 0;
static int g_button_middle = 0;
static int g_button_right = 0;
static int g_paused = 0;
static int g_key_forward = 0;
static int g_key_backward = 0;
static int g_key_leg_up = 0;
static int g_key_leg_down = 0;
static double g_last_x = 0.0;
static double g_last_y = 0.0;
static mjtNum g_joint_ctrl_sign[4] = {1.0, 1.0, 1.0, 1.0};
static mjtNum g_wheel_ctrl_sign[2] = {1.0, 1.0};
static int g_use_wheel_balance_override = 1;
static float g_balance_pitch_target = -0.075f;
static float g_balance_pitch_kp = 75.0f;
static float g_balance_pitch_kd = 18.0f;
static float g_balance_pos_kp = 65.0f;
static float g_balance_vel_kd = 32.0f;
static float g_balance_pos_ramp_time = 2.0f;
static float g_balance_drive_kff = 100.0f;
static float g_balance_yaw_kp = 1.2f;
static float g_balance_yaw_kd = 0.25f;
static float g_balance_wheel_limit = 35.0f;
static float g_keyboard_leg_rate = 0.04f;
static DriveCommand g_drive_command = {
    .mode = DRIVE_STAND,
    .leg_set = INIT_LEG_LENGTH,
    .roll_set = INIT_ROLL,
    .forward_speed = 0.2f,
    .current_speed = 0.0f,
    .target_x = 0.0f,
    .position_hold_blend = 1.0f,
    .yaw_hold = 0.0f,
    .yaw_lock = 1,
    .hold_position_pending = 1,
    .hold_yaw_pending = 1,
};






static int find_required_id(const mjModel *m, int type, const char *name)
{
    int id = mj_name2id(m, type, name);
    if (id < 0)
    {
        fprintf(stderr, "Missing MuJoCo object: %s\n", name);
        exit(2);
    }
    return id;
}








static int find_optional_id(const mjModel *m, int type, const char *name)
{
    return mj_name2id(m, type, name);
}

static int file_exists(const char *path)
{
    FILE *fp = fopen(path, "rb");
    if (fp != 0)
    {
        fclose(fp);
        return 1;
    }
    return 0;
}

static const char *resolve_default_model_path(const char *model_path)
{
    static char prefixed_path[1024];

    if (model_path == 0 || model_path[0] == '\0' || file_exists(model_path))
    {
        return model_path;
    }

    if (model_path[0] != '/')
    {
        int written = snprintf(prefixed_path, sizeof(prefixed_path), "mujoco_control_extract/%s", model_path);
        if (written > 0 && written < (int)sizeof(prefixed_path) && file_exists(prefixed_path))
        {
            return prefixed_path;
        }
    }

    return model_path;
}

static void rotate_xy_into_controller_frame(double in_x, double in_y, float *out_x, float *out_y)
{
    if (out_x != 0)
    {
        *out_x = (float)(-in_y);
    }
    if (out_y != 0)
    {
        *out_y = (float)in_x;
    }
}

static void rotate_vec3_into_controller_frame(const mjtNum in[3], float out[3])
{
    out[0] = (float)(-in[1]);
    out[1] = (float)in[0];
    out[2] = (float)in[2];
}

static void rotate_quat_into_controller_frame(const mjtNum in[4], mjtNum out[4])
{
    const mjtNum c = 0.7071067811865476;
    const mjtNum z90[4] = {c, 0.0, 0.0, -c};

    out[0] = in[0] * z90[0] - in[1] * z90[1] - in[2] * z90[2] - in[3] * z90[3];
    out[1] = in[0] * z90[1] + in[1] * z90[0] + in[2] * z90[3] - in[3] * z90[2];
    out[2] = in[0] * z90[2] - in[1] * z90[3] + in[2] * z90[0] + in[3] * z90[1];
    out[3] = in[0] * z90[3] + in[1] * z90[2] - in[2] * z90[1] + in[3] * z90[0];
}




static JointRef find_joint(const mjModel *m, const char *name)
{
    int id = find_required_id(m, mjOBJ_JOINT, name);
    JointRef ref = {id, m->jnt_qposadr[id], m->jnt_dofadr[id]};
    return ref;
}

static double clamp_double(double value, double min_value, double max_value)
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

static const char *drive_command_name(const DriveCommand *command)
{
    if (command->mode == DRIVE_FORWARD)
    {
        return command->forward_speed < 0.0f ? "backward" : "forward";
    }
    return "stand";
}

static float drive_speed_magnitude(const DriveCommand *command)
{
    const float speed = fabsf(command->forward_speed);
    return speed > 1.0e-4f ? speed : 0.2f;
}

static void queue_drive_mode(DriveCommand *command, DriveMode mode)
{
    if (command->mode != mode)
    {
        const DriveMode previous_mode = command->mode;
        command->mode = mode;
        command->hold_position_pending = 1;
        if (mode == DRIVE_FORWARD || previous_mode == DRIVE_FORWARD)
        {
            command->position_hold_blend = 0.0f;
        }
    }
    command->hold_yaw_pending = 1;
}

static void request_drive_speed(DriveCommand *command, float desired_speed)
{
    if (fabsf(desired_speed) < 1.0e-6f)
    {
        command->forward_speed = drive_speed_magnitude(command);
        queue_drive_mode(command, DRIVE_STAND);
        return;
    }

    if (command->mode != DRIVE_FORWARD || command->forward_speed * desired_speed <= 0.0f)
    {
        command->hold_position_pending = 1;
    }

    queue_drive_mode(command, DRIVE_FORWARD);
    command->forward_speed = desired_speed;
}

static void sync_keyboard_drive_command(GLFWwindow *window, double dt)
{
    const int forward_down = glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS ||
                             glfwGetKey(window, GLFW_KEY_F) == GLFW_PRESS;
    const int backward_down = glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS;
    const int leg_up_down = glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS;
    const int leg_down_down = glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS;
    const char *before = drive_command_name(&g_drive_command);
    float desired_speed = 0.0f;

    g_key_forward = forward_down;
    g_key_backward = backward_down;
    g_key_leg_up = leg_up_down;
    g_key_leg_down = leg_down_down;

    if (forward_down && !backward_down)
    {
        desired_speed = drive_speed_magnitude(&g_drive_command);
    }
    else if (backward_down && !forward_down)
    {
        desired_speed = -drive_speed_magnitude(&g_drive_command);
    }

    request_drive_speed(&g_drive_command, desired_speed);
    if (leg_up_down && !leg_down_down)
    {
        g_drive_command.leg_set += (float)dt * g_keyboard_leg_rate;
    }
    else if (leg_down_down && !leg_up_down)
    {
        g_drive_command.leg_set -= (float)dt * g_keyboard_leg_rate;
    }
    g_drive_command.leg_set = (float)clamp_double(g_drive_command.leg_set,
                                                  MIN_LEG_LENGTH,
                                                  MAX_LEG_LENGTH);

    if (strcmp(before, drive_command_name(&g_drive_command)) != 0)
    {
        printf("Drive mode -> %s at t=%6.3f\n",
               drive_command_name(&g_drive_command),
               g_data ? g_data->time : 0.0);
        fflush(stdout);
    }
}




static void quat_to_euler(const mjtNum q[4], float *roll, float *pitch, float *yaw)
{
    double w = q[0];
    double x = q[1];
    double y = q[2];
    double z = q[3];

    double sinr_cosp = 2.0 * (w * x + y * z);
    double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    double sinp = 2.0 * (w * y - z * x);
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);

    if (sinp > 1.0)
    {
        sinp = 1.0;
    }
    else if (sinp < -1.0)
    {
        sinp = -1.0;
    }

    *roll = (float)atan2(sinr_cosp, cosr_cosp);
    *pitch = (float)asin(sinp);
    *yaw = (float)atan2(siny_cosp, cosy_cosp);
}




static void build_model_map(const mjModel *m, ModelMap *map)
{
    memset(map, 0, sizeof(*map));
    map->rotate_control_frame = 0;

    if (find_optional_id(m, mjOBJ_BODY, "base_link") >= 0)
    {
        map->base_body = find_required_id(m, mjOBJ_BODY, "base_link");
        map->base_freejoint = find_required_id(m, mjOBJ_JOINT, "base_freejoint");

        map->joint[0] = find_joint(m, "left_leg");
        map->joint[1] = find_joint(m, "left_small_leg");
        map->joint[2] = find_joint(m, "right_leg");
        map->joint[3] = find_joint(m, "right_small_leg");
        map->wheel[0] = find_joint(m, "left_wheel");
        map->wheel[1] = find_joint(m, "right_wheel");

        map->actuator[0] = find_required_id(m, mjOBJ_ACTUATOR, "left_leg_motor");
        map->actuator[1] = find_required_id(m, mjOBJ_ACTUATOR, "left_small_leg_motor");
        map->actuator[2] = find_required_id(m, mjOBJ_ACTUATOR, "right_leg_motor");
        map->actuator[3] = find_required_id(m, mjOBJ_ACTUATOR, "right_small_leg_motor");
        map->actuator[4] = find_required_id(m, mjOBJ_ACTUATOR, "left_wheel_motor");
        map->actuator[5] = find_required_id(m, mjOBJ_ACTUATOR, "right_wheel_motor");
        return;
    }



    
    if (find_optional_id(m, mjOBJ_BODY, "base") >= 0)
    {
        map->base_body = find_required_id(m, mjOBJ_BODY, "base");
        map->base_freejoint = find_required_id(m, mjOBJ_JOINT, "base_free");
        map->rotate_control_frame = 1;

        // joint[0]=jIJ 左前/J0/phi4, joint[1]=jIO 左后/J1/phi1
        // joint[2]=jAG 右后/J2/phi1, joint[3]=jAB 右前/J3/phi4
        map->joint[0] = find_joint(m, "jIJ");
        map->joint[1] = find_joint(m, "jIO");
        map->joint[2] = find_joint(m, "jAG");
        map->joint[3] = find_joint(m, "jAB");
        map->wheel[0] = find_joint(m, "jwheel_left");
        map->wheel[1] = find_joint(m, "jwheel_right");

        map->actuator[0] = find_required_id(m, mjOBJ_ACTUATOR, "Left_front_joint_act");
        map->actuator[1] = find_required_id(m, mjOBJ_ACTUATOR, "Left_rear_joint_act");
        map->actuator[2] = find_required_id(m, mjOBJ_ACTUATOR, "Right_rear_joint_act");
        map->actuator[3] = find_required_id(m, mjOBJ_ACTUATOR, "Right_front_joint_act");
        map->actuator[4] = find_required_id(m, mjOBJ_ACTUATOR, "Left_Wheel_act");
        map->actuator[5] = find_required_id(m, mjOBJ_ACTUATOR, "Right_Wheel_act");
        return;
    }

    fprintf(stderr, "Unsupported MuJoCo model: expected body 'base_link' or 'base'.\n");
    exit(2);
}

static int is_closed_chain_model(const mjModel *m)
{
    return find_optional_id(m, mjOBJ_JOINT, "jBE") >= 0 &&
           find_optional_id(m, mjOBJ_JOINT, "jJM") >= 0;
}

static double wrap_pi(double value)
{
    while (value > M_PI)
    {
        value -= 2.0 * M_PI;
    }
    while (value < -M_PI)
    {
        value += 2.0 * M_PI;
    }
    return value;
}

static double sim_theta_transform(double angle, double dangle, int direction)
{
    return wrap_pi((angle + dangle) * (double)direction);
}

static int calc_phi1_phi4(double phi0, double leg_length, double phi1_phi4[2])
{
    const double cos_beta1 = (LEG_L1 * LEG_L1 + leg_length * leg_length - LEG_L2 * LEG_L2) /
                             (2.0 * LEG_L1 * leg_length);
    const double cos_beta2 = (LEG_L4 * LEG_L4 + leg_length * leg_length - LEG_L3 * LEG_L3) /
                             (2.0 * LEG_L4 * leg_length);

    if (cos_beta1 < -1.0 || cos_beta1 > 1.0 || cos_beta2 < -1.0 || cos_beta2 > 1.0)
    {
        return 0;
    }

    phi1_phi4[0] = phi0 + acos(cos_beta1);
    phi1_phi4[1] = phi0 - acos(cos_beta2);
    return 1;
}

static void calc_initial_stand_joint_qpos(mjtNum joint_qpos[4])
{
    const double leg_length = INIT_LEG_LENGTH;
    const double phi0 = INIT_L0_PITCH;
    double phi1_phi4[2] = {0.0, 0.0};

    if (!calc_phi1_phi4(phi0, leg_length, phi1_phi4))
    {
        fprintf(stderr, "Failed to calculate initial stand pose.\n");
        exit(2);
    }

    joint_qpos[0] = -sim_theta_transform(phi1_phi4[1], -J0_ANGLE_OFFSET, J0_DIRECTION);
    joint_qpos[1] = -sim_theta_transform(phi1_phi4[0], -J1_ANGLE_OFFSET, J1_DIRECTION);
    joint_qpos[2] = -sim_theta_transform(phi1_phi4[0], -J2_ANGLE_OFFSET, J2_DIRECTION);
    joint_qpos[3] = -sim_theta_transform(phi1_phi4[1], -J3_ANGLE_OFFSET, J3_DIRECTION);
}

static void zero_fixed_velocities(const mjModel *m, mjData *d, const ModelMap *map)
{
    int base_qvel = m->jnt_dofadr[map->base_freejoint];
    for (int i = 0; i < 6; ++i)
    {
        d->qvel[base_qvel + i] = 0.0;
    }

    for (int i = 0; i < 4; ++i)
    {
        d->qvel[map->joint[i].qvel] = 0.0;
    }
}

static void relax_closed_chain_pose(mjModel *m, mjData *d, const ModelMap *map)
{
    if (!is_closed_chain_model(m))
    {
        return;
    }

    int base_qpos = m->jnt_qposadr[map->base_freejoint];
    mjtNum base_pose[7];
    mjtNum joint_pose[4];
    mjtNum gravity[3];
    mjtNum saved_time = d->time;

    for (int i = 0; i < 7; ++i)
    {
        base_pose[i] = d->qpos[base_qpos + i];
    }
    for (int i = 0; i < 4; ++i)
    {
        joint_pose[i] = d->qpos[map->joint[i].qpos];
    }
    for (int i = 0; i < 3; ++i)
    {
        gravity[i] = m->opt.gravity[i];
        m->opt.gravity[i] = 0.0;
    }

    for (int iter = 0; iter < 2000; ++iter)
    {
        for (int i = 0; i < 7; ++i)
        {
            d->qpos[base_qpos + i] = base_pose[i];
        }
        for (int i = 0; i < 4; ++i)
        {
            d->qpos[map->joint[i].qpos] = joint_pose[i];
        }
        zero_fixed_velocities(m, d, map);

        mj_step(m, d);

        for (int i = 0; i < m->nv; ++i)
        {
            d->qvel[i] *= 0.96;
        }
    }

    for (int i = 0; i < 7; ++i)
    {
        d->qpos[base_qpos + i] = base_pose[i];
    }
    for (int i = 0; i < 4; ++i)
    {
        d->qpos[map->joint[i].qpos] = joint_pose[i];
    }
    memset(d->qvel, 0, sizeof(mjtNum) * m->nv);
    memset(d->ctrl, 0, sizeof(mjtNum) * m->nu);
    d->time = saved_time;

    for (int i = 0; i < 3; ++i)
    {
        m->opt.gravity[i] = gravity[i];
    }
    mj_forward(m, d);
}

static void settle_closed_chain_pose(mjModel *m, mjData *d, const ModelMap *map, const mjtNum target_joint_pose[4])
{
    if (!is_closed_chain_model(m))
    {
        return;
    }

    const int base_qpos = m->jnt_qposadr[map->base_freejoint];
    mjtNum base_pose[7];
    mjtNum start_joint_pose[4];
    mjtNum gravity[3];
    const mjtNum saved_time = d->time;
    const int settle_steps = 3000;

    for (int i = 0; i < 7; ++i)
    {
        base_pose[i] = d->qpos[base_qpos + i];
    }
    for (int i = 0; i < 4; ++i)
    {
        start_joint_pose[i] = d->qpos[map->joint[i].qpos];
    }
    for (int i = 0; i < 3; ++i)
    {
        gravity[i] = m->opt.gravity[i];
        m->opt.gravity[i] = 0.0;
    }

    for (int iter = 0; iter < settle_steps; ++iter)
    {
        const mjtNum alpha = (mjtNum)(iter + 1) / (mjtNum)settle_steps;

        for (int i = 0; i < 7; ++i)
        {
            d->qpos[base_qpos + i] = base_pose[i];
        }
        for (int i = 0; i < 4; ++i)
        {
            d->qpos[map->joint[i].qpos] =
                (1.0 - alpha) * start_joint_pose[i] + alpha * target_joint_pose[i];
        }
        zero_fixed_velocities(m, d, map);

        mj_step(m, d);

        for (int i = 0; i < m->nv; ++i)
        {
            d->qvel[i] *= 0.94;
        }
    }

    for (int i = 0; i < 7; ++i)
    {
        d->qpos[base_qpos + i] = base_pose[i];
    }
    for (int i = 0; i < 4; ++i)
    {
        d->qpos[map->joint[i].qpos] = target_joint_pose[i];
    }
    memset(d->qvel, 0, sizeof(mjtNum) * m->nv);
    memset(d->ctrl, 0, sizeof(mjtNum) * m->nu);
    d->time = saved_time;

    for (int i = 0; i < 3; ++i)
    {
        m->opt.gravity[i] = gravity[i];
    }
    mj_forward(m, d);
}

static void apply_closed_chain_initial_pose(mjModel *m, mjData *d, const ModelMap *map, int ground_init)
{
    const int base_qpos = m->jnt_qposadr[map->base_freejoint];
    const int left_wheel_body = m->jnt_bodyid[map->wheel[0].id];
    const int right_wheel_body = m->jnt_bodyid[map->wheel[1].id];
    mjtNum joint_qpos[4];

    mj_resetData(m, d);

    const mjtNum base_x = d->qpos[base_qpos + 0];
    const mjtNum base_y = d->qpos[base_qpos + 1];
    calc_initial_stand_joint_qpos(joint_qpos);

    d->qpos[base_qpos + 0] = base_x;
    d->qpos[base_qpos + 1] = base_y;
    d->qpos[base_qpos + 2] = 1.0;
    d->qpos[base_qpos + 3] = 1.0;
    d->qpos[base_qpos + 4] = 0.0;
    d->qpos[base_qpos + 5] = 0.0;
    d->qpos[base_qpos + 6] = 0.0;

    for (int i = 0; i < 4; ++i)
    {
        d->qpos[map->joint[i].qpos] = joint_qpos[i];
    }
    for (int i = 0; i < 2; ++i)
    {
        d->qpos[map->wheel[i].qpos] = 0.0;
    }

    memset(d->qvel, 0, sizeof(mjtNum) * m->nv);
    memset(d->ctrl, 0, sizeof(mjtNum) * m->nu);
    mj_forward(m, d);
    settle_closed_chain_pose(m, d, map, joint_qpos);

    const double left_clearance = (double)d->xpos[3 * left_wheel_body + 2] - WHEEL_RADIUS;
    const double right_clearance = (double)d->xpos[3 * right_wheel_body + 2] - WHEEL_RADIUS;
    const double ground_margin = 0.001;
    d->qpos[base_qpos + 2] += -fmin(left_clearance, right_clearance) + ground_margin;
    if (!ground_init)
    {
        d->qpos[base_qpos + 2] += 0.5;
    }

    memset(d->qvel, 0, sizeof(mjtNum) * m->nv);
    memset(d->ctrl, 0, sizeof(mjtNum) * m->nu);
    d->time = 0.0;
    mj_forward(m, d);
}

static void apply_model_initial_pose(mjModel *m, mjData *d, const ModelMap *map, const char *init_key_name)
{
    const int init_key = init_key_name ? find_optional_id(m, mjOBJ_KEY, init_key_name) : -1;
    if (init_key >= 0)
    {
        mj_resetDataKeyframe(m, d, init_key);
    }
    else if (find_optional_id(m, mjOBJ_BODY, "base") >= 0 &&
             init_key_name != 0 &&
             (strcmp(init_key_name, "pos_debug_ground") == 0 ||
              strcmp(init_key_name, "pos_debug_hang") == 0))
    {
        apply_closed_chain_initial_pose(m, d, map, strcmp(init_key_name, "pos_debug_ground") == 0);
    }
    else
    {
        mj_resetData(m, d);
    }

    memset(d->qvel, 0, sizeof(mjtNum) * m->nv);
    memset(d->ctrl, 0, sizeof(mjtNum) * m->nu);
    mj_forward(m, d);
}




static void read_state(const mjModel *m, const mjData *d, const ModelMap *map, SimControllerState *state)
{
    memset(state, 0, sizeof(*state));

    for (int i = 0; i < 4; ++i)
    {
        state->joint_pos[i] = (float)d->qpos[map->joint[i].qpos];
        state->joint_vel[i] = (float)d->qvel[map->joint[i].qvel];
    }

    for (int i = 0; i < 2; ++i)
    {
        state->wheel_vel[i] = (float)(g_wheel_ctrl_sign[i] * d->qvel[map->wheel[i].qvel]);
    }

    int base_qpos = m->jnt_qposadr[map->base_freejoint];
    int base_qvel = m->jnt_dofadr[map->base_freejoint];
    if (map->rotate_control_frame)
    {
        mjtNum rotated_quat[4];
        mjtNum raw_gyro[3] = {
            d->qvel[base_qvel + 3],
            d->qvel[base_qvel + 4],
            d->qvel[base_qvel + 5],
        };
        float rotated_gyro[3];
        float body_x = 0.0f;
        float body_y = 0.0f;
        float body_v = 0.0f;

        rotate_quat_into_controller_frame(&d->qpos[base_qpos + 3], rotated_quat);
        quat_to_euler(rotated_quat, &state->roll, &state->pitch, &state->yaw);

        rotate_vec3_into_controller_frame(raw_gyro, rotated_gyro);
        state->gyro[0] = rotated_gyro[0];
        state->gyro[1] = rotated_gyro[1];
        state->gyro[2] = rotated_gyro[2];

        rotate_xy_into_controller_frame(d->qpos[base_qpos + 0], d->qpos[base_qpos + 1], &body_x, &body_y);
        state->body_x = body_x;
        state->body_y = body_y;
        state->body_z = (float)d->qpos[base_qpos + 2];

        rotate_xy_into_controller_frame(d->qvel[base_qvel + 0], d->qvel[base_qvel + 1], &body_v, 0);
        state->body_v = body_v;
    }
    else
    {
        quat_to_euler(&d->qpos[base_qpos + 3], &state->roll, &state->pitch, &state->yaw);

        state->gyro[0] = (float)d->qvel[base_qvel + 3];
        state->gyro[1] = (float)d->qvel[base_qvel + 4];
        state->gyro[2] = (float)d->qvel[base_qvel + 5];
        state->body_x = (float)d->qpos[base_qpos + 0];
        state->body_y = (float)d->qpos[base_qpos + 1];
        state->body_z = (float)d->qpos[base_qpos + 2];
        state->body_v = (float)d->qvel[base_qvel + 0];
    }
}

static void update_drive_command(DriveCommand *command, const SimControllerState *state, double dt)
{
    if (command->hold_position_pending)
    {
        command->target_x = state->body_x;
        command->hold_position_pending = 0;
    }
    if (command->hold_yaw_pending)
    {
        command->yaw_hold = state->yaw;
        command->hold_yaw_pending = 0;
    }

    const float target_speed = command->mode == DRIVE_FORWARD ? command->forward_speed : 0.0f;
    const float max_speed_delta = (float)(dt * 0.8);
    float speed_delta = target_speed - command->current_speed;
    speed_delta = (float)clamp_double(speed_delta, -max_speed_delta, max_speed_delta);
    command->current_speed += speed_delta;

    if (command->mode == DRIVE_STAND)
    {
        if (command->position_hold_blend < 1.0f)
        {
            const float blend_delta = g_balance_pos_ramp_time > 1.0e-4f
                                          ? (float)(dt / g_balance_pos_ramp_time)
                                          : 1.0f;
            command->position_hold_blend = (float)clamp_double(command->position_hold_blend + blend_delta,
                                                               0.0,
                                                               1.0);
            command->target_x = state->body_x +
                                command->position_hold_blend * (command->target_x - state->body_x);
        }
    }
    else
    {
        command->position_hold_blend = 0.0f;
    }
}

static double measure_leg_length(const mjModel *m, const mjData *d, const ModelMap *map, int left_leg)
{
    const int hip_a_joint = left_leg ? map->joint[0].id : map->joint[2].id;
    const int hip_b_joint = left_leg ? map->joint[1].id : map->joint[3].id;
    const int wheel_joint = left_leg ? map->wheel[0].id : map->wheel[1].id;

    const int hip_a_body = m->jnt_bodyid[hip_a_joint];
    const int hip_b_body = m->jnt_bodyid[hip_b_joint];
    const int wheel_body = m->jnt_bodyid[wheel_joint];

    double hip_mid[3];
    for (int i = 0; i < 3; ++i)
    {
        hip_mid[i] = 0.5 * (d->xpos[3 * hip_a_body + i] + d->xpos[3 * hip_b_body + i]);
    }

    const double dx = d->xpos[3 * wheel_body + 0] - hip_mid[0];
    const double dy = d->xpos[3 * wheel_body + 1] - hip_mid[1];
    const double dz = d->xpos[3 * wheel_body + 2] - hip_mid[2];
    return sqrt(dx * dx + dy * dy + dz * dz);
}

static double distance3(const mjtNum *a, const mjtNum *b)
{
    const double dx = (double)a[0] - (double)b[0];
    const double dy = (double)a[1] - (double)b[1];
    const double dz = (double)a[2] - (double)b[2];
    return sqrt(dx * dx + dy * dy + dz * dz);
}

static int get_site_pos(const mjModel *m, const mjData *d, const char *name, const mjtNum **pos)
{
    int site = find_optional_id(m, mjOBJ_SITE, name);
    if (site < 0)
    {
        return 0;
    }

    *pos = &d->site_xpos[3 * site];
    return 1;
}

static double measure_mid_to_site(const mjModel *m, const mjData *d, const ModelMap *map, int left_leg, const char *site_name)
{
    const int hip_a_joint = left_leg ? map->joint[0].id : map->joint[2].id;
    const int hip_b_joint = left_leg ? map->joint[1].id : map->joint[3].id;
    const int hip_a_body = m->jnt_bodyid[hip_a_joint];
    const int hip_b_body = m->jnt_bodyid[hip_b_joint];
    const mjtNum *site_pos = 0;
    mjtNum hip_mid[3];

    if (!get_site_pos(m, d, site_name, &site_pos))
    {
        return NAN;
    }

    for (int i = 0; i < 3; ++i)
    {
        hip_mid[i] = 0.5 * (d->xpos[3 * hip_a_body + i] + d->xpos[3 * hip_b_body + i]);
    }

    return distance3(hip_mid, site_pos);
}

static double measure_virtual_leg_length(const mjModel *m, const mjData *d, const ModelMap *map, int left_leg)
{
    const int hip_a_joint = left_leg ? map->joint[0].id : map->joint[2].id;
    const int hip_b_joint = left_leg ? map->joint[1].id : map->joint[3].id;
    const int hip_a_body = m->jnt_bodyid[hip_a_joint];
    const int hip_b_body = m->jnt_bodyid[hip_b_joint];
    const char *site_a_name = left_leg ? "OP-N" : "GH-F";
    const char *site_b_name = left_leg ? "KN-N" : "CF-F";
    const mjtNum *site_a = 0;
    const mjtNum *site_b = 0;
    mjtNum hip_mid[3];
    mjtNum foot_mid[3];

    if (!get_site_pos(m, d, site_a_name, &site_a) || !get_site_pos(m, d, site_b_name, &site_b))
    {
        return NAN;
    }

    for (int i = 0; i < 3; ++i)
    {
        hip_mid[i] = 0.5 * (d->xpos[3 * hip_a_body + i] + d->xpos[3 * hip_b_body + i]);
        foot_mid[i] = 0.5 * (site_a[i] + site_b[i]);
    }

    return distance3(hip_mid, foot_mid);
}

static void print_joint_qpos(const mjModel *m, const mjData *d, const char *name)
{
    const int joint = find_optional_id(m, mjOBJ_JOINT, name);
    if (joint >= 0)
    {
        printf("  %-14s qpos=% .6f\n", name, d->qpos[m->jnt_qposadr[joint]]);
    }
}

static void print_site_error(const mjModel *m, const mjData *d, const char *a, const char *b)
{
    const mjtNum *pa = 0;
    const mjtNum *pb = 0;
    if (get_site_pos(m, d, a, &pa) && get_site_pos(m, d, b, &pb))
    {
        printf("  %-6s <-> %-6s err=% .6f\n", a, b, distance3(pa, pb));
    }
}

static void print_geometry_debug(const mjModel *m, const mjData *d, const ModelMap *map, const char *tag)
{
    printf("geometry[%s]\n", tag);
    printf("  length left:  wheel=% .6f OP-N=% .6f KN-N=% .6f\n",
           measure_leg_length(m, d, map, 1),
           measure_mid_to_site(m, d, map, 1, "OP-N"),
           measure_mid_to_site(m, d, map, 1, "KN-N"));
    printf("  length right: wheel=% .6f GH-F=% .6f CF-F=% .6f\n",
           measure_leg_length(m, d, map, 0),
           measure_mid_to_site(m, d, map, 0, "GH-F"),
           measure_mid_to_site(m, d, map, 0, "CF-F"));
    printf("  equality residuals:\n");
    print_site_error(m, d, "IO-L", "MK-L");
    print_site_error(m, d, "OP-N", "KN-N");
    print_site_error(m, d, "AG-D", "EC-D");
    print_site_error(m, d, "GH-F", "CF-F");
    printf("  left joints:\n");
    print_joint_qpos(m, d, "jIO");
    print_joint_qpos(m, d, "jOP");
    print_joint_qpos(m, d, "jIJ");
    print_joint_qpos(m, d, "jJM");
    print_joint_qpos(m, d, "jMK");
    print_joint_qpos(m, d, "jKN");
    printf("  right joints:\n");
    print_joint_qpos(m, d, "jAG");
    print_joint_qpos(m, d, "jGH");
    print_joint_qpos(m, d, "jAB");
    print_joint_qpos(m, d, "jBE");
    print_joint_qpos(m, d, "jEC");
    print_joint_qpos(m, d, "jCF");
}

static void print_joint_axis_heights(const mjModel *m, const mjData *d)
{
    printf("axis_z=");
    for (int joint = 0; joint < m->njnt; ++joint)
    {
        if (m->jnt_type[joint] == mjJNT_FREE)
        {
            continue;
        }

        const char *name = mj_id2name(m, mjOBJ_JOINT, joint);
        if (name == 0)
        {
            name = "(unnamed)";
        }

        printf("%s:% .3f ", name, d->xanchor[3 * joint + 2]);
    }
    printf("\n");
}





static void write_output(mjData *d, const ModelMap *map, const SimControllerOutput *output)
{
    d->ctrl[map->actuator[0]] = g_joint_ctrl_sign[0] * output->joint_torque[0];
    d->ctrl[map->actuator[1]] = g_joint_ctrl_sign[1] * output->joint_torque[1];
    d->ctrl[map->actuator[2]] = g_joint_ctrl_sign[2] * output->joint_torque[2];
    d->ctrl[map->actuator[3]] = g_joint_ctrl_sign[3] * output->joint_torque[3];
    d->ctrl[map->actuator[4]] = g_wheel_ctrl_sign[0] * output->wheel_torque[0];
    d->ctrl[map->actuator[5]] = g_wheel_ctrl_sign[1] * output->wheel_torque[1];
}

static mjtNum clamp_actuator_ctrl(const mjModel *m, int actuator, mjtNum value)
{
    if (m->actuator_ctrllimited[actuator])
    {
        const mjtNum min_value = m->actuator_ctrlrange[2 * actuator + 0];
        const mjtNum max_value = m->actuator_ctrlrange[2 * actuator + 1];
        if (value < min_value)
        {
            return min_value;
        }
        if (value > max_value)
        {
            return max_value;
        }
    }
    return value;
}

static void clamp_output_to_model(const mjModel *m, const ModelMap *map, SimControllerOutput *output)
{
    output->joint_torque[0] = clamp_actuator_ctrl(m, map->actuator[0], output->joint_torque[0]);
    output->joint_torque[1] = clamp_actuator_ctrl(m, map->actuator[1], output->joint_torque[1]);
    output->joint_torque[2] = clamp_actuator_ctrl(m, map->actuator[2], output->joint_torque[2]);
    output->joint_torque[3] = clamp_actuator_ctrl(m, map->actuator[3], output->joint_torque[3]);
    output->wheel_torque[0] = clamp_actuator_ctrl(m, map->actuator[4], output->wheel_torque[0]);
    output->wheel_torque[1] = clamp_actuator_ctrl(m, map->actuator[5], output->wheel_torque[1]);
}

static void apply_wheel_balance_override(const SimControllerState *state, SimControllerOutput *output)
{
    const float pitch_term = g_balance_pitch_kp * (state->pitch - g_balance_pitch_target) +
                             g_balance_pitch_kd * state->gyro[1];
    const float pos_term = g_drive_command.mode == DRIVE_FORWARD
                               ? 0.0f
                               : g_balance_pos_kp * g_drive_command.position_hold_blend *
                                     (state->body_x - g_drive_command.target_x);
    const float vel_term = g_balance_vel_kd * (state->body_v - g_drive_command.current_speed);
    const float drive_term = g_drive_command.mode == DRIVE_FORWARD ? (g_balance_drive_kff * g_drive_command.current_speed) : 0.0f;
    const float yaw_term = g_drive_command.yaw_lock
                               ? g_balance_yaw_kp * (state->yaw - g_drive_command.yaw_hold) +
                                     g_balance_yaw_kd * state->gyro[2]
                               : 0.0f;
    const float common = (float)clamp_double(pitch_term + pos_term + vel_term + drive_term,
                                             -g_balance_wheel_limit,
                                             g_balance_wheel_limit);

    output->wheel_torque[0] = common - yaw_term;
    output->wheel_torque[1] = common + yaw_term;
}




static void mouse_button_callback(GLFWwindow *window, int button, int act, int mods)
{
    (void)button;
    (void)act;
    (void)mods;

    g_button_left = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    g_button_middle = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
    g_button_right = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
    glfwGetCursorPos(window, &g_last_x, &g_last_y);
}

static void mouse_move_callback(GLFWwindow *window, double xpos, double ypos)
{
    if (!g_button_left && !g_button_middle && !g_button_right)
    {
        return;
    }

    double dx = xpos - g_last_x;
    double dy = ypos - g_last_y;
    g_last_x = xpos;
    g_last_y = ypos;

    int width = 0;
    int height = 0;
    glfwGetWindowSize(window, &width, &height);

    int shift = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;

    mjtMouse action;
    if (g_button_right)
    {
        action = shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    }
    else if (g_button_left)
    {
        action = shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    }
    else
    {
        action = mjMOUSE_ZOOM;
    }

    mjv_moveCamera(g_model, action, dx / (double)height, dy / (double)height, &g_scene, &g_camera);
}

static void scroll_callback(GLFWwindow *window, double xoffset, double yoffset)
{
    (void)window;
    (void)xoffset;
    mjv_moveCamera(g_model, mjMOUSE_ZOOM, 0.0, -0.05 * yoffset, &g_scene, &g_camera);
}

static void keyboard_callback(GLFWwindow *window, int key, int scancode, int act, int mods)
{
    (void)scancode;
    (void)mods;

    if (act == GLFW_PRESS && key == GLFW_KEY_ESCAPE)
    {
        glfwSetWindowShouldClose(window, GLFW_TRUE);
    }
    else if (act == GLFW_PRESS && key == GLFW_KEY_SPACE)
    {
        g_paused = !g_paused;
        printf("Simulation %s at t=%6.3f\n", g_paused ? "paused" : "resumed", g_data ? g_data->time : 0.0);
        fflush(stdout);
    }
}

static void step_controller(const mjModel *m,
                            mjData *d,
                            const ModelMap *map,
                            int print_line,
                            int zero_control,
                            int zero_wheels,
                            int invert_right_joints,
                            int start_mode,
                            double standup_time,
                            int *switched_to_safe)
{
    SimControllerState state;
    SimControllerOutput output;

    read_state(m, d, map, &state);
    if (zero_control)
    {
        memset(&output, 0, sizeof(output));
    }
    else
    {
        if (start_mode == CHASSIS_STAND_UP && standup_time >= 0.0 && d->time >= standup_time)
        {
            SimController_SetMode(CHASSIS_SAFE);
            if (switched_to_safe != 0 && *switched_to_safe == 0)
            {
                g_drive_command.hold_position_pending = 1;
                g_drive_command.position_hold_blend = 0.0f;
                printf("Switch STAND_UP -> SAFE at t=%6.3f\n", d->time);
                fflush(stdout);
                *switched_to_safe = 1;
            }
        }
        SimController_SetState(&state);
        update_drive_command(&g_drive_command, &state, m->opt.timestep);
        SimController_SetCommand(g_drive_command.current_speed,
                                 g_drive_command.target_x,
                                 g_drive_command.leg_set,
                                 g_drive_command.roll_set,
                                 g_drive_command.yaw_lock ? g_drive_command.yaw_hold : 0.0f);
        SimController_Step((float)m->opt.timestep);
        SimController_GetOutput(&output);
        if (g_use_wheel_balance_override &&
            (chassis_move.mode == CHASSIS_SAFE || chassis_move.mode == CHASSIS_STAND_UP))
        {
            apply_wheel_balance_override(&state, &output);
        }
        if (zero_wheels)
        {
            output.wheel_torque[0] = 0.0f;
            output.wheel_torque[1] = 0.0f;
        }
        if (invert_right_joints)
        {
            output.joint_torque[2] = -output.joint_torque[2];
            output.joint_torque[3] = -output.joint_torque[3];
        }
        clamp_output_to_model(m, map, &output);
    }
    write_output(d, map, &output);

    mj_step(m, d);

    if (print_line)
    {
        const double virtual_left = measure_virtual_leg_length(m, d, map, 1);
        const double virtual_right = measure_virtual_leg_length(m, d, map, 0);
        const double wheel_left = measure_leg_length(m, d, map, 1);
        const double wheel_right = measure_leg_length(m, d, map, 0);

        printf("t=%6.3f drive=%s keys=[F:%d B:%d U:%d D:%d] vx_ref=% .3f x_ref=% .3f pos_hold=% .2f leg_ref=% .3f pos=[% .3f % .3f % .3f] rpy=[% .3f % .3f % .3f] "
               "vmcL0=[% .3f % .3f] siteL=[% .3f % .3f] wheelL=[% .3f % .3f] "
               "phi1=[% .3f % .3f] phi4=[% .3f % .3f] "
               "u=[% .2f % .2f % .2f % .2f | % .2f % .2f]\n",
               d->time,
               drive_command_name(&g_drive_command),
               g_key_forward,
               g_key_backward,
               g_key_leg_up,
               g_key_leg_down,
               g_drive_command.current_speed,
               g_drive_command.target_x,
               g_drive_command.position_hold_blend,
               g_drive_command.leg_set,
               state.body_x,
               state.body_y,
               state.body_z,
               state.roll,
               state.pitch,
               state.yaw,
               left.L0,
               right.L0,
               virtual_left,
               virtual_right,
               wheel_left,
               wheel_right,
               left.phi1,
               right.phi1,
               left.phi4,
               right.phi4,
               output.joint_torque[0],
               output.joint_torque[1],
               output.joint_torque[2],
               output.joint_torque[3],
               output.wheel_torque[0],
               output.wheel_torque[1]);
        print_joint_axis_heights(m, d);
    }
}

static int run_headless(const mjModel *m,
                        mjData *d,
                        const ModelMap *map,
                        double sim_time,
                        int zero_control,
                        int zero_wheels,
                        int invert_right_joints,
                        int freeze_init,
                        int start_mode,
                        double standup_time)
{
    if (freeze_init)
    {
        printf("Initial state frozen at t=%6.3f\n", d->time);
        return 0;
    }

    int switched_to_safe = 0;
    int steps = (int)(sim_time / m->opt.timestep);
    for (int i = 0; i < steps; ++i)
    {
        step_controller(m, d, map, i % 500 == 0, zero_control, zero_wheels, invert_right_joints, start_mode, standup_time, &switched_to_safe);
    }

    return 0;
}

static int run_viewer(mjModel *m,
                      mjData *d,
                      const ModelMap *map,
                      double sim_time,
                      int zero_control,
                      int zero_wheels,
                      int invert_right_joints,
                      int freeze_init,
                      int start_mode,
                      double standup_time)
{
    if (!glfwInit())
    {
        fprintf(stderr, "Failed to initialize GLFW. If you are in WSL, check that WSLg/X server is available.\n");
        return 1;
    }

    GLFWwindow *window = glfwCreateWindow(1200, 900, "rm mujoco bridge", 0, 0);
    if (!window)
    {
        glfwTerminate();
        fprintf(stderr, "Failed to create GLFW window.\n");
        return 1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    g_model = m;
    g_data = d;
    mjv_defaultCamera(&g_camera);
    mjv_defaultOption(&g_option);
    mjv_defaultScene(&g_scene);
    mjr_defaultContext(&g_context);

    g_camera.distance = 2.0;
    g_camera.azimuth = 135.0;
    g_camera.elevation = -20.0;
    g_camera.lookat[0] = 0.0;
    g_camera.lookat[1] = 0.0;
    g_camera.lookat[2] = 0.25;

    mjv_makeScene(m, &g_scene, 2000);
    mjr_makeContext(m, &g_context, mjFONTSCALE_150);

    glfwSetKeyCallback(window, keyboard_callback);
    glfwSetCursorPosCallback(window, mouse_move_callback);
    glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwSetScrollCallback(window, scroll_callback);

    double last_wall = glfwGetTime();
    int print_tick = 0;
    int switched_to_safe = 0;

    while (!glfwWindowShouldClose(window) && d->time < sim_time)
    {
        double now = glfwGetTime();
        double elapsed = now - last_wall;
        last_wall = now;
        sync_keyboard_drive_command(window, elapsed);

        if (!g_paused && !freeze_init)
        {
            double target_time = d->time + elapsed;
            while (d->time < target_time && !glfwWindowShouldClose(window))
            {
                step_controller(m, d, map, print_tick % 500 == 0, zero_control, zero_wheels, invert_right_joints, start_mode, standup_time, &switched_to_safe);
                ++print_tick;
            }
        }

        mjv_updateScene(m, d, &g_option, 0, &g_camera, mjCAT_ALL, &g_scene);

        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
        mjr_render(viewport, &g_scene, &g_context);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    mjr_freeContext(&g_context);
    mjv_freeScene(&g_scene);
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}

int main(int argc, char **argv)
{
    const char *model_path = "sim/models/wheel_leg_urdf4_self_mesh_all.xml";
    int headless = 0;
    int zero_control = 0;
    int zero_wheels = 0;
    int invert_right_joints = 0;
    int freeze_init = 0;
    int debug_geometry = 0;
    int start_mode = CHASSIS_SAFE;
    double standup_time = 0.2;
    double sim_time = 100.0;
    const char *init_key_name = "pos_debug_ground";
    for (int i = 1; i < argc; ++i)
    {
        if (strcmp(argv[i], "--headless") == 0)
        {
            headless = 1;
        }
        else if (strcmp(argv[i], "--zero-control") == 0)
        {
            zero_control = 1;
        }
        else if (strcmp(argv[i], "--zero-wheels") == 0)
        {
            zero_wheels = 1;
        }
        else if (strcmp(argv[i], "--invert-right-joints") == 0)
        {
            invert_right_joints = 1;
        }
        else if (strcmp(argv[i], "--freeze-init") == 0)
        {
            freeze_init = 1;
        }
        else if (strcmp(argv[i], "--debug-geometry") == 0)
        {
            debug_geometry = 1;
        }
        else if (strcmp(argv[i], "--time") == 0 && i + 1 < argc)
        {
            sim_time = atof(argv[++i]);
        }
        else if (strcmp(argv[i], "--standup-time") == 0 && i + 1 < argc)
        {
            standup_time = atof(argv[++i]);
        }
        else if (strcmp(argv[i], "--forward-speed") == 0 && i + 1 < argc)
        {
            g_drive_command.forward_speed = (float)atof(argv[++i]);
        }
        else if (strcmp(argv[i], "--drive") == 0 && i + 1 < argc)
        {
            const char *drive = argv[++i];
            if (strcmp(drive, "stand") == 0)
            {
                queue_drive_mode(&g_drive_command, DRIVE_STAND);
            }
            else if (strcmp(drive, "forward") == 0)
            {
                queue_drive_mode(&g_drive_command, DRIVE_FORWARD);
            }
            else
            {
                fprintf(stderr, "--drive expects stand or forward.\n");
                return 2;
            }
        }
        else if (strcmp(argv[i], "--free-yaw") == 0)
        {
            g_drive_command.yaw_lock = 0;
        }
        else if (strcmp(argv[i], "--no-wheel-override") == 0)
        {
            g_use_wheel_balance_override = 0;
        }
        else if ((strcmp(argv[i], "--override-pitch-kp") == 0 ||
                  strcmp(argv[i], "--stand-pitch-kp") == 0) &&
                 i + 1 < argc)
        {
            g_balance_pitch_kp = (float)atof(argv[++i]);
        }
        else if ((strcmp(argv[i], "--override-pitch-target") == 0 ||
                  strcmp(argv[i], "--stand-pitch-target") == 0) &&
                 i + 1 < argc)
        {
            g_balance_pitch_target = (float)atof(argv[++i]);
        }
        else if ((strcmp(argv[i], "--override-pitch-kd") == 0 ||
                  strcmp(argv[i], "--stand-pitch-kd") == 0) &&
                 i + 1 < argc)
        {
            g_balance_pitch_kd = (float)atof(argv[++i]);
        }
        else if ((strcmp(argv[i], "--override-pos-kp") == 0 ||
                  strcmp(argv[i], "--stand-pos-kp") == 0) &&
                 i + 1 < argc)
        {
            g_balance_pos_kp = (float)atof(argv[++i]);
        }
        else if ((strcmp(argv[i], "--override-vel-kd") == 0 ||
                  strcmp(argv[i], "--stand-vel-kd") == 0) &&
                 i + 1 < argc)
        {
            g_balance_vel_kd = (float)atof(argv[++i]);
        }
        else if ((strcmp(argv[i], "--override-pos-ramp-time") == 0 ||
                  strcmp(argv[i], "--stand-pos-ramp-time") == 0) &&
                 i + 1 < argc)
        {
            g_balance_pos_ramp_time = (float)atof(argv[++i]);
        }
        else if ((strcmp(argv[i], "--override-drive-kff") == 0 ||
                  strcmp(argv[i], "--stand-drive-kff") == 0) &&
                 i + 1 < argc)
        {
            g_balance_drive_kff = (float)atof(argv[++i]);
        }
        else if ((strcmp(argv[i], "--override-yaw-kp") == 0 ||
                  strcmp(argv[i], "--stand-yaw-kp") == 0) &&
                 i + 1 < argc)
        {
            g_balance_yaw_kp = (float)atof(argv[++i]);
        }
        else if ((strcmp(argv[i], "--override-yaw-kd") == 0 ||
                  strcmp(argv[i], "--stand-yaw-kd") == 0) &&
                 i + 1 < argc)
        {
            g_balance_yaw_kd = (float)atof(argv[++i]);
        }
        else if ((strcmp(argv[i], "--override-wheel-limit") == 0 ||
                  strcmp(argv[i], "--stand-wheel-limit") == 0) &&
                 i + 1 < argc)
        {
            g_balance_wheel_limit = (float)atof(argv[++i]);
        }
        else if (strcmp(argv[i], "--mode") == 0 && i + 1 < argc)
        {
            const char *mode = argv[++i];
            if (strcmp(mode, "stand") == 0)
            {
                start_mode = CHASSIS_STAND_UP;
            }
            else if (strcmp(mode, "safe") == 0)
            {
                start_mode = CHASSIS_SAFE;
            }
            else if (strcmp(mode, "off") == 0)
            {
                start_mode = CHASSIS_OFF;
            }
            else
            {
                fprintf(stderr, "--mode expects stand, safe, or off.\n");
                return 2;
            }
        }
        else if (strcmp(argv[i], "--joint-signs") == 0 && i + 1 < argc)
        {
            const char *signs = argv[++i];
            if (strlen(signs) != 4)
            {
                fprintf(stderr, "--joint-signs expects four characters, for example +-+-.\n");
                return 2;
            }
            for (int j = 0; j < 4; ++j)
            {
                if (signs[j] == '+')
                {
                    g_joint_ctrl_sign[j] = 1.0;
                }
                else if (signs[j] == '-')
                {
                    g_joint_ctrl_sign[j] = -1.0;
                }
                else
                {
                    fprintf(stderr, "--joint-signs only accepts + or - characters.\n");
                    return 2;
                }
            }
        }
       
        
        else if (strcmp(argv[i], "--init-key") == 0 && i + 1 < argc)
        {
            init_key_name = argv[++i];
        }
        else if (strcmp(argv[i], "--hang-init") == 0)
        {
            init_key_name = "pos_debug_hang";
        }
        else if (strcmp(argv[i], "--ground-init") == 0)
        {
            init_key_name = "pos_debug_ground";
        }
        else
        {
            model_path = argv[i];
        }
    }

    char error[1024] = {0};
    model_path = resolve_default_model_path(model_path);

    mjModel *m = mj_loadXML(model_path, 0, error, sizeof(error));
    if (m == 0)
    {
        fprintf(stderr, "Failed to load MuJoCo model: %s\n%s\n", model_path, error);
        return 1;
    }

    mjData *d = mj_makeData(m);
    if (d == 0)
    {
        fprintf(stderr, "Failed to allocate MuJoCo data.\n");
        mj_deleteModel(m);
        return 1;
    }

    ModelMap map;
    build_model_map(m, &map);
    for (int i = 0; i < 2; ++i)
    {
        const mjtNum axis_y = m->jnt_axis[3 * map.wheel[i].id + 1];
        g_wheel_ctrl_sign[i] = axis_y > 0.0 ? -1.0 : 1.0;
    }
    apply_model_initial_pose(m, d, &map, init_key_name);
    if (debug_geometry)
    {
        print_geometry_debug(m, d, &map, "init");
    }
    SimController_Init();
    SimController_SetMode(start_mode);
    g_drive_command.current_speed = 0.0f;
    g_drive_command.leg_set = INIT_LEG_LENGTH;
    g_drive_command.roll_set = INIT_ROLL;
    g_drive_command.position_hold_blend = start_mode == CHASSIS_STAND_UP ? 0.0f : 1.0f;
    g_drive_command.hold_position_pending = 1;
    g_drive_command.hold_yaw_pending = 1;
    printf("Drive mode: %s | forward_speed=%.3f | yaw_lock=%d | wheel_sign=[%.0f %.0f] | wheel_override=%d\n",
           drive_command_name(&g_drive_command),
           g_drive_command.forward_speed,
           g_drive_command.yaw_lock,
           g_wheel_ctrl_sign[0],
           g_wheel_ctrl_sign[1],
           g_use_wheel_balance_override);
    printf("Wheel override gains: pitch_target=%.3f pitch_kp=%.3f pitch_kd=%.3f pos_kp=%.3f vel_kd=%.3f pos_ramp=%.3f drive_kff=%.3f yaw_kp=%.3f yaw_kd=%.3f limit=%.3f\n",
           g_balance_pitch_target,
           g_balance_pitch_kp,
           g_balance_pitch_kd,
           g_balance_pos_kp,
           g_balance_vel_kd,
           g_balance_pos_ramp_time,
           g_balance_drive_kff,
           g_balance_yaw_kp,
           g_balance_yaw_kd,
           g_balance_wheel_limit);
    if (start_mode == CHASSIS_STAND_UP && standup_time >= 0.0)
    {
        printf("Start in STAND_UP, auto switch to SAFE after %.3f s.\n", standup_time);
    }

    int result = headless ? run_headless(m, d, &map, sim_time, zero_control, zero_wheels, invert_right_joints, freeze_init, start_mode, standup_time) : run_viewer(m, d, &map, sim_time, zero_control, zero_wheels, invert_right_joints, freeze_init, start_mode, standup_time);

    mj_deleteData(d);
    mj_deleteModel(m);
    return result;
}
