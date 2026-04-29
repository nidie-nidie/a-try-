#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include "pos_debug.h"

static mjModel *g_model = 0;
static mjData *g_data = 0;
static mjvCamera g_camera;
static mjvOption g_option;
static mjvScene g_scene;
static mjrContext g_context;
static int g_button_left = 0;
static int g_button_middle = 0;
static int g_button_right = 0;
static double g_last_x = 0.0;
static double g_last_y = 0.0;

static double clamp(double value, double min_value, double max_value)
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

static void clear_input_command(PosDebugState *state)
{
    snprintf(state->input_command, sizeof(state->input_command), "none");
}

static void append_input_command(PosDebugState *state, const char *command)
{
    if (strcmp(state->input_command, "none") == 0)
    {
        state->input_command[0] = '\0';
    }

    if (state->input_command[0] != '\0')
    {
        strncat(state->input_command, " + ", sizeof(state->input_command) - strlen(state->input_command) - 1);
    }
    strncat(state->input_command, command, sizeof(state->input_command) - strlen(state->input_command) - 1);
}

static double wrap_pi(double value)
{
    while (value > kPi)
    {
        value -= 2.0 * kPi;
    }
    while (value < -kPi)
    {
        value += 2.0 * kPi;
    }
    return value;
}

static double theta_transform(double angle, double dangle, double direction)
{
    return wrap_pi((angle + dangle) * direction);
}

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

static JointRef find_joint(const mjModel *m, const char *name)
{
    int id = find_required_id(m, mjOBJ_JOINT, name);
    JointRef ref = {m->jnt_qposadr[id], m->jnt_dofadr[id]};
    return ref;
}

static void set_joint_ref(const mjModel *m, int index, const char *name, ModelMap *map)
{
    int id = find_required_id(m, mjOBJ_JOINT, name);
    map->joint_id[index] = id;
    map->joint[index].qpos = m->jnt_qposadr[id];
    map->joint[index].qvel = m->jnt_dofadr[id];
}

static void build_model_map(const mjModel *m, ModelMap *map)
{
    memset(map, 0, sizeof(*map));

    if (find_optional_id(m, mjOBJ_BODY, "base") >= 0)
    {
        map->base_body = find_required_id(m, mjOBJ_BODY, "base");
        map->base_freejoint = find_required_id(m, mjOBJ_JOINT, "base_free");

        set_joint_ref(m, 0, "jIO", map);
        set_joint_ref(m, 1, "jIJ", map);
        set_joint_ref(m, 2, "jAB", map);
        set_joint_ref(m, 3, "jAG", map);
        map->wheel[0] = find_joint(m, "jwheel_left");
        map->wheel[1] = find_joint(m, "jwheel_right");

        map->actuator[0] = find_required_id(m, mjOBJ_ACTUATOR, "Left_rear_joint_act");
        map->actuator[1] = find_required_id(m, mjOBJ_ACTUATOR, "Left_front_joint_act");
        map->actuator[2] = find_required_id(m, mjOBJ_ACTUATOR, "Right_front_joint_act");
        map->actuator[3] = find_required_id(m, mjOBJ_ACTUATOR, "Right_rear_joint_act");
        map->actuator[4] = find_required_id(m, mjOBJ_ACTUATOR, "Left_Wheel_act");
        map->actuator[5] = find_required_id(m, mjOBJ_ACTUATOR, "Right_Wheel_act");
        return;
    }

    if (find_optional_id(m, mjOBJ_BODY, "base_link") >= 0)
    {
        map->base_body = find_required_id(m, mjOBJ_BODY, "base_link");
        map->base_freejoint = find_required_id(m, mjOBJ_JOINT, "base_freejoint");

        set_joint_ref(m, 0, "left_leg", map);
        set_joint_ref(m, 1, "left_small_leg", map);
        set_joint_ref(m, 2, "right_leg", map);
        set_joint_ref(m, 3, "right_small_leg", map);
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

    fprintf(stderr, "Unsupported model: expected body 'base' or 'base_link'.\n");
    exit(2);
}

static void widen_active_joint_ranges(mjModel *m, const ModelMap *map)
{
    for (int i = 0; i < POS_DEBUG_LEG_JOINT_COUNT; ++i)
    {
        int id = map->joint_id[i];
        m->jnt_limited[id] = 1;
        m->jnt_range[2 * id] = kActiveJointRangeMin;
        m->jnt_range[2 * id + 1] = kActiveJointRangeMax;
    }
}

static void print_model_map(const mjModel *m, const ModelMap *map)
{
    const char *slot_name[6] = {
        "left rear",
        "left front",
        "right front",
        "right rear",
        "left wheel",
        "right wheel",
    };

    const JointRef joint_ref[6] = {
        map->joint[JOINT_LEFT_REAR],
        map->joint[JOINT_LEFT_FRONT],
        map->joint[JOINT_RIGHT_FRONT],
        map->joint[JOINT_RIGHT_REAR],
        map->wheel[0],
        map->wheel[1],
    };

    printf("\nModel mapping check:\n");
    for (int i = 0; i < POS_DEBUG_ACTUATOR_COUNT; ++i)
    {
        const int actuator_id = map->actuator[i];
        const int joint_id = i < 4 ? map->joint_id[i] : m->actuator_trnid[2 * actuator_id];
        const char *joint_name = mj_id2name(m, mjOBJ_JOINT, joint_id);
        const char *body_name = mj_id2name(m, mjOBJ_BODY, m->jnt_bodyid[joint_id]);
        const char *actuator_name = mj_id2name(m, mjOBJ_ACTUATOR, actuator_id);
        const int actuator_joint_id = m->actuator_trnid[2 * actuator_id];
        const char *actuator_joint_name = mj_id2name(m, mjOBJ_JOINT, actuator_joint_id);
        const double *axis = &m->jnt_axis[3 * joint_id];

        printf("  %-11s joint=%-14s body=%-12s axis=[% .0f % .0f % .0f] qpos=%2d qvel=%2d | actuator=%-22s -> joint=%s\n",
               slot_name[i],
               joint_name ? joint_name : "(null)",
               body_name ? body_name : "(null)",
               axis[0],
               axis[1],
               axis[2],
               joint_ref[i].qpos,
               joint_ref[i].qvel,
               actuator_name ? actuator_name : "(null)",
               actuator_joint_name ? actuator_joint_name : "(null)");
    }
    printf("\n");
}

static void save_hang_pose(const mjModel *m, mjData *d, const ModelMap *map, PosDebugState *state)
{
    int base_qpos = m->jnt_qposadr[map->base_freejoint];
    state->saved_base_qpos[0] = 0.0;
    state->saved_base_qpos[1] = 0.0;
    state->saved_base_qpos[2] = state->hang_z;
    state->saved_base_qpos[3] = 1.0;
    state->saved_base_qpos[4] = 0.0;
    state->saved_base_qpos[5] = 0.0;
    state->saved_base_qpos[6] = 0.0;

    for (int i = 0; i < POS_DEBUG_BASE_QPOS_SIZE; ++i)
    {
        d->qpos[base_qpos + i] = state->saved_base_qpos[i];
    }
    mj_forward(m, d);
}

static void hold_base_pose(const mjModel *m, mjData *d, const ModelMap *map, const PosDebugState *state)
{
    int base_qpos = m->jnt_qposadr[map->base_freejoint];
    int base_qvel = m->jnt_dofadr[map->base_freejoint];

    for (int i = 0; i < POS_DEBUG_BASE_QPOS_SIZE; ++i)
    {
        d->qpos[base_qpos + i] = state->saved_base_qpos[i];
    }
    for (int i = 0; i < POS_DEBUG_BASE_QVEL_SIZE; ++i)
    {
        d->qvel[base_qvel + i] = 0.0;
    }
}

static int calc_phi1_phi4(double phi0, double leg_length, double phi1_phi4[2])
{
    const double cos_beta1 = (kLegL1 * kLegL1 + leg_length * leg_length - kLegL2 * kLegL2) / (2.0 * kLegL1 * leg_length);
    const double cos_beta2 = (kLegL4 * kLegL4 + leg_length * leg_length - kLegL3 * kLegL3) / (2.0 * kLegL4 * leg_length);

    if (cos_beta1 < -1.0 || cos_beta1 > 1.0 || cos_beta2 < -1.0 || cos_beta2 > 1.0)
    {
        return 0;
    }

    phi1_phi4[0] = phi0 + acos(cos_beta1);
    phi1_phi4[1] = phi0 - acos(cos_beta2);
    return 1;
}

static void solve_left_leg_targets(PosDebugState *state)
{
    double left_phi[2] = {0.0, 0.0};

    if (!calc_phi1_phi4(kPi / 2.0 + state->left_l0_pitch, state->left_leg_length, left_phi))
    {
        return;
    }

    state->target[JOINT_LEFT_REAR] = -theta_transform(left_phi[1], -kJ0Offset, 1.0);
    state->target[JOINT_LEFT_FRONT] = -theta_transform(left_phi[0], -kJ1Offset, 1.0);
}

static void solve_right_leg_targets(PosDebugState *state)
{
    double right_phi[2] = {0.0, 0.0};

    if (!calc_phi1_phi4(kPi / 2.0 + state->right_l0_pitch, state->right_leg_length, right_phi))
    {
        return;
    }

    state->target[JOINT_RIGHT_FRONT] = -theta_transform(right_phi[0], -kJ2Offset, 1.0);
    state->target[JOINT_RIGHT_REAR] = -theta_transform(right_phi[1], -kJ3Offset, 1.0);
}

static void solve_targets(PosDebugState *state)
{
    state->visual_left_leg_length = clamp(state->visual_left_leg_length, state->min_leg_length, state->max_leg_length);
    state->visual_right_leg_length = clamp(state->visual_right_leg_length, state->min_leg_length, state->max_leg_length);
    state->left_l0_pitch = clamp(state->left_l0_pitch, state->min_l0_pitch, state->max_l0_pitch);
    state->right_l0_pitch = clamp(state->right_l0_pitch, state->min_l0_pitch, state->max_l0_pitch);
    state->left_leg_length = state->min_leg_length + state->max_leg_length - state->visual_left_leg_length;
    state->right_leg_length = state->min_leg_length + state->max_leg_length - state->visual_right_leg_length;
    state->left_leg_length = clamp(state->left_leg_length, state->min_leg_length, state->max_leg_length);
    state->right_leg_length = clamp(state->right_leg_length, state->min_leg_length, state->max_leg_length);
    solve_left_leg_targets(state);
    solve_right_leg_targets(state);
}

static void apply_position_control(const mjModel *m, mjData *d, const ModelMap *map, PosDebugState *state)
{
    solve_targets(state);

    for (int i = 0; i < m->nu; ++i)
    {
        d->ctrl[i] = 0.0;
    }

    for (int i = 0; i < POS_DEBUG_LEG_JOINT_COUNT; ++i)
    {
        const int actuator = map->actuator[i];
        const double q = d->qpos[map->joint[i].qpos];
        const double qd = d->qvel[map->joint[i].qvel];
        double tau = state->kp * (state->target[i] - q) - state->kd * qd;
        tau = clamp(tau, -state->max_torque, state->max_torque);

        if (m->actuator_ctrllimited[actuator])
        {
            tau = clamp(tau, m->actuator_ctrlrange[2 * actuator], m->actuator_ctrlrange[2 * actuator + 1]);
        }
        d->ctrl[actuator] = tau;
        state->last_tau[i] = tau;
    }
}

static double measure_leg_length(const mjModel *m, const mjData *d, const ModelMap *map, int left_leg)
{
    const int hip_a_joint = left_leg ? map->joint_id[JOINT_LEFT_REAR] : map->joint_id[JOINT_RIGHT_FRONT];
    const int hip_b_joint = left_leg ? map->joint_id[JOINT_LEFT_FRONT] : map->joint_id[JOINT_RIGHT_REAR];
    const char *wheel_joint_name = left_leg ? "jwheel_left" : "jwheel_right";
    int wheel_joint = mj_name2id(m, mjOBJ_JOINT, wheel_joint_name);

    if (wheel_joint < 0)
    {
        wheel_joint_name = left_leg ? "left_wheel" : "right_wheel";
        wheel_joint = mj_name2id(m, mjOBJ_JOINT, wheel_joint_name);
    }

    if (wheel_joint < 0)
    {
        return 0.0;
    }

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

static void print_control_result(const mjModel *m, const mjData *d, const ModelMap *map, const PosDebugState *state)
{
    const double measured_left = measure_leg_length(m, d, map, 1);
    const double measured_right = measure_leg_length(m, d, map, 0);

    printf("t=%7.3f | input=%s | visual command L=% .3f R=% .3f | solver L0 L=% .3f R=% .3f | l0 pitch L=% .3f R=% .3f | measured leg L=% .3f R=% .3f | "
           "target LR/LF/RF/RR=[% .3f % .3f % .3f % .3f] | "
           "q=[% .3f % .3f % .3f % .3f] | "
           "tau=[% .2f % .2f % .2f % .2f]\n",
           d->time,
           state->input_command,
           state->visual_left_leg_length,
           state->visual_right_leg_length,
           state->left_leg_length,
           state->right_leg_length,
           state->left_l0_pitch,
           state->right_l0_pitch,
           measured_left,
           measured_right,
           state->target[JOINT_LEFT_REAR],
           state->target[JOINT_LEFT_FRONT],
           state->target[JOINT_RIGHT_FRONT],
           state->target[JOINT_RIGHT_REAR],
           d->qpos[map->joint[JOINT_LEFT_REAR].qpos],
           d->qpos[map->joint[JOINT_LEFT_FRONT].qpos],
           d->qpos[map->joint[JOINT_RIGHT_FRONT].qpos],
           d->qpos[map->joint[JOINT_RIGHT_REAR].qpos],
           state->last_tau[JOINT_LEFT_REAR],
           state->last_tau[JOINT_LEFT_FRONT],
           state->last_tau[JOINT_RIGHT_FRONT],
           state->last_tau[JOINT_RIGHT_REAR]);
    fflush(stdout);
}

static void set_initial_target_pose(const mjModel *m, mjData *d, const ModelMap *map, PosDebugState *state)
{
    solve_targets(state);
    hold_base_pose(m, d, map, state);

    for (int i = 0; i < POS_DEBUG_LEG_JOINT_COUNT; ++i)
    {
        d->qpos[map->joint[i].qpos] = state->target[i];
        d->qvel[map->joint[i].qvel] = 0.0;
    }

    for (int i = 0; i < POS_DEBUG_WHEEL_COUNT; ++i)
    {
        d->qvel[map->wheel[i].qvel] = 0.0;
    }

    for (int i = 0; i < m->nu; ++i)
    {
        d->ctrl[i] = 0.0;
    }

    mj_forward(m, d);
}

static void print_initial_pose(const mjData *d, const ModelMap *map, const PosDebugState *state)
{
    printf("Initial left leg:  visual=% .3f solver_L0=% .3f l0_pitch=% .3f q=[% .3f % .3f] target=[% .3f % .3f]\n",
           state->visual_left_leg_length,
           state->left_leg_length,
           state->left_l0_pitch,
           d->qpos[map->joint[JOINT_LEFT_REAR].qpos],
           d->qpos[map->joint[JOINT_LEFT_FRONT].qpos],
           state->target[JOINT_LEFT_REAR],
           state->target[JOINT_LEFT_FRONT]);
    printf("Initial right leg: visual=% .3f solver_L0=% .3f l0_pitch=% .3f q=[% .3f % .3f] target=[% .3f % .3f]\n",
           state->visual_right_leg_length,
           state->right_leg_length,
           state->right_l0_pitch,
           d->qpos[map->joint[JOINT_RIGHT_FRONT].qpos],
           d->qpos[map->joint[JOINT_RIGHT_REAR].qpos],
           state->target[JOINT_RIGHT_FRONT],
           state->target[JOINT_RIGHT_REAR]);
}





static void update_keyboard_command(GLFWwindow *window, PosDebugState *state)
{
    const double length_rate = kKeyboardLengthRate;
    const double pitch_rate = kKeyboardPitchRate;
    const double dt = kKeyboardControlDt;
    const double length_step = length_rate * dt;
    const double pitch_step = pitch_rate * dt;

    clear_input_command(state);

    if (glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS)
    {
        state->visual_left_leg_length = kResetVisualLegLength;
        state->visual_right_leg_length = kResetVisualLegLength;
        state->left_l0_pitch = kResetL0Pitch;
        state->right_l0_pitch = kResetL0Pitch;
        append_input_command(state, "R reset");
        return;
    }
    if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS)
    {
        state->visual_left_leg_length += length_step;
        state->visual_right_leg_length += length_step;
        append_input_command(state, "UP both extend");
    }
    if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS)
    {
        state->visual_left_leg_length -= length_step;
        state->visual_right_leg_length -= length_step;
        append_input_command(state, "DOWN both retract");
    }
    if (glfwGetKey(window, GLFW_KEY_U) == GLFW_PRESS)
    {
        state->visual_left_leg_length += length_step;
        append_input_command(state, "U left extend");
    }
    if (glfwGetKey(window, GLFW_KEY_J) == GLFW_PRESS)
    {
        state->visual_left_leg_length -= length_step;
        append_input_command(state, "J left retract");
    }
    if (glfwGetKey(window, GLFW_KEY_O) == GLFW_PRESS)
    {
        state->visual_right_leg_length += length_step;
        append_input_command(state, "O right extend");
    }
    if (glfwGetKey(window, GLFW_KEY_L) == GLFW_PRESS)
    {
        state->visual_right_leg_length -= length_step;
        append_input_command(state, "L right retract");
    }
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
    {
        state->left_l0_pitch += pitch_step;
        state->right_l0_pitch += pitch_step;
        append_input_command(state, "A both pitch +");
    }
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
    {
        state->left_l0_pitch -= pitch_step;
        state->right_l0_pitch -= pitch_step;
        append_input_command(state, "D both pitch -");
    }
    if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS)
    {
        state->left_l0_pitch += pitch_step;
        append_input_command(state, "Q left pitch +");
    }
    if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS)
    {
        state->left_l0_pitch -= pitch_step;
        append_input_command(state, "E left pitch -");
    }
    if (glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS)
    {
        state->right_l0_pitch += pitch_step;
        append_input_command(state, "Z right pitch +");
    }
    if (glfwGetKey(window, GLFW_KEY_C) == GLFW_PRESS)
    {
        state->right_l0_pitch -= pitch_step;
        append_input_command(state, "C right pitch -");
    }

    state->visual_left_leg_length = clamp(state->visual_left_leg_length, state->min_leg_length, state->max_leg_length);
    state->visual_right_leg_length = clamp(state->visual_right_leg_length, state->min_leg_length, state->max_leg_length);
    state->left_l0_pitch = clamp(state->left_l0_pitch, state->min_l0_pitch, state->max_l0_pitch);
    state->right_l0_pitch = clamp(state->right_l0_pitch, state->min_l0_pitch, state->max_l0_pitch);
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
}

static void draw_overlay(mjrRect viewport, const PosDebugState *state)
{
    char left[512];
    char right[512];

    snprintf(left, sizeof(left),
             "Up/Down both extend/retract\n"
             "U/J left extend/retract\n"
             "O/L right extend/retract\n"
             "A/D both l0 pitch +/-\n"
             "Q/E left l0 pitch +/-\n"
             "Z/C right l0 pitch +/-\n"
             "R reset pose\n"
             "Esc quit");

    snprintf(right, sizeof(right),
             "left visual=% .3f solver_L0=% .3f pitch=% .3f target=[% .3f % .3f]\n"
             "right visual=% .3f solver_L0=% .3f pitch=% .3f target=[% .3f % .3f]\n"
             "kp=% .1f kd=% .1f max_tau=% .1f\n"
             "base held at z=% .2f",
             state->visual_left_leg_length,
             state->left_leg_length,
             state->left_l0_pitch,
             state->target[JOINT_LEFT_REAR],
             state->target[JOINT_LEFT_FRONT],
             state->visual_right_leg_length,
             state->right_leg_length,
             state->right_l0_pitch,
             state->target[JOINT_RIGHT_FRONT],
             state->target[JOINT_RIGHT_REAR],
             state->kp,
             state->kd,
             state->max_torque,
             state->hang_z);

    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, left, right, &g_context);
}

static int run_viewer(mjModel *m, mjData *d, const ModelMap *map, PosDebugState *state)
{
    if (!glfwInit())
    {
        fprintf(stderr, "Failed to initialize GLFW. If you are in WSL, check WSLg/X server.\n");
        return 1;
    }

    GLFWwindow *window = glfwCreateWindow(kViewerWidth, kViewerHeight, kWindowTitle, 0, 0);
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




    g_camera.distance = kCameraDistance;
    g_camera.azimuth = kCameraAzimuth;
    g_camera.elevation = kCameraElevation;
    g_camera.lookat[0] = kCameraLookatX;
    g_camera.lookat[1] = kCameraLookatY;
    g_camera.lookat[2] = kCameraLookatZ;




    
    mjv_makeScene(m, &g_scene, kSceneMaxGeom);
    mjr_makeContext(m, &g_context, mjFONTSCALE_150);

    glfwSetKeyCallback(window, keyboard_callback);
    glfwSetCursorPosCallback(window, mouse_move_callback);
    glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwSetScrollCallback(window, scroll_callback);

    double wall_start = glfwGetTime();
    double sim_start = d->time;
    double last_print_time = -state->print_period;

    while (!glfwWindowShouldClose(window))
    {
        update_keyboard_command(window, state);

        double elapsed = glfwGetTime() - wall_start;
        double target_time = sim_start + elapsed;

        while (d->time < target_time && !glfwWindowShouldClose(window))
        {
            hold_base_pose(m, d, map, state);
            apply_position_control(m, d, map, state);
            mj_step(m, d);
            hold_base_pose(m, d, map, state);
        }

        if (state->print_period > 0.0 && d->time - last_print_time >= state->print_period)
        {
            print_control_result(m, d, map, state);
            last_print_time = d->time;
        }

        mjv_updateScene(m, d, &g_option, 0, &g_camera, mjCAT_ALL, &g_scene);

        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
        mjr_render(viewport, &g_scene, &g_context);
        draw_overlay(viewport, state);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    mjr_freeContext(&g_context);
    mjv_freeScene(&g_scene);
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}

static int run_headless(mjModel *m, mjData *d, const ModelMap *map, PosDebugState *state, double sim_time)
{
    int steps = (int)(sim_time / m->opt.timestep);
    for (int i = 0; i < steps; ++i)
    {
        hold_base_pose(m, d, map, state);
        apply_position_control(m, d, map, state);
        mj_step(m, d);
        hold_base_pose(m, d, map, state);

        if (i % 500 == 0)
        {
            print_control_result(m, d, map, state);
        }
    }

    return 0;
}

int main(int argc, char **argv)
{
    const char *model_path = kDefaultModelPath;
    int headless = 0;
    double sim_time = kDefaultSimTime;

    PosDebugState state;
    memset(&state, 0, sizeof(state));
    state.kp = kDefaultKp;
    state.kd = kDefaultKd;
    state.max_torque = kDefaultMaxTorque;
    state.min_leg_length = kDefaultMinLegLength;
    state.max_leg_length = kDefaultMaxLegLength;
    state.min_l0_pitch = kDefaultMinL0Pitch;
    state.max_l0_pitch = kDefaultMaxL0Pitch;
    state.visual_left_leg_length = kDefaultVisualLegLength;
    state.visual_right_leg_length = kDefaultVisualLegLength;
    state.left_l0_pitch = kDefaultL0Pitch;
    state.right_l0_pitch = kDefaultL0Pitch;
    state.hang_z = kDefaultHangZ;
    state.print_period = kDefaultPrintPeriod;
    clear_input_command(&state);



    for (int i = 1; i < argc; ++i)
    {
        if (strcmp(argv[i], "--headless") == 0)
        {
            headless = 1;
        }
        else if (strcmp(argv[i], "--time") == 0 && i + 1 < argc)
        {
            sim_time = atof(argv[++i]);
        }
        else if (strcmp(argv[i], "--kp") == 0 && i + 1 < argc)
        {
            state.kp = atof(argv[++i]);
        }
        else if (strcmp(argv[i], "--kd") == 0 && i + 1 < argc)
        {
            state.kd = atof(argv[++i]);
        }
        else if (strcmp(argv[i], "--max-torque") == 0 && i + 1 < argc)
        {
            state.max_torque = atof(argv[++i]);
        }
        else if (strcmp(argv[i], "--hang-z") == 0 && i + 1 < argc)
        {
            state.hang_z = atof(argv[++i]);
        }
        else if (strcmp(argv[i], "--print-period") == 0 && i + 1 < argc)
        {
            state.print_period = atof(argv[++i]);
        }
        else if (strcmp(argv[i], "--no-print") == 0)
        {
            state.print_period = 0.0;
        }
        else
        {
            model_path = argv[i];
        }
    }

    char error[1024] = {0};
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
    print_model_map(m, &map);
    widen_active_joint_ranges(m, &map);
    save_hang_pose(m, d, &map, &state);
    set_initial_target_pose(m, d, &map, &state);

    printf("Position debug model: %s\n", model_path);
    printf("Left leg: U extend, J retract. Right leg: O extend, L retract.\n");
    printf("Both legs: Up extend, Down retract. L0 pitch: A/D both, Q/E left, Z/C right.\n");
    printf("R reset pose, Esc quit.\n");
    print_initial_pose(d, &map, &state);

    int result = headless ? run_headless(m, d, &map, &state, sim_time) : run_viewer(m, d, &map, &state);

    mj_deleteData(d);
    mj_deleteModel(m);
    return result;
}
