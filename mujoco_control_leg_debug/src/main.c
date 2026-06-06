#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rm_third_party/glfw.h"
#include "rm_third_party/mujoco.h"

#include "control_joint_space.h"
#include "control_task_space.h"
#include "debug_draw.h"
#include "leg_debug.h"
#include "model_helpers.h"

#ifndef LEG_DEBUG_DEFAULT_MODEL_PATH
#define LEG_DEBUG_DEFAULT_MODEL_PATH "/home/shun/MuJoCoBin/rm_control/mujoco_control_leg_debug/models/wheel_leg_debug.xml"
#endif

#ifndef LEG_DEBUG_DEFAULT_COLLISION_MODEL_PATH
#define LEG_DEBUG_DEFAULT_COLLISION_MODEL_PATH "/home/shun/MuJoCoBin/rm_control/mujoco_control_leg_debug/models/wheel_leg_debug_collision.xml"
#endif

static mjModel *g_model = 0;
static mjData *g_data = 0;
static const LegDebugModelMap *g_map = 0;
static LegDebugState *g_state = 0;
static mjvCamera g_camera;
static mjvOption g_option;
static mjvScene g_scene;
static mjrContext g_context;
static int g_button_left = 0;
static int g_button_middle = 0;
static int g_button_right = 0;
static int g_single_step_requested = 0;
static int g_time_rebase_requested = 0;
static double g_last_x = 0.0;
static double g_last_y = 0.0;

static void print_usage(const char *argv0)
{
    printf("Usage: %s [model.xml] [options]\n", argv0);
    printf("Options:\n");
    printf("  --collision           load wheel_leg_debug_collision.xml (hold_ff off by default)\n");
    printf("  --headless            run without viewer\n");
    printf("  --time SEC            headless simulation time\n");
    printf("  --mode task|joint     initial control mode\n");
    printf("  --paused              start viewer paused\n");
    printf("  --no-control          start with zero actuator output\n");
    printf("  --no-draw             hide debug helper lines\n");
    printf("  --gravity-ff          enable MuJoCo inverse-dynamics hold feedforward\n");
    printf("  --no-gravity-ff       disable MuJoCo inverse-dynamics hold feedforward\n");
    printf("  --print-period SEC    set log period\n");
    printf("  --no-print            disable periodic log\n");
    printf("  --joint-kp VALUE      joint PD kp (debug default 80)\n");
    printf("  --joint-kd VALUE      joint PD kd (debug default 4)\n");
    printf("  --max-torque VALUE    joint torque clamp\n");
    printf("  --hold-ff-period SEC  inverse-dynamics feedforward refresh period\n");
    printf("  --firmware-gains      use local firmware-like kp/kd and torque limit\n");
    printf("  --length-rate VALUE   task-space L0 keyboard rate\n");
    printf("  --phi0-rate VALUE     task-space phi0 keyboard rate\n");
    printf("  --joint-rate VALUE    joint-space keyboard rate\n");
    printf("  --init-key NAME       reset from model keyframe\n");
}

static void apply_scripted_demo(LegDebugState *state, double sim_time)
{
    if (state->control_mode != LEG_DEBUG_CONTROL_TASK_SPACE)
    {
        return;
    }

    leg_debug_clear_input_command(state);
    state->control_enabled = 1;

    if (sim_time < 0.5)
    {
        state->left_l0_cmd = state->home_left_l0_cmd;
        state->right_l0_cmd = state->home_right_l0_cmd;
        state->left_phi0_cmd = state->home_left_phi0_cmd;
        state->right_phi0_cmd = state->home_right_phi0_cmd;
        leg_debug_append_input_command(state, "script hold");
    }
    else if (sim_time < 1.5)
    {
        state->left_l0_cmd = state->home_left_l0_cmd + 0.03;
        state->right_l0_cmd = state->home_right_l0_cmd + 0.03;
        leg_debug_append_input_command(state, "script L0 +");
    }
    else if (sim_time < 2.5)
    {
        state->left_l0_cmd = state->home_left_l0_cmd - 0.03;
        state->right_l0_cmd = state->home_right_l0_cmd - 0.03;
        leg_debug_append_input_command(state, "script L0 -");
    }
    else if (sim_time < 3.5)
    {
        state->left_phi0_cmd = state->home_left_phi0_cmd + 0.12;
        state->right_phi0_cmd = state->home_right_phi0_cmd + 0.12;
        leg_debug_append_input_command(state, "script phi0 +");
    }
    else
    {
        state->left_l0_cmd = state->home_left_l0_cmd;
        state->right_l0_cmd = state->home_right_l0_cmd;
        state->left_phi0_cmd = state->home_left_phi0_cmd;
        state->right_phi0_cmd = state->home_right_phi0_cmd;
        leg_debug_append_input_command(state, "script reset");
    }

    state->left_l0_cmd = leg_debug_clamp(state->left_l0_cmd, state->min_leg_length, state->max_leg_length);
    state->right_l0_cmd = leg_debug_clamp(state->right_l0_cmd, state->min_leg_length, state->max_leg_length);
}

static void apply_leg_control(const mjModel *m, mjData *d, const LegDebugModelMap *map, LegDebugState *state)
{
    LegDebugState previous;
    double tau_ff[LEG_DEBUG_LEG_JOINT_COUNT] = {0.0, 0.0, 0.0, 0.0};

    for (int i = 0; i < m->nu; ++i)
    {
        d->ctrl[i] = 0.0;
    }

    leg_debug_update_measurements(m, d, map, state);
    previous = *state;

    if (!state->control_enabled)
    {
        for (int i = 0; i < LEG_DEBUG_LEG_JOINT_COUNT; ++i)
        {
            state->last_tau_pd[i] = 0.0;
            state->last_tau_ff[i] = 0.0;
            state->last_tau[i] = 0.0;
        }
        return;
    }

    if (state->control_mode == LEG_DEBUG_CONTROL_TASK_SPACE)
    {
        if (!leg_debug_task_space_solve_targets(d, map, &previous, state))
        {
            for (int i = 0; i < LEG_DEBUG_LEG_JOINT_COUNT; ++i)
            {
                state->last_tau_pd[i] = 0.0;
                state->last_tau_ff[i] = 0.0;
                state->last_tau[i] = 0.0;
            }
            return;
        }
    }
    leg_debug_update_target_vmc(state);

    if (state->use_gravity_ff)
    {
        const int refresh_hold_ff = state->last_hold_ff_time < -0.5 ||
                                    state->hold_ff_period <= 0.0 ||
                                    d->time - state->last_hold_ff_time >= state->hold_ff_period;
        if (refresh_hold_ff)
        {
            memset(d->qacc, 0, sizeof(mjtNum) * m->nv);
            mj_inverse(m, d);
            for (int i = 0; i < LEG_DEBUG_LEG_JOINT_COUNT; ++i)
            {
                state->last_tau_ff[i] = d->qfrc_inverse[map->joint[i].qvel];
            }
            state->last_hold_ff_time = d->time;
        }
        for (int i = 0; i < LEG_DEBUG_LEG_JOINT_COUNT; ++i)
        {
            tau_ff[i] = state->last_tau_ff[i];
        }
    }
    else
    {
        for (int i = 0; i < LEG_DEBUG_LEG_JOINT_COUNT; ++i)
        {
            state->last_tau_ff[i] = 0.0;
        }
        state->last_hold_ff_time = -1.0;
    }

    for (int i = 0; i < LEG_DEBUG_LEG_JOINT_COUNT; ++i)
    {
        const int actuator = map->actuator[i];
        const double q = d->qpos[map->joint[i].qpos];
        const double qd = d->qvel[map->joint[i].qvel];
        const double tau_pd = state->joint_kp * (state->target_q[i] - q) - state->joint_kd * qd;
        double tau = tau_pd + tau_ff[i];
        double ctrl;

        tau = leg_debug_clamp(tau, -state->max_joint_torque, state->max_joint_torque);
        state->last_tau_pd[i] = tau_pd;
        state->last_tau[i] = tau;

        ctrl = state->joint_ctrl_sign[i] * tau;
        if (m->actuator_ctrllimited[actuator])
        {
            ctrl = leg_debug_clamp(ctrl, m->actuator_ctrlrange[2 * actuator], m->actuator_ctrlrange[2 * actuator + 1]);
        }
        d->ctrl[actuator] = ctrl;
    }

    if (LEG_DEBUG_ACTUATOR_COUNT >= 6)
    {
        d->ctrl[map->actuator[4]] = 0.0;
        d->ctrl[map->actuator[5]] = 0.0;
    }
}




// 
static void reset_mode_home_from_current_pose(mjModel *m, mjData *d, const LegDebugModelMap *map, LegDebugState *state)
{
    // 初始化 VMC 相关状态
    leg_debug_initialize_vmc_runtime(state);
    // 让 MuJoCo 根据当前的 qpos 和 qvel 更新派生量
    mj_forward(m, d);
    // 根据当前关节角用 vmc 计算测量值
    leg_debug_update_measurements(m, d, map, state);
    // 给 task space 模式设置 home ，把当前的末端位置作为 任务空间的初始目标。
    leg_debug_task_space_reset_home(map, d, state);
    // 给 joint space 模式设置 home ，把当前的关节角作为 关节空间的初始目标。
    leg_debug_joint_space_reset_home(map, d, state);
    // 如果是 task sapce 模式，计算一次 target vmc 以更新 target_l0_vmc 和 target_phi0_vmc，使得它们和测量值一致，从而避免 reset 后的第一步出现大跳变。
    if (state->control_mode == LEG_DEBUG_CONTROL_TASK_SPACE)
    {
        LegDebugState previous = *state;
        leg_debug_task_space_solve_targets(d, map, &previous, state);
    }
    leg_debug_update_target_vmc(state);
}



static void reset_simulation_to_xml_initial(mjModel *m, mjData *d, const LegDebugModelMap *map, LegDebugState *state)
{
    const LegDebugControlMode mode = state->control_mode;
    const int control_enabled = state->control_enabled;
    const int paused = state->paused;
    const int show_debug_draw = state->show_debug_draw;

    if (!leg_debug_reset_data(m, d, map, state->init_key_name))
    {
        leg_debug_clear_input_command(state);
        leg_debug_append_input_command(state, "R reset failed");
        return;
    }

    state->control_mode = mode;
    state->control_enabled = control_enabled;
    state->paused = paused;
    state->show_debug_draw = show_debug_draw;
    reset_mode_home_from_current_pose(m, d, map, state);
    leg_debug_infer_ctrl_signs(m, d, map, state);
    leg_debug_update_measurements(m, d, map, state);
    memset(d->ctrl, 0, sizeof(mjtNum) * m->nu);
    state->last_hold_ff_time = -1.0;

    leg_debug_clear_input_command(state);
    leg_debug_append_input_command(state, "R xml reset");
    g_single_step_requested = 0;
    g_time_rebase_requested = 1;
}

static void toggle_control_mode(mjModel *m, mjData *d, const LegDebugModelMap *map, LegDebugState *state)
{
    if (state->control_mode == LEG_DEBUG_CONTROL_TASK_SPACE)
    {
        state->control_mode = LEG_DEBUG_CONTROL_JOINT_SPACE;
        leg_debug_update_measurements(m, d, map, state);
        leg_debug_joint_space_reset_home(map, d, state);
        leg_debug_update_target_vmc(state);
        leg_debug_clear_input_command(state);
        leg_debug_append_input_command(state, "M joint-space");
    }
    else
    {
        state->control_mode = LEG_DEBUG_CONTROL_TASK_SPACE;
        leg_debug_update_measurements(m, d, map, state);
        leg_debug_task_space_reset_home(map, d, state);
        LegDebugState previous = *state;
        leg_debug_task_space_solve_targets(d, map, &previous, state);
        leg_debug_update_target_vmc(state);
        leg_debug_clear_input_command(state);
        leg_debug_append_input_command(state, "M task-space");
    }
    state->control_enabled = 1;
}

static int handle_global_key(GLFWwindow *window, int key, int action)
{
    if (action == GLFW_PRESS && key == GLFW_KEY_ESCAPE)
    {
        glfwSetWindowShouldClose(window, GLFW_TRUE);
        return 1;
    }

    if (!g_state || action != GLFW_PRESS)
    {
        return 0;
    }

    if (key == GLFW_KEY_M)
    {
        toggle_control_mode(g_model, g_data, g_map, g_state);
        return 1;
    }
    if (key == GLFW_KEY_R)
    {
        reset_simulation_to_xml_initial(g_model, g_data, g_map, g_state);
        return 1;
    }
    if (key == GLFW_KEY_P)
    {
        g_state->paused = !g_state->paused;
        leg_debug_clear_input_command(g_state);
        leg_debug_append_input_command(g_state, g_state->paused ? "P pause" : "P run");
        return 1;
    }
    if (key == GLFW_KEY_SPACE)
    {
        g_state->control_enabled = !g_state->control_enabled;
        leg_debug_clear_input_command(g_state);
        leg_debug_append_input_command(g_state, g_state->control_enabled ? "control on" : "control off");
        return 1;
    }
    if (key == GLFW_KEY_B)
    {
        g_state->show_debug_draw = !g_state->show_debug_draw;
        leg_debug_clear_input_command(g_state);
        leg_debug_append_input_command(g_state, g_state->show_debug_draw ? "B draw on" : "B draw off");
        return 1;
    }
    if (key == GLFW_KEY_G)
    {
        g_state->use_gravity_ff = !g_state->use_gravity_ff;
        leg_debug_clear_input_command(g_state);
        leg_debug_append_input_command(g_state, g_state->use_gravity_ff ? "G hold ff on" : "G hold ff off");
        return 1;
    }
    if (key == GLFW_KEY_N)
    {
        g_single_step_requested = 1;
        leg_debug_clear_input_command(g_state);
        leg_debug_append_input_command(g_state, "N single step");
        return 1;
    }

    return 0;
}

static void keyboard_callback(GLFWwindow *window, int key, int scancode, int action, int mods)
{
    (void)scancode;
    (void)mods;

    if (action != GLFW_PRESS && action != GLFW_REPEAT)
    {
        return;
    }

    if (handle_global_key(window, key, action))
    {
        printf("[key] %s\n", g_state ? g_state->input_command : "global");
        fflush(stdout);
        return;
    }
    if (key == GLFW_KEY_R)
    {
        return;
    }

    if (!g_state)
    {
        return;
    }

    if (g_state->control_mode == LEG_DEBUG_CONTROL_TASK_SPACE)
    {
        leg_debug_task_space_apply_key(g_state, key);
    }
    else
    {
        leg_debug_joint_space_apply_key(g_state, key);
    }

    if (strcmp(g_state->input_command, "none") != 0)
    {
        printf("[key] %s\n", g_state->input_command);
        fflush(stdout);
    }
}

static void mouse_button_callback(GLFWwindow *window, int button, int action, int mods)
{
    (void)button;
    (void)action;
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

    const double dx = xpos - g_last_x;
    const double dy = ypos - g_last_y;
    int width = 0;
    int height = 0;
    int shift;
    mjtMouse action;

    g_last_x = xpos;
    g_last_y = ypos;
    glfwGetWindowSize(window, &width, &height);

    shift = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
            glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;

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

static void draw_overlay(mjrRect viewport, const LegDebugState *state)
{
    char left[768];
    char right[1024];

    snprintf(left, sizeof(left),
             "M mode  P pause  N step  Space control  B lines\n"
             "G inverse-dynamics hold feedforward\n"
             "task: Up/Down both L0, U/J left, O/L right\n"
             "task: A/D both phi0, Q/E left, Z/C right\n"
             "joint: 1-4 select, Left/Right select\n"
             "joint: Up/Down selected q, [/] selected q, ,/. all q\n"
             "R reset XML initial pose  Esc quit");

    snprintf(right, sizeof(right),
             "mode=%s paused=%d control=%d draw=%d ncon=%d input=%s\n"
             "hold_ff=%d ff_dt=%.3f kp=%.1f kd=%.1f max_tau=%.1f\n"
             "ik_L0_cmd L=% .3f R=% .3f ik_phi0_cmd L=% .3f R=% .3f\n"
             "target_L0_vmc L=% .3f R=% .3f | L0_vmc L=% .3f R=% .3f\n"
             "L0_world L=% .3f R=% .3f\n"
             "phi0_vmc L=% .3f R=% .3f theta L=% .3f R=% .3f\n"
             "selected=%s target=[% .2f % .2f % .2f % .2f]\n"
             "q=[% .2f % .2f % .2f % .2f] err=[% .2f % .2f % .2f % .2f]\n"
             "pd=[% .1f % .1f % .1f % .1f] ff=[% .1f % .1f % .1f % .1f]\n"
             "tau=[% .1f % .1f % .1f % .1f]",
             leg_debug_control_mode_name(state->control_mode),
             state->paused,
             state->control_enabled,
             state->show_debug_draw,
             g_data ? g_data->ncon : 0,
             state->input_command,
             state->use_gravity_ff,
             state->hold_ff_period,
             state->joint_kp,
             state->joint_kd,
             state->max_joint_torque,
             state->left_l0_cmd,
             state->right_l0_cmd,
             state->left_phi0_cmd,
             state->right_phi0_cmd,
             state->target_left_l0_vmc,
             state->target_right_l0_vmc,
             state->measured_left_l0_vmc,
             state->measured_right_l0_vmc,
             state->left_world.l0_world,
             state->right_world.l0_world,
             state->measured_left_phi0_vmc,
             state->measured_right_phi0_vmc,
             state->measured_left_theta,
             state->measured_right_theta,
             kLegDebugJointShortName[state->selected_joint],
             state->target_q[0],
             state->target_q[1],
             state->target_q[2],
             state->target_q[3],
             state->q[0],
             state->q[1],
             state->q[2],
             state->q[3],
             state->target_q[0] - state->q[0],
             state->target_q[1] - state->q[1],
             state->target_q[2] - state->q[2],
             state->target_q[3] - state->q[3],
             state->last_tau_pd[0],
             state->last_tau_pd[1],
             state->last_tau_pd[2],
             state->last_tau_pd[3],
             state->last_tau_ff[0],
             state->last_tau_ff[1],
             state->last_tau_ff[2],
             state->last_tau_ff[3],
             state->last_tau[0],
             state->last_tau[1],
             state->last_tau[2],
             state->last_tau[3]);

    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, left, right, &g_context);
}

static int run_viewer(mjModel *m, mjData *d, const LegDebugModelMap *map, LegDebugState *state)
{
    if (!glfwInit())
    {
        fprintf(stderr, "Failed to initialize GLFW. If you are in WSL, check WSLg/X server.\n");
        return 1;
    }

    GLFWwindow *window = glfwCreateWindow(kLegDebugViewerWidth, kLegDebugViewerHeight, "mujoco leg debug", 0, 0);
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
    g_map = map;
    g_state = state;

    mjv_defaultCamera(&g_camera);
    mjv_defaultOption(&g_option);
    mjv_defaultScene(&g_scene);
    mjr_defaultContext(&g_context);

    g_camera.distance = 2.0;
    g_camera.azimuth = 135.0;
    g_camera.elevation = -20.0;
    g_camera.lookat[0] = 0.0;
    g_camera.lookat[1] = 0.0;
    g_camera.lookat[2] = 0.35;

    mjv_makeScene(m, &g_scene, kLegDebugSceneMaxGeom);
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
        if (g_time_rebase_requested)
        {
            wall_start = glfwGetTime();
            sim_start = d->time;
            last_print_time = -state->print_period;
            g_time_rebase_requested = 0;
        }

        if (state->scripted_demo)
        {
            apply_scripted_demo(state, d->time);
        }

        if (state->paused && !g_single_step_requested)
        {
            wall_start = glfwGetTime();
            sim_start = d->time;
        }
        else if (g_single_step_requested)
        {
            apply_leg_control(m, d, map, state);
            mj_step(m, d);
            g_single_step_requested = 0;
            wall_start = glfwGetTime();
            sim_start = d->time;
        }
        else
        {
            const double elapsed = glfwGetTime() - wall_start;
            const double target_time = sim_start + elapsed;
            while (d->time < target_time && !glfwWindowShouldClose(window))
            {
                apply_leg_control(m, d, map, state);
                mj_step(m, d);
            }
        }

        leg_debug_update_measurements(m, d, map, state);

        if (state->print_period > 0.0 && d->time - last_print_time >= state->print_period)
        {
            leg_debug_log_state(m, d, map, state);
            last_print_time = d->time;
        }

        mjv_updateScene(m, d, &g_option, 0, &g_camera, mjCAT_ALL, &g_scene);
        leg_debug_draw_scene(&g_scene, state);

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
    g_state = 0;
    g_map = 0;
    g_data = 0;
    g_model = 0;
    return 0;
}

static int run_headless(mjModel *m, mjData *d, const LegDebugModelMap *map, LegDebugState *state, double sim_time)
{
    const int steps = (int)(sim_time / m->opt.timestep);
    int print_stride = state->print_period > 0.0 ? (int)(state->print_period / m->opt.timestep) : 0;
    if (print_stride < 1)
    {
        print_stride = 1;
    }

    for (int i = 0; i < steps; ++i)
    {
        if (state->scripted_demo)
        {
            apply_scripted_demo(state, d->time);
        }
        apply_leg_control(m, d, map, state);
        mj_step(m, d);

        if (state->print_period > 0.0 && i % print_stride == 0)
        {
            leg_debug_log_state(m, d, map, state);
        }
    }

    return 0;
}

int main(int argc, char **argv)
{
    const char *model_path = LEG_DEBUG_DEFAULT_MODEL_PATH;
    int headless = 0;
    int collision_model_requested = 0;
    int gravity_ff_explicit = 0;
    double sim_time = 5.0;

    LegDebugState state;
    memset(&state, 0, sizeof(state));

    state.control_mode = LEG_DEBUG_CONTROL_TASK_SPACE;
    state.control_enabled = 1;
    state.paused = 0;
    state.show_debug_draw = 1;
    state.use_gravity_ff = 1;
    state.selected_joint = LEG_DEBUG_JOINT_LEFT_FRONT;
    state.joint_kp = kLegDebugDefaultJointKp;
    state.joint_kd = kLegDebugDefaultJointKd;
    state.max_joint_torque = kLegDebugDefaultMaxJointTorque;
    state.hold_ff_period = kLegDebugDefaultHoldFfPeriod;
    state.last_hold_ff_time = -1.0;
    state.min_leg_length = LEG_DEBUG_PARAM_MIN_LEG_LENGTH;
    state.max_leg_length = fmax(LEG_DEBUG_PARAM_MAX_LEG_LENGTH, kLegDebugDefaultMaxLegLength);
    state.keyboard_length_rate = kLegDebugDefaultLengthRate;
    state.keyboard_phi0_rate = kLegDebugDefaultPhi0Rate;
    state.keyboard_joint_rate = kLegDebugDefaultJointRate;
    state.print_period = kLegDebugDefaultPrintPeriod;
    leg_debug_initialize_vmc_runtime(&state);
    leg_debug_clear_input_command(&state);

    for (int i = 1; i < argc; ++i)
    {
        if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0)
        {
            print_usage(argv[0]);
            return 0;
        }
        if (strcmp(argv[i], "--collision") == 0)
        {
            model_path = LEG_DEBUG_DEFAULT_COLLISION_MODEL_PATH;
            collision_model_requested = 1;
        }
        else if (strcmp(argv[i], "--headless") == 0)
        {
            headless = 1;
        }
        else if (strcmp(argv[i], "--time") == 0 && i + 1 < argc)
        {
            sim_time = atof(argv[++i]);
        }
        else if (strcmp(argv[i], "--mode") == 0 && i + 1 < argc)
        {
            const char *mode = argv[++i];
            if (strcmp(mode, "joint") == 0 || strcmp(mode, "joint-space") == 0)
            {
                state.control_mode = LEG_DEBUG_CONTROL_JOINT_SPACE;
            }
            else if (strcmp(mode, "task") == 0 || strcmp(mode, "task-space") == 0)
            {
                state.control_mode = LEG_DEBUG_CONTROL_TASK_SPACE;
            }
            else
            {
                fprintf(stderr, "Unknown mode: %s\n", mode);
                return 1;
            }
        }
        else if (strcmp(argv[i], "--paused") == 0)
        {
            state.paused = 1;
        }
        else if (strcmp(argv[i], "--no-control") == 0)
        {
            state.control_enabled = 0;
        }
        else if (strcmp(argv[i], "--no-draw") == 0)
        {
            state.show_debug_draw = 0;
        }
        else if (strcmp(argv[i], "--gravity-ff") == 0)
        {
            state.use_gravity_ff = 1;
            gravity_ff_explicit = 1;
        }
        else if (strcmp(argv[i], "--no-gravity-ff") == 0)
        {
            state.use_gravity_ff = 0;
            gravity_ff_explicit = 1;
        }
        else if (strcmp(argv[i], "--scripted-demo") == 0)
        {
            state.scripted_demo = 1;
        }
        else if (strcmp(argv[i], "--print-period") == 0 && i + 1 < argc)
        {
            state.print_period = atof(argv[++i]);
        }
        else if (strcmp(argv[i], "--no-print") == 0)
        {
            state.print_period = 0.0;
        }
        else if (strcmp(argv[i], "--joint-kp") == 0 && i + 1 < argc)
        {
            state.joint_kp = atof(argv[++i]);
        }
        else if (strcmp(argv[i], "--joint-kd") == 0 && i + 1 < argc)
        {
            state.joint_kd = atof(argv[++i]);
        }
        else if (strcmp(argv[i], "--max-torque") == 0 && i + 1 < argc)
        {
            state.max_joint_torque = atof(argv[++i]);
        }
        else if (strcmp(argv[i], "--hold-ff-period") == 0 && i + 1 < argc)
        {
            state.hold_ff_period = atof(argv[++i]);
            state.last_hold_ff_time = -1.0;
        }
        else if (strcmp(argv[i], "--firmware-gains") == 0)
        {
            state.joint_kp = LEG_DEBUG_PARAM_FIRMWARE_POS_KP;
            state.joint_kd = LEG_DEBUG_PARAM_FIRMWARE_POS_KD;
            state.max_joint_torque = LEG_DEBUG_PARAM_FIRMWARE_MAX_TORQUE;
        }
        else if (strcmp(argv[i], "--length-rate") == 0 && i + 1 < argc)
        {
            state.keyboard_length_rate = atof(argv[++i]);
        }
        else if (strcmp(argv[i], "--phi0-rate") == 0 && i + 1 < argc)
        {
            state.keyboard_phi0_rate = atof(argv[++i]);
        }
        else if (strcmp(argv[i], "--joint-rate") == 0 && i + 1 < argc)
        {
            state.keyboard_joint_rate = atof(argv[++i]);
        }
        else if (strcmp(argv[i], "--init-key") == 0 && i + 1 < argc)
        {
            snprintf(state.init_key_name, sizeof(state.init_key_name), "%s", argv[++i]);
        }
        else
        {
            model_path = argv[i];
        }
    }

    const int using_collision_model =
        collision_model_requested || strstr(model_path, "wheel_leg_debug_collision") != NULL;
    if (using_collision_model && !gravity_ff_explicit)
    {
        state.use_gravity_ff = 0;
    }

    if (state.max_leg_length < state.min_leg_length)
    {
        const double tmp = state.min_leg_length;
        state.min_leg_length = state.max_leg_length;
        state.max_leg_length = tmp;
    }

    char error[1024] = {0};
    mjModel *m = mj_loadXML(model_path, 0, error, sizeof(error));
    if (!m)
    {
        fprintf(stderr, "Failed to load MuJoCo model: %s\n%s\n", model_path, error);
        return 1;
    }

    mjData *d = mj_makeData(m);
    if (!d)
    {
        fprintf(stderr, "Failed to allocate MuJoCo data.\n");
        mj_deleteModel(m);
        return 1;
    }

    LegDebugModelMap map;
    if (!leg_debug_build_model_map(m, &map))
    {
        mj_deleteData(d);
        mj_deleteModel(m);
        return 1;
    }

    if (!leg_debug_reset_data(m, d, &map, state.init_key_name))
    {
        mj_deleteData(d);
        mj_deleteModel(m);
        return 1;
    }
// 启动时调用 reset home 函数，
    reset_mode_home_from_current_pose(m, d, &map, &state);
    leg_debug_infer_ctrl_signs(m, d, &map, &state);
    leg_debug_update_measurements(m, d, &map, &state);

    leg_debug_print_model_map(m, &map);
    printf("Leg debug model: %s\n", model_path);
    printf("Mode: %s | control=%d | draw=%d | hold_ff=%d ff_dt=%.3f | kp=%.3f kd=%.3f max_tau=%.3f | print_period=%.3f\n",
           leg_debug_control_mode_name(state.control_mode),
           state.control_enabled,
           state.show_debug_draw,
           state.use_gravity_ff,
           state.hold_ff_period,
           state.joint_kp,
           state.joint_kd,
           state.max_joint_torque,
           state.print_period);
    printf("Initial L0_vmc L=% .3f R=% .3f | L0_world L=% .3f R=% .3f | phi0_vmc L=% .3f R=% .3f\n",
           state.measured_left_l0_vmc,
           state.measured_right_l0_vmc,
           state.left_world.l0_world,
           state.right_world.l0_world,
           state.measured_left_phi0_vmc,
           state.measured_right_phi0_vmc);
    printf("Joint ctrl sign LF=%+.0f LR=%+.0f RR=%+.0f RF=%+.0f\n",
           state.joint_ctrl_sign[0],
           state.joint_ctrl_sign[1],
           state.joint_ctrl_sign[2],
           state.joint_ctrl_sign[3]);

    const int result = headless ? run_headless(m, d, &map, &state, sim_time) : run_viewer(m, d, &map, &state);

    mj_deleteData(d);
    mj_deleteModel(m);
    return result;
}
