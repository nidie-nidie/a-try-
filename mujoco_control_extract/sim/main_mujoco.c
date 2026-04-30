#include "sim_adapter.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

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
} ModelMap;

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
static mjtNum g_joint_ctrl_sign[4] = {1.0, 1.0, 1.0, 1.0};
static mjtNum g_wheel_ctrl_sign[2] = {1.0, 1.0};

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
    JointRef ref = {id, m->jnt_qposadr[id], m->jnt_dofadr[id]};
    return ref;
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

        map->joint[0] = find_joint(m, "jIO");
        map->joint[1] = find_joint(m, "jIJ");
        map->joint[2] = find_joint(m, "jAB");
        map->joint[3] = find_joint(m, "jAG");
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
    const double phi0 = M_PI_2;
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

static void set_initial_stand_pose(mjModel *m, mjData *d, const ModelMap *map, int freeze_init, const char *init_key_name)
{
    const mjtNum base_pos[3] = {0.0, 0.0, freeze_init ? 0.55 : 0.42};
    const mjtNum base_quat[4] = {1.0, 0.0, 0.0, 0.0};
    mjtNum joint_qpos[4];
    int base_qpos = m->jnt_qposadr[map->base_freejoint];
    int base_qvel = m->jnt_dofadr[map->base_freejoint];

    calc_initial_stand_joint_qpos(joint_qpos);

    const int init_key = find_optional_id(m, mjOBJ_KEY, init_key_name);
    if (init_key >= 0)
    {
        mj_resetDataKeyframe(m, d, init_key);
        for (int i = 0; i < 4; ++i)
        {
            d->qpos[map->joint[i].qpos] = joint_qpos[i];
        }
        memset(d->qvel, 0, sizeof(mjtNum) * m->nv);
        memset(d->ctrl, 0, sizeof(mjtNum) * m->nu);
        mj_forward(m, d);
        relax_closed_chain_pose(m, d, map);

        if (freeze_init)
        {
            return;
        }
        goto adjust_wheel_height;
    }

    for (int i = 0; i < 3; ++i)
    {
        d->qpos[base_qpos + i] = base_pos[i];
    }
    for (int i = 0; i < 4; ++i)
    {
        d->qpos[base_qpos + 3 + i] = base_quat[i];
    }
    for (int i = 0; i < 6; ++i)
    {
        d->qvel[base_qvel + i] = 0.0;
    }

    for (int i = 0; i < 4; ++i)
    {
        d->qpos[map->joint[i].qpos] = joint_qpos[i];
        d->qvel[map->joint[i].qvel] = 0.0;
    }

    for (int i = 0; i < 2; ++i)
    {
        d->qvel[map->wheel[i].qvel] = 0.0;
    }

    mj_forward(m, d);
    relax_closed_chain_pose(m, d, map);

    if (freeze_init)
    {
        return;
    }

adjust_wheel_height:
    int wheel_bodies[4] = {
        find_optional_id(m, mjOBJ_BODY, "left_wheel"),
        find_optional_id(m, mjOBJ_BODY, "right_wheel"),
        find_optional_id(m, mjOBJ_BODY, "wheel_left"),
        find_optional_id(m, mjOBJ_BODY, "wheel_right"),
    };
    mjtNum min_wheel_z = 1.0e9;
    int wheel_count = 0;
    for (int i = 0; i < 4; ++i)
    {
        if (wheel_bodies[i] >= 0)
        {
            mjtNum wheel_z = d->xpos[3 * wheel_bodies[i] + 2];
            if (wheel_z < min_wheel_z)
            {
                min_wheel_z = wheel_z;
            }
            ++wheel_count;
        }
    }

    if (wheel_count > 0)
    {
        d->qpos[base_qpos + 2] += WHEEL_RADIUS - min_wheel_z;
        mj_forward(m, d);
    }
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
        state->wheel_vel[i] = (float)d->qvel[map->wheel[i].qvel];
    }

    int base_qpos = m->jnt_qposadr[map->base_freejoint];
    int base_qvel = m->jnt_dofadr[map->base_freejoint];
    quat_to_euler(&d->qpos[base_qpos + 3], &state->roll, &state->pitch, &state->yaw);

    state->gyro[0] = (float)d->qvel[base_qvel + 3];
    state->gyro[1] = (float)d->qvel[base_qvel + 4];
    state->gyro[2] = (float)d->qvel[base_qvel + 5];
    state->body_x = (float)d->qpos[base_qpos + 0];
    state->body_y = (float)d->qpos[base_qpos + 1];
    state->body_z = (float)d->qpos[base_qpos + 2];
    state->body_v = (float)d->qvel[base_qvel + 0];
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





static void write_output(mjData *d, const ModelMap *map, const SimControllerOutput *output)
{
    d->ctrl[map->actuator[0]] = g_joint_ctrl_sign[0] * output->joint_torque[0];
    d->ctrl[map->actuator[1]] = g_joint_ctrl_sign[1] * output->joint_torque[1];
    d->ctrl[map->actuator[2]] = g_joint_ctrl_sign[2] * output->joint_torque[2];
    d->ctrl[map->actuator[3]] = g_joint_ctrl_sign[3] * output->joint_torque[3];
    d->ctrl[map->actuator[4]] = output->wheel_torque[0];
    d->ctrl[map->actuator[5]] = output->wheel_torque[1];
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

static void step_controller(const mjModel *m, mjData *d, const ModelMap *map, int print_line, int zero_control, int zero_wheels, int invert_right_joints)
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
        SimController_SetState(&state);
        SimController_SetCommand(0.0f, state.body_x, INIT_LEG_LENGTH, 0.0f, 0.0f);
        SimController_Step((float)m->opt.timestep);
        SimController_GetOutput(&output);
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
    }
    write_output(d, map, &output);

    mj_step(m, d);

    if (print_line)
    {
        const double measured_left = measure_leg_length(m, d, map, 1);
        const double measured_right = measure_leg_length(m, d, map, 0);

        printf("t=%6.3f pos=[% .3f % .3f % .3f] rpy=[% .3f % .3f % .3f] "
               "vmcL0=[% .3f % .3f] measL=[% .3f % .3f] "
               "phi1=[% .3f % .3f] phi4=[% .3f % .3f] "
               "u=[% .2f % .2f % .2f % .2f | % .2f % .2f]\n",
               d->time,
               state.body_x,
               state.body_y,
               state.body_z,
               state.roll,
               state.pitch,
               state.yaw,
               left.L0,
               right.L0,
               measured_left,
               measured_right,
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
    }
}

static int run_headless(const mjModel *m, mjData *d, const ModelMap *map, double sim_time, int zero_control, int zero_wheels, int invert_right_joints, int freeze_init)
{
    if (freeze_init)
    {
        printf("Initial state frozen at t=%6.3f\n", d->time);
        return 0;
    }

    int steps = (int)(sim_time / m->opt.timestep);
    for (int i = 0; i < steps; ++i)
    {
        step_controller(m, d, map, i % 500 == 0, zero_control, zero_wheels, invert_right_joints);
    }

    return 0;
}

static int run_viewer(mjModel *m, mjData *d, const ModelMap *map, double sim_time, int zero_control, int zero_wheels, int invert_right_joints, int freeze_init)
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

    double wall_start = glfwGetTime();
    double sim_start = d->time;
    int print_tick = 0;

    while (!glfwWindowShouldClose(window) && d->time < sim_time)
    {
        double elapsed = glfwGetTime() - wall_start;
        double target_time = sim_start + elapsed;

        while (!freeze_init && d->time < target_time && !glfwWindowShouldClose(window))
        {
            step_controller(m, d, map, print_tick % 500 == 0, zero_control, zero_wheels, invert_right_joints);
            ++print_tick;
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
    const char *model_path = "sim/models/rm_robot.xml";
    int headless = 0;
    int zero_control = 0;
    int zero_wheels = 0;
    int invert_right_joints = 0;
    int freeze_init = 0;
    int start_mode = CHASSIS_STAND_UP;
    double sim_time = 100.0;
    const char *init_key_name = "pos_debug_hang";
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
        else if (strcmp(argv[i], "--time") == 0 && i + 1 < argc)
        {
            sim_time = atof(argv[++i]);
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
    set_initial_stand_pose(m, d, &map, freeze_init, init_key_name);
    SimController_Init();
    SimController_SetMode(start_mode);

    int result = headless ? run_headless(m, d, &map, sim_time, zero_control, zero_wheels, invert_right_joints, freeze_init) : run_viewer(m, d, &map, sim_time, zero_control, zero_wheels, invert_right_joints, freeze_init);

    mj_deleteData(d);
    mj_deleteModel(m);
    return result;
}
