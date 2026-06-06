#include "model_helpers.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "User_Lib.h"

const char *const kLegDebugSlotName[LEG_DEBUG_ACTUATOR_COUNT] = {
    "left front",
    "left rear",
    "right rear",
    "right front",
    "left wheel",
    "right wheel",
};

const char *const kLegDebugJointShortName[LEG_DEBUG_LEG_JOINT_COUNT] = {
    "LF",
    "LR",
    "RR",
    "RF",
};

static int find_required_id(const mjModel *m, int type, const char *name)
{
    const int id = mj_name2id(m, type, name);
    if (id < 0)
    {
        fprintf(stderr, "Missing MuJoCo object: %s\n", name);
    }
    return id;
}

static int find_optional_id(const mjModel *m, int type, const char *name)
{
    return mj_name2id(m, type, name);
}

static int set_joint_ref(const mjModel *m, int index, const char *name, LegDebugModelMap *map)
{
    const int id = find_required_id(m, mjOBJ_JOINT, name);
    if (id < 0)
    {
        return 0;
    }

    map->joint_id[index] = id;
    map->joint[index].qpos = m->jnt_qposadr[id];
    map->joint[index].qvel = m->jnt_dofadr[id];
    return 1;
}

static int set_wheel_ref(const mjModel *m, int index, const char *name, LegDebugModelMap *map)
{
    const int id = find_required_id(m, mjOBJ_JOINT, name);
    if (id < 0)
    {
        return 0;
    }

    map->wheel_joint_id[index] = id;
    map->wheel[index].qpos = m->jnt_qposadr[id];
    map->wheel[index].qvel = m->jnt_dofadr[id];
    return 1;
}

double leg_debug_clamp(double value, double min_value, double max_value)
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

double leg_debug_wrap_pi(double value)
{
    while (value > kLegDebugPi)
    {
        value -= 2.0 * kLegDebugPi;
    }
    while (value < -kLegDebugPi)
    {
        value += 2.0 * kLegDebugPi;
    }
    return value;
}

double leg_debug_nearest_equivalent_angle(double reference, double candidate)
{
    const double period = 2.0 * kLegDebugPi;
    const double turns = round((reference - candidate) / period);
    return candidate + turns * period;
}

void leg_debug_clear_input_command(LegDebugState *state)
{
    snprintf(state->input_command, sizeof(state->input_command), "none");
}

void leg_debug_append_input_command(LegDebugState *state, const char *command)
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

const char *leg_debug_control_mode_name(LegDebugControlMode mode)
{
    return mode == LEG_DEBUG_CONTROL_JOINT_SPACE ? "joint-space" : "task-space";
}

void leg_debug_initialize_vmc_runtime(LegDebugState *state)
{
    memset(&state->left_vmc, 0, sizeof(state->left_vmc));
    memset(&state->right_vmc, 0, sizeof(state->right_vmc));
    VMC_init(&state->left_vmc);
    VMC_init(&state->right_vmc);
}

int leg_debug_build_model_map(const mjModel *m, LegDebugModelMap *map)
{
    memset(map, 0, sizeof(*map));
    for (int i = 0; i < LEG_DEBUG_LEG_JOINT_COUNT; ++i)
    {
        map->joint_id[i] = -1;
    }
    for (int i = 0; i < LEG_DEBUG_WHEEL_COUNT; ++i)
    {
        map->wheel_joint_id[i] = -1;
    }
    for (int i = 0; i < LEG_DEBUG_ACTUATOR_COUNT; ++i)
    {
        map->actuator[i] = -1;
    }
    map->base_body = -1;
    map->base_freejoint = -1;

    if (find_optional_id(m, mjOBJ_BODY, "base") >= 0)
    {
        map->base_body = find_required_id(m, mjOBJ_BODY, "base");
        map->base_freejoint = find_required_id(m, mjOBJ_JOINT, "base_free");

        return map->base_body >= 0 &&
               map->base_freejoint >= 0 &&
               set_joint_ref(m, LEG_DEBUG_JOINT_LEFT_FRONT, "jIJ", map) &&
               set_joint_ref(m, LEG_DEBUG_JOINT_LEFT_REAR, "jIO", map) &&
               set_joint_ref(m, LEG_DEBUG_JOINT_RIGHT_REAR, "jAG", map) &&
               set_joint_ref(m, LEG_DEBUG_JOINT_RIGHT_FRONT, "jAB", map) &&
               set_wheel_ref(m, 0, "jwheel_left", map) &&
               set_wheel_ref(m, 1, "jwheel_right", map) &&
               (map->actuator[0] = find_required_id(m, mjOBJ_ACTUATOR, "Left_front_joint_act")) >= 0 &&
               (map->actuator[1] = find_required_id(m, mjOBJ_ACTUATOR, "Left_rear_joint_act")) >= 0 &&
               (map->actuator[2] = find_required_id(m, mjOBJ_ACTUATOR, "Right_rear_joint_act")) >= 0 &&
               (map->actuator[3] = find_required_id(m, mjOBJ_ACTUATOR, "Right_front_joint_act")) >= 0 &&
               (map->actuator[4] = find_required_id(m, mjOBJ_ACTUATOR, "Left_Wheel_act")) >= 0 &&
               (map->actuator[5] = find_required_id(m, mjOBJ_ACTUATOR, "Right_Wheel_act")) >= 0;
    }

    if (find_optional_id(m, mjOBJ_BODY, "base_link") >= 0)
    {
        map->base_body = find_required_id(m, mjOBJ_BODY, "base_link");
        map->base_freejoint = find_required_id(m, mjOBJ_JOINT, "base_freejoint");

        return map->base_body >= 0 &&
               map->base_freejoint >= 0 &&
               set_joint_ref(m, LEG_DEBUG_JOINT_LEFT_FRONT, "left_leg", map) &&
               set_joint_ref(m, LEG_DEBUG_JOINT_LEFT_REAR, "left_small_leg", map) &&
               set_joint_ref(m, LEG_DEBUG_JOINT_RIGHT_REAR, "right_leg", map) &&
               set_joint_ref(m, LEG_DEBUG_JOINT_RIGHT_FRONT, "right_small_leg", map) &&
               set_wheel_ref(m, 0, "left_wheel", map) &&
               set_wheel_ref(m, 1, "right_wheel", map) &&
               (map->actuator[0] = find_required_id(m, mjOBJ_ACTUATOR, "left_leg_motor")) >= 0 &&
               (map->actuator[1] = find_required_id(m, mjOBJ_ACTUATOR, "left_small_leg_motor")) >= 0 &&
               (map->actuator[2] = find_required_id(m, mjOBJ_ACTUATOR, "right_leg_motor")) >= 0 &&
               (map->actuator[3] = find_required_id(m, mjOBJ_ACTUATOR, "right_small_leg_motor")) >= 0 &&
               (map->actuator[4] = find_required_id(m, mjOBJ_ACTUATOR, "left_wheel_motor")) >= 0 &&
               (map->actuator[5] = find_required_id(m, mjOBJ_ACTUATOR, "right_wheel_motor")) >= 0;
    }

    fprintf(stderr, "Unsupported model: expected body 'base' or 'base_link'.\n");
    return 0;
}

void leg_debug_print_model_map(const mjModel *m, const LegDebugModelMap *map)
{
    const LegDebugJointRef joint_ref[LEG_DEBUG_ACTUATOR_COUNT] = {
        map->joint[LEG_DEBUG_JOINT_LEFT_FRONT],
        map->joint[LEG_DEBUG_JOINT_LEFT_REAR],
        map->joint[LEG_DEBUG_JOINT_RIGHT_REAR],
        map->joint[LEG_DEBUG_JOINT_RIGHT_FRONT],
        map->wheel[0],
        map->wheel[1],
    };

    printf("\nLeg debug model mapping:\n");
    for (int i = 0; i < LEG_DEBUG_ACTUATOR_COUNT; ++i)
    {
        const int actuator_id = map->actuator[i];
        const int joint_id = i < LEG_DEBUG_LEG_JOINT_COUNT ? map->joint_id[i] : map->wheel_joint_id[i - LEG_DEBUG_LEG_JOINT_COUNT];
        const char *joint_name = mj_id2name(m, mjOBJ_JOINT, joint_id);
        const char *body_name = mj_id2name(m, mjOBJ_BODY, m->jnt_bodyid[joint_id]);
        const char *actuator_name = mj_id2name(m, mjOBJ_ACTUATOR, actuator_id);
        const int actuator_joint_id = m->actuator_trnid[2 * actuator_id];
        const char *actuator_joint_name = mj_id2name(m, mjOBJ_JOINT, actuator_joint_id);
        const double *axis = &m->jnt_axis[3 * joint_id];

        printf("  %-11s joint=%-14s body=%-12s axis=[% .0f % .0f % .0f] qpos=%2d qvel=%2d | actuator=%-22s -> joint=%s\n",
               kLegDebugSlotName[i],
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

static double closed_chain_site_cost(const mjModel *m, mjData *d)
{
    const char *site_pairs[][2] = {
        {"site_ec_ag_a1", "site_ec_ag_b1"},
        {"site_ec_ag_a2", "site_ec_ag_b2"},
        {"site_cf_gh_a1", "site_cf_gh_b1"},
        {"site_cf_gh_a2", "site_cf_gh_b2"},
        {"site_mk_io_a1", "site_mk_io_b1"},
        {"site_mk_io_a2", "site_mk_io_b2"},
        {"site_kn_op_a1", "site_kn_op_b1"},
        {"site_kn_op_a2", "site_kn_op_b2"},
    };
    double cost = 0.0;

    mj_forward(m, d);
    for (int i = 0; i < 8; ++i)
    {
        const int site1 = find_optional_id(m, mjOBJ_SITE, site_pairs[i][0]);
        const int site2 = find_optional_id(m, mjOBJ_SITE, site_pairs[i][1]);
        if (site1 >= 0 && site2 >= 0)
        {
            const mjtNum *p1 = &d->site_xpos[3 * site1];
            const mjtNum *p2 = &d->site_xpos[3 * site2];
            const double dx = p1[0] - p2[0];
            const double dy = p1[1] - p2[1];
            const double dz = p1[2] - p2[2];
            cost += dx * dx + dy * dy + dz * dz;
        }
    }
    return cost;
}

static int append_passive_qpos(const mjModel *m, const char *joint_name, int qpos_adr[], int count, int max_count)
{
    const int joint_id = find_optional_id(m, mjOBJ_JOINT, joint_name);
    if (joint_id >= 0 && count < max_count)
    {
        qpos_adr[count++] = m->jnt_qposadr[joint_id];
    }
    return count;
}

static void relax_closed_chain_pose(const mjModel *m, mjData *d, const LegDebugModelMap *map)
{
    if (find_optional_id(m, mjOBJ_JOINT, "jBE") < 0 || find_optional_id(m, mjOBJ_JOINT, "jJM") < 0)
    {
        return;
    }

    const int base_qpos = m->jnt_qposadr[map->base_freejoint];
    mjtNum base_pose[LEG_DEBUG_BASE_QPOS_SIZE];
    mjtNum joint_pose[LEG_DEBUG_LEG_JOINT_COUNT];
    int passive_qpos[8];
    int passive_count = 0;

    for (int i = 0; i < LEG_DEBUG_BASE_QPOS_SIZE; ++i)
    {
        base_pose[i] = d->qpos[base_qpos + i];
    }
    for (int i = 0; i < LEG_DEBUG_LEG_JOINT_COUNT; ++i)
    {
        joint_pose[i] = d->qpos[map->joint[i].qpos];
    }

    passive_count = append_passive_qpos(m, "jGH", passive_qpos, passive_count, 8);
    passive_count = append_passive_qpos(m, "jBE", passive_qpos, passive_count, 8);
    passive_count = append_passive_qpos(m, "jEC", passive_qpos, passive_count, 8);
    passive_count = append_passive_qpos(m, "jCF", passive_qpos, passive_count, 8);
    passive_count = append_passive_qpos(m, "jJM", passive_qpos, passive_count, 8);
    passive_count = append_passive_qpos(m, "jMK", passive_qpos, passive_count, 8);
    passive_count = append_passive_qpos(m, "jKN", passive_qpos, passive_count, 8);
    passive_count = append_passive_qpos(m, "jOP", passive_qpos, passive_count, 8);

    double best_cost = closed_chain_site_cost(m, d);
    int sweep_count = 0;
    for (double step = 1.0; step > 1.0e-4 && sweep_count < 160;)
    {
        int improved = 0;
        ++sweep_count;

        for (int var = 0; var < passive_count; ++var)
        {
            const int qadr = passive_qpos[var];
            const double original = d->qpos[qadr];
            double best_value = original;

            for (int sign = -1; sign <= 1; sign += 2)
            {
                d->qpos[qadr] = original + (double)sign * step;
                for (int i = 0; i < LEG_DEBUG_BASE_QPOS_SIZE; ++i)
                {
                    d->qpos[base_qpos + i] = base_pose[i];
                }
                for (int i = 0; i < LEG_DEBUG_LEG_JOINT_COUNT; ++i)
                {
                    d->qpos[map->joint[i].qpos] = joint_pose[i];
                }

                const double cost = closed_chain_site_cost(m, d);
                if (cost < best_cost)
                {
                    best_cost = cost;
                    best_value = d->qpos[qadr];
                    improved = 1;
                }
            }
            d->qpos[qadr] = best_value;
        }

        if (!improved)
        {
            step *= 0.5;
        }
        if (best_cost < 1.0e-10)
        {
            break;
        }
    }

    for (int i = 0; i < LEG_DEBUG_BASE_QPOS_SIZE; ++i)
    {
        d->qpos[base_qpos + i] = base_pose[i];
    }
    for (int i = 0; i < LEG_DEBUG_LEG_JOINT_COUNT; ++i)
    {
        d->qpos[map->joint[i].qpos] = joint_pose[i];
    }
    memset(d->qvel, 0, sizeof(mjtNum) * m->nv);
    memset(d->ctrl, 0, sizeof(mjtNum) * m->nu);
    mj_forward(m, d);
}

int leg_debug_reset_data(const mjModel *m, mjData *d, const LegDebugModelMap *map, const char *key_name)
{
    const int key_id = key_name && key_name[0] != '\0' ? find_optional_id(m, mjOBJ_KEY, key_name) : -1;
    (void)map;
    if (key_name && key_name[0] != '\0' && key_id < 0)
    {
        fprintf(stderr, "Keyframe not found: %s\n", key_name);
        return 0;
    }

    if (key_id >= 0)
    {
        mj_resetDataKeyframe(m, d, key_id);
    }
    else
    {
        mj_resetData(m, d);
    }

    memset(d->qvel, 0, sizeof(mjtNum) * m->nv);
    memset(d->ctrl, 0, sizeof(mjtNum) * m->nu);
    mj_forward(m, d);
    relax_closed_chain_pose(m, d, map);
    return 1;
}

double leg_debug_joint_qpos_to_phi(double joint_qpos, double angle_offset, int direction)
{
    return theta_transform((float)(-joint_qpos), (float)angle_offset, (int8_t)direction, 1);
}

double leg_debug_phi_to_joint_qpos(double phi, double angle_offset, int direction)
{
    return -theta_transform((float)phi, (float)(-angle_offset), (int8_t)direction, 1);
}



// 把当前的目标关节角 再正解成 vmc 的腿长和角度，用来现实和检查 IK 是否能够自洽。
void leg_debug_update_target_vmc(LegDebugState *state)
{
    // 创建两个临时的 vmc 结构，分别代表左腿和右腿的目标状态
    vmc_leg_t left;
    vmc_leg_t right;
    // 清零这两个结构体，确保它们的初始状态是干净的
    memset(&left, 0, sizeof(left));
    memset(&right, 0, sizeof(right));
    // 调用 VMC_init 函数来初始化这两个结构体，设置它们的默认值和参数
    VMC_init(&left);
    VMC_init(&right);
    // 
    left.phi4 = (float)leg_debug_joint_qpos_to_phi(state->target_q[LEG_DEBUG_JOINT_LEFT_FRONT],
                                                    LEG_DEBUG_PARAM_J0_ANGLE_OFFSET,
                                                    LEG_DEBUG_PARAM_J0_DIRECTION);
    left.phi1 = (float)leg_debug_joint_qpos_to_phi(state->target_q[LEG_DEBUG_JOINT_LEFT_REAR],
                                                    LEG_DEBUG_PARAM_J1_ANGLE_OFFSET,
                                                    LEG_DEBUG_PARAM_J1_DIRECTION);
    right.phi1 = (float)leg_debug_joint_qpos_to_phi(state->target_q[LEG_DEBUG_JOINT_RIGHT_REAR],
                                                     LEG_DEBUG_PARAM_J2_ANGLE_OFFSET,
                                                     LEG_DEBUG_PARAM_J2_DIRECTION);
    right.phi4 = (float)leg_debug_joint_qpos_to_phi(state->target_q[LEG_DEBUG_JOINT_RIGHT_FRONT],
                                                     LEG_DEBUG_PARAM_J3_ANGLE_OFFSET,
                                                     LEG_DEBUG_PARAM_J3_DIRECTION);

    VMC_calc_1(&left, 0.0f, 0.0f, 0.001f);
    VMC_calc_1(&right, 0.0f, 0.0f, 0.001f);

    state->target_left_l0_vmc = left.L0;
    state->target_right_l0_vmc = right.L0;
    state->target_left_phi0_vmc = left.phi0;
    state->target_right_phi0_vmc = right.phi0;
}

static double base_pitch_from_qpos(const mjModel *m, const mjData *d, const LegDebugModelMap *map)
{
    const int base_qpos = m->jnt_qposadr[map->base_freejoint];
    const double qw = d->qpos[base_qpos + 3];
    const double qx = d->qpos[base_qpos + 4];
    const double qy = d->qpos[base_qpos + 5];
    const double qz = d->qpos[base_qpos + 6];
    double sinp = 2.0 * (qw * qy - qz * qx);

    sinp = leg_debug_clamp(sinp, -1.0, 1.0);
    return asin(sinp);
}

static void copy3(double dst[3], const mjtNum *src)
{
    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
}

static double dist3(const double a[3], const double b[3])
{
    const double dx = a[0] - b[0];
    const double dy = a[1] - b[1];
    const double dz = a[2] - b[2];
    return sqrt(dx * dx + dy * dy + dz * dz);
}

static double dist3_sq(const double a[3], const double b[3])
{
    const double dx = a[0] - b[0];
    const double dy = a[1] - b[1];
    const double dz = a[2] - b[2];
    return dx * dx + dy * dy + dz * dz;
}

static void compute_leg_world_points(const mjModel *m, const mjData *d, const LegDebugModelMap *map,
                                     int left_leg, const vmc_leg_t *vmc, LegDebugLegWorld *world)
{
    const int hip_a_joint = left_leg ? map->joint_id[LEG_DEBUG_JOINT_LEFT_REAR] : map->joint_id[LEG_DEBUG_JOINT_RIGHT_FRONT];
    const int hip_b_joint = left_leg ? map->joint_id[LEG_DEBUG_JOINT_LEFT_FRONT] : map->joint_id[LEG_DEBUG_JOINT_RIGHT_REAR];
    const int wheel_joint = left_leg ? map->wheel_joint_id[0] : map->wheel_joint_id[1];
    const mjtNum *hip_a = &d->xanchor[3 * hip_a_joint];
    const mjtNum *hip_b = &d->xanchor[3 * hip_b_joint];
    const mjtNum *wheel = &d->xanchor[3 * wheel_joint];

    for (int i = 0; i < 3; ++i)
    {
        world->hip_mid[i] = 0.5 * (hip_a[i] + hip_b[i]);
    }
    copy3(world->wheel_axis, wheel);
    world->l0_world = dist3(world->hip_mid, world->wheel_axis);

    const double *base_xmat = &d->xmat[9 * map->base_body];
    const double x_axis[3] = {base_xmat[0], base_xmat[3], base_xmat[6]};
    const double z_axis[3] = {base_xmat[2], base_xmat[5], base_xmat[8]};
    const double local_x = (double)vmc->L0 * cos((double)vmc->phi0);
    const double local_z = (double)vmc->L0 * sin((double)vmc->phi0);
    double candidate_a[3];
    double candidate_b[3];

    for (int i = 0; i < 3; ++i)
    {
        candidate_a[i] = world->hip_mid[i] + x_axis[i] * local_x + z_axis[i] * local_z;
        candidate_b[i] = world->hip_mid[i] + x_axis[i] * local_x - z_axis[i] * local_z;
    }

    if (!isfinite(local_x) || !isfinite(local_z))
    {
        copy3(world->vmc_c, world->hip_mid);
    }
    else if (dist3_sq(candidate_a, world->wheel_axis) <= dist3_sq(candidate_b, world->wheel_axis))
    {
        copy3(world->vmc_c, candidate_a);
    }
    else
    {
        copy3(world->vmc_c, candidate_b);
    }

    (void)m;
}

void leg_debug_update_measurements(const mjModel *m, const mjData *d, const LegDebugModelMap *map, LegDebugState *state)
{
    vmc_leg_t *left = &state->left_vmc;
    vmc_leg_t *right = &state->right_vmc;
    const double body_pitch = base_pitch_from_qpos(m, d, map);
    const int base_qvel = m->jnt_dofadr[map->base_freejoint];
    const double body_pitch_rate = d->qvel[base_qvel + 4];
    const double dt = m->opt.timestep > 0.0 ? m->opt.timestep : 0.001;

    for (int i = 0; i < LEG_DEBUG_LEG_JOINT_COUNT; ++i)
    {
        state->q[i] = d->qpos[map->joint[i].qpos];
        state->qd[i] = d->qvel[map->joint[i].qvel];
    }

    left->phi4 = (float)leg_debug_joint_qpos_to_phi(state->q[LEG_DEBUG_JOINT_LEFT_FRONT],
                                                     LEG_DEBUG_PARAM_J0_ANGLE_OFFSET,
                                                     LEG_DEBUG_PARAM_J0_DIRECTION);
    left->phi1 = (float)leg_debug_joint_qpos_to_phi(state->q[LEG_DEBUG_JOINT_LEFT_REAR],
                                                     LEG_DEBUG_PARAM_J1_ANGLE_OFFSET,
                                                     LEG_DEBUG_PARAM_J1_DIRECTION);
    right->phi1 = (float)leg_debug_joint_qpos_to_phi(state->q[LEG_DEBUG_JOINT_RIGHT_REAR],
                                                      LEG_DEBUG_PARAM_J2_ANGLE_OFFSET,
                                                      LEG_DEBUG_PARAM_J2_DIRECTION);
    right->phi4 = (float)leg_debug_joint_qpos_to_phi(state->q[LEG_DEBUG_JOINT_RIGHT_FRONT],
                                                      LEG_DEBUG_PARAM_J3_ANGLE_OFFSET,
                                                      LEG_DEBUG_PARAM_J3_DIRECTION);

    VMC_calc_1(left, (float)(-body_pitch), (float)(-body_pitch_rate), (float)dt);
    VMC_calc_1(right, (float)(body_pitch), (float)(body_pitch_rate), (float)dt);

    state->measured_left_l0_vmc = left->L0;
    state->measured_right_l0_vmc = right->L0;
    state->measured_left_phi0_vmc = left->phi0;
    state->measured_right_phi0_vmc = right->phi0;
    state->measured_left_theta = left->theta;
    state->measured_right_theta = right->theta;
    state->measured_left_dl0 = left->d_L0;
    state->measured_right_dl0 = right->d_L0;
    state->measured_left_dphi0 = left->d_phi0;
    state->measured_right_dphi0 = right->d_phi0;

    compute_leg_world_points(m, d, map, 1, left, &state->left_world);
    compute_leg_world_points(m, d, map, 0, right, &state->right_world);
}

void leg_debug_infer_ctrl_signs(const mjModel *m, mjData *d, const LegDebugModelMap *map, LegDebugState *state)
{
    (void)d;

    for (int i = 0; i < LEG_DEBUG_LEG_JOINT_COUNT; ++i)
    {
        const int actuator = map->actuator[i];
        const int actuator_joint = m->actuator_trnid[2 * actuator];
        const double gear = m->actuator_gear[6 * actuator];
        state->joint_ctrl_sign[i] = gear >= 0.0 ? 1.0 : -1.0;
        if (actuator_joint != map->joint_id[i] || fabs(gear) < 1.0e-9)
        {
            state->joint_ctrl_sign[i] = 1.0;
        }
    }
}
