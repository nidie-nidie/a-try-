#include "control_task_space.h"

#include <GLFW/glfw3.h>
#include <math.h>

#include "model_helpers.h"

void leg_debug_task_space_reset_home(const LegDebugModelMap *map, const mjData *d, LegDebugState *state)
{
    state->left_l0_cmd = state->measured_left_l0_vmc;
    state->right_l0_cmd = state->measured_right_l0_vmc;
    state->left_phi0_cmd = state->measured_left_phi0_vmc;
    state->right_phi0_cmd = state->measured_right_phi0_vmc;
    state->min_leg_length = fmin(state->min_leg_length, fmin(state->left_l0_cmd, state->right_l0_cmd));

    state->home_left_l0_cmd = state->left_l0_cmd;
    state->home_right_l0_cmd = state->right_l0_cmd;
    state->home_left_phi0_cmd = state->left_phi0_cmd;
    state->home_right_phi0_cmd = state->right_phi0_cmd;

    for (int i = 0; i < LEG_DEBUG_LEG_JOINT_COUNT; ++i)
    {
        state->target_q[i] = d->qpos[map->joint[i].qpos];
    }
}

void leg_debug_task_space_apply_key(LegDebugState *state, int key)
{
    const double length_step = state->keyboard_length_rate * kLegDebugKeyboardDt;
    const double phi0_step = state->keyboard_phi0_rate * kLegDebugKeyboardDt;

    leg_debug_clear_input_command(state);

    if (key == GLFW_KEY_R)
    {
        state->left_l0_cmd = state->home_left_l0_cmd;
        state->right_l0_cmd = state->home_right_l0_cmd;
        state->left_phi0_cmd = state->home_left_phi0_cmd;
        state->right_phi0_cmd = state->home_right_phi0_cmd;
        state->control_enabled = 1;
        leg_debug_append_input_command(state, "R reset task");
        return;
    }
    if (key == GLFW_KEY_UP)
    {
        state->left_l0_cmd += length_step;
        state->right_l0_cmd += length_step;
        state->control_enabled = 1;
        leg_debug_append_input_command(state, "UP both L0 +");
    }
    if (key == GLFW_KEY_DOWN)
    {
        state->left_l0_cmd -= length_step;
        state->right_l0_cmd -= length_step;
        state->control_enabled = 1;
        leg_debug_append_input_command(state, "DOWN both L0 -");
    }
    if (key == GLFW_KEY_U)
    {
        state->left_l0_cmd += length_step;
        state->control_enabled = 1;
        leg_debug_append_input_command(state, "U left L0 +");
    }
    if (key == GLFW_KEY_J)
    {
        state->left_l0_cmd -= length_step;
        state->control_enabled = 1;
        leg_debug_append_input_command(state, "J left L0 -");
    }
    if (key == GLFW_KEY_O)
    {
        state->right_l0_cmd += length_step;
        state->control_enabled = 1;
        leg_debug_append_input_command(state, "O right L0 +");
    }
    if (key == GLFW_KEY_L)
    {
        state->right_l0_cmd -= length_step;
        state->control_enabled = 1;
        leg_debug_append_input_command(state, "L right L0 -");
    }
    if (key == GLFW_KEY_A)
    {
        state->left_phi0_cmd += phi0_step;
        state->right_phi0_cmd += phi0_step;
        state->control_enabled = 1;
        leg_debug_append_input_command(state, "A both phi0 +");
    }
    if (key == GLFW_KEY_D)
    {
        state->left_phi0_cmd -= phi0_step;
        state->right_phi0_cmd -= phi0_step;
        state->control_enabled = 1;
        leg_debug_append_input_command(state, "D both phi0 -");
    }
    if (key == GLFW_KEY_Q)
    {
        state->left_phi0_cmd += phi0_step;
        state->control_enabled = 1;
        leg_debug_append_input_command(state, "Q left phi0 +");
    }
    if (key == GLFW_KEY_E)
    {
        state->left_phi0_cmd -= phi0_step;
        state->control_enabled = 1;
        leg_debug_append_input_command(state, "E left phi0 -");
    }
    if (key == GLFW_KEY_Z)
    {
        state->right_phi0_cmd += phi0_step;
        state->control_enabled = 1;
        leg_debug_append_input_command(state, "Z right phi0 +");
    }
    if (key == GLFW_KEY_C)
    {
        state->right_phi0_cmd -= phi0_step;
        state->control_enabled = 1;
        leg_debug_append_input_command(state, "C right phi0 -");
    }

    state->left_l0_cmd = leg_debug_clamp(state->left_l0_cmd, state->min_leg_length, state->max_leg_length);
    state->right_l0_cmd = leg_debug_clamp(state->right_l0_cmd, state->min_leg_length, state->max_leg_length);
}

int leg_debug_task_space_solve_targets(const mjData *d, const LegDebugModelMap *map, const LegDebugState *previous, LegDebugState *state)
{
    float left_phi1_phi4[2];
    float right_phi1_phi4[2];
    double raw_target_q[LEG_DEBUG_LEG_JOINT_COUNT];

    CalcPhi1AndPhi4((float)state->left_phi0_cmd, (float)state->left_l0_cmd, left_phi1_phi4);
    CalcPhi1AndPhi4((float)state->right_phi0_cmd, (float)state->right_l0_cmd, right_phi1_phi4);

    if (!isfinite(left_phi1_phi4[0]) || !isfinite(left_phi1_phi4[1]) ||
        !isfinite(right_phi1_phi4[0]) || !isfinite(right_phi1_phi4[1]))
    {
        return 0;
    }

    raw_target_q[LEG_DEBUG_JOINT_LEFT_FRONT] =
        leg_debug_phi_to_joint_qpos(left_phi1_phi4[1],
                                    LEG_DEBUG_PARAM_J0_ANGLE_OFFSET,
                                    LEG_DEBUG_PARAM_J0_DIRECTION);
    raw_target_q[LEG_DEBUG_JOINT_LEFT_REAR] =
        leg_debug_phi_to_joint_qpos(left_phi1_phi4[0],
                                    LEG_DEBUG_PARAM_J1_ANGLE_OFFSET,
                                    LEG_DEBUG_PARAM_J1_DIRECTION);
    raw_target_q[LEG_DEBUG_JOINT_RIGHT_REAR] =
        leg_debug_phi_to_joint_qpos(right_phi1_phi4[0],
                                    LEG_DEBUG_PARAM_J2_ANGLE_OFFSET,
                                    LEG_DEBUG_PARAM_J2_DIRECTION);
    raw_target_q[LEG_DEBUG_JOINT_RIGHT_FRONT] =
        leg_debug_phi_to_joint_qpos(right_phi1_phi4[1],
                                    LEG_DEBUG_PARAM_J3_ANGLE_OFFSET,
                                    LEG_DEBUG_PARAM_J3_DIRECTION);

    for (int i = 0; i < LEG_DEBUG_LEG_JOINT_COUNT; ++i)
    {
        const double reference = previous->control_enabled ? previous->target_q[i] : d->qpos[map->joint[i].qpos];
        state->target_q[i] = leg_debug_nearest_equivalent_angle(reference, raw_target_q[i]);
    }

    return 1;
}
