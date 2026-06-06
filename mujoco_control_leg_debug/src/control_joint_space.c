#include "control_joint_space.h"

#include "rm_third_party/glfw.h"
#include <stdio.h>

void leg_debug_joint_space_reset_home(const LegDebugModelMap *map, const mjData *d, LegDebugState *state)
{
    for (int i = 0; i < LEG_DEBUG_LEG_JOINT_COUNT; ++i)
    {
        state->target_q[i] = d->qpos[map->joint[i].qpos];
        state->home_target_q[i] = state->target_q[i];
    }
    state->selected_joint = LEG_DEBUG_JOINT_LEFT_FRONT;
}

void leg_debug_joint_space_apply_key(LegDebugState *state, int key)
{
    const double step = state->keyboard_joint_rate * kLegDebugKeyboardDt;
    char command[64];

    leg_debug_clear_input_command(state);

    if (key >= GLFW_KEY_1 && key <= GLFW_KEY_4)
    {
        state->selected_joint = key - GLFW_KEY_1;
        snprintf(command, sizeof(command), "select %s", kLegDebugJointShortName[state->selected_joint]);
        leg_debug_append_input_command(state, command);
        return;
    }

    if (key == GLFW_KEY_LEFT)
    {
        state->selected_joint = (state->selected_joint + LEG_DEBUG_LEG_JOINT_COUNT - 1) % LEG_DEBUG_LEG_JOINT_COUNT;
        snprintf(command, sizeof(command), "select %s", kLegDebugJointShortName[state->selected_joint]);
        leg_debug_append_input_command(state, command);
        return;
    }
    if (key == GLFW_KEY_RIGHT)
    {
        state->selected_joint = (state->selected_joint + 1) % LEG_DEBUG_LEG_JOINT_COUNT;
        snprintf(command, sizeof(command), "select %s", kLegDebugJointShortName[state->selected_joint]);
        leg_debug_append_input_command(state, command);
        return;
    }

    if (key == GLFW_KEY_R)
    {
        for (int i = 0; i < LEG_DEBUG_LEG_JOINT_COUNT; ++i)
        {
            state->target_q[i] = state->home_target_q[i];
        }
        state->control_enabled = 1;
        leg_debug_append_input_command(state, "R reset joints");
        return;
    }

    if (key == GLFW_KEY_LEFT_BRACKET || key == GLFW_KEY_DOWN)
    {
        state->target_q[state->selected_joint] -= step;
        state->control_enabled = 1;
        snprintf(command, sizeof(command), "%s q -", kLegDebugJointShortName[state->selected_joint]);
        leg_debug_append_input_command(state, command);
    }
    if (key == GLFW_KEY_RIGHT_BRACKET || key == GLFW_KEY_UP)
    {
        state->target_q[state->selected_joint] += step;
        state->control_enabled = 1;
        snprintf(command, sizeof(command), "%s q +", kLegDebugJointShortName[state->selected_joint]);
        leg_debug_append_input_command(state, command);
    }
    if (key == GLFW_KEY_COMMA)
    {
        for (int i = 0; i < LEG_DEBUG_LEG_JOINT_COUNT; ++i)
        {
            state->target_q[i] -= step;
        }
        state->control_enabled = 1;
        leg_debug_append_input_command(state, ", all q -");
    }
    if (key == GLFW_KEY_PERIOD)
    {
        for (int i = 0; i < LEG_DEBUG_LEG_JOINT_COUNT; ++i)
        {
            state->target_q[i] += step;
        }
        state->control_enabled = 1;
        leg_debug_append_input_command(state, ". all q +");
    }
}
