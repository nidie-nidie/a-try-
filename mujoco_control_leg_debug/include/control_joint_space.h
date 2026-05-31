#ifndef MUJOCO_CONTROL_LEG_DEBUG_CONTROL_JOINT_SPACE_H
#define MUJOCO_CONTROL_LEG_DEBUG_CONTROL_JOINT_SPACE_H

#include "leg_debug.h"

void leg_debug_joint_space_reset_home(const LegDebugModelMap *map, const mjData *d, LegDebugState *state);
void leg_debug_joint_space_apply_key(LegDebugState *state, int key);

#endif
