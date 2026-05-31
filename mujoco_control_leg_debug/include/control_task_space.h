#ifndef MUJOCO_CONTROL_LEG_DEBUG_CONTROL_TASK_SPACE_H
#define MUJOCO_CONTROL_LEG_DEBUG_CONTROL_TASK_SPACE_H

#include "leg_debug.h"

void leg_debug_task_space_reset_home(const LegDebugModelMap *map, const mjData *d, LegDebugState *state);
void leg_debug_task_space_apply_key(LegDebugState *state, int key);
int leg_debug_task_space_solve_targets(const mjData *d, const LegDebugModelMap *map, const LegDebugState *previous, LegDebugState *state);

#endif
