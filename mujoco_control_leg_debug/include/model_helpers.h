#ifndef MUJOCO_CONTROL_LEG_DEBUG_MODEL_HELPERS_H
#define MUJOCO_CONTROL_LEG_DEBUG_MODEL_HELPERS_H

#include "leg_debug.h"

double leg_debug_clamp(double value, double min_value, double max_value);
double leg_debug_wrap_pi(double value);
double leg_debug_nearest_equivalent_angle(double reference, double candidate);

void leg_debug_initialize_vmc_runtime(LegDebugState *state);
int leg_debug_build_model_map(const mjModel *m, LegDebugModelMap *map);
void leg_debug_print_model_map(const mjModel *m, const LegDebugModelMap *map);
int leg_debug_reset_data(const mjModel *m, mjData *d, const LegDebugModelMap *map, const char *key_name);
void leg_debug_update_measurements(const mjModel *m, const mjData *d, const LegDebugModelMap *map, LegDebugState *state);
void leg_debug_update_target_vmc(LegDebugState *state);
void leg_debug_infer_ctrl_signs(const mjModel *m, mjData *d, const LegDebugModelMap *map, LegDebugState *state);

double leg_debug_joint_qpos_to_phi(double joint_qpos, double angle_offset, int direction);
double leg_debug_phi_to_joint_qpos(double phi, double angle_offset, int direction);

#endif
