#include "leg_debug.h"

#include <stdio.h>

void leg_debug_log_state(const mjModel *m, const mjData *d, const LegDebugModelMap *map, const LegDebugState *state)
{
    (void)m;
    (void)map;

    printf("t=%7.3f | mode=%s | ncon=%d | input=%s | ik_L0_cmd L=% .3f R=% .3f ik_phi0_cmd L=% .3f R=% .3f | "
           "target_L0_vmc L=% .3f R=% .3f target_phi0_vmc L=% .3f R=% .3f | "
           "L0_vmc L=% .3f R=% .3f | L0_world L=% .3f R=% .3f | phi0_vmc L=% .3f R=% .3f | "
           "q[%s %s %s %s]=[% .3f % .3f % .3f % .3f] | target=[% .3f % .3f % .3f % .3f] | "
           "q_err=[% .3f % .3f % .3f % .3f] | "
           "pd=[% .2f % .2f % .2f % .2f] ff=[% .2f % .2f % .2f % .2f] tau=[% .2f % .2f % .2f % .2f]\n",
           d->time,
           leg_debug_control_mode_name(state->control_mode),
           d->ncon,
           state->input_command,
           state->left_l0_cmd,
           state->right_l0_cmd,
           state->left_phi0_cmd,
           state->right_phi0_cmd,
           state->target_left_l0_vmc,
           state->target_right_l0_vmc,
           state->target_left_phi0_vmc,
           state->target_right_phi0_vmc,
           state->measured_left_l0_vmc,
           state->measured_right_l0_vmc,
           state->left_world.l0_world,
           state->right_world.l0_world,
           state->measured_left_phi0_vmc,
           state->measured_right_phi0_vmc,
           kLegDebugJointShortName[0],
           kLegDebugJointShortName[1],
           kLegDebugJointShortName[2],
           kLegDebugJointShortName[3],
           state->q[0],
           state->q[1],
           state->q[2],
           state->q[3],
           state->target_q[0],
           state->target_q[1],
           state->target_q[2],
           state->target_q[3],
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
    fflush(stdout);
}
