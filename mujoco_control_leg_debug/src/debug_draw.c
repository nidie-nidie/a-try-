#include "debug_draw.h"

static void copy_mjt(mjtNum dst[3], const double src[3])
{
    dst[0] = (mjtNum)src[0];
    dst[1] = (mjtNum)src[1];
    dst[2] = (mjtNum)src[2];
}

static void add_line(mjvScene *scene, const double from_d[3], const double to_d[3], const float rgba[4], mjtNum width)
{
    if (scene->ngeom >= scene->maxgeom)
    {
        return;
    }

    mjtNum from[3];
    mjtNum to[3];
    mjtNum size[3] = {width, width, width};
    mjtNum pos[3] = {0.0, 0.0, 0.0};
    mjtNum mat[9] = {1.0, 0.0, 0.0,
                     0.0, 1.0, 0.0,
                     0.0, 0.0, 1.0};

    copy_mjt(from, from_d);
    copy_mjt(to, to_d);
    mjv_initGeom(&scene->geoms[scene->ngeom], mjGEOM_LINE, size, pos, mat, rgba);
    mjv_connector(&scene->geoms[scene->ngeom], mjGEOM_LINE, width, from, to);
    scene->ngeom++;
}

void leg_debug_draw_scene(mjvScene *scene, const LegDebugState *state)
{
    if (!state->show_debug_draw)
    {
        return;
    }

    const float left_world_rgba[4] = {0.10f, 0.80f, 1.00f, 1.0f};
    const float right_world_rgba[4] = {0.20f, 1.00f, 0.45f, 1.0f};
    const float left_vmc_rgba[4] = {1.00f, 0.80f, 0.05f, 1.0f};
    const float right_vmc_rgba[4] = {1.00f, 0.25f, 0.25f, 1.0f};
    const float hip_axis_rgba[4] = {0.95f, 0.95f, 1.00f, 1.0f};
    const float wheel_axis_rgba[4] = {0.65f, 0.45f, 1.00f, 1.0f};

    add_line(scene, state->left_world.hip_mid, state->left_world.wheel_axis, left_world_rgba, 0.006);
    add_line(scene, state->right_world.hip_mid, state->right_world.wheel_axis, right_world_rgba, 0.006);

    add_line(scene, state->left_world.hip_mid, state->left_world.vmc_c, left_vmc_rgba, 0.004);
    add_line(scene, state->right_world.hip_mid, state->right_world.vmc_c, right_vmc_rgba, 0.004);

    add_line(scene, state->left_world.hip_mid, state->right_world.hip_mid, hip_axis_rgba, 0.004);
    add_line(scene, state->left_world.wheel_axis, state->right_world.wheel_axis, wheel_axis_rgba, 0.004);
}
