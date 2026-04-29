#ifndef SIM_ADAPTER_H
#define SIM_ADAPTER_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float joint_pos[4];       // controller motor coordinates, rad
    float joint_vel[4];       // rad/s
    float joint_torque_fdb[4]; // Nm

    float wheel_vel[2];       // rad/s

    float roll;               // rad
    float pitch;              // rad
    float yaw;                // rad, continuous if possible
    float gyro[3];            // rad/s, x/y/z
    float accel_body[3];      // m/s^2, optional
    float accel_world[3];     // m/s^2, optional

    float body_x;             // m
    float body_y;             // m
    float body_z;             // m
    float body_v;             // m/s
} SimControllerState;

typedef struct
{
    float joint_torque[4];    // Nm, same order as DM_8009_Motor[0..3]
    float wheel_torque[2];    // Nm, same order as LK_9025_Motor[0..1]
} SimControllerOutput;

void SimController_Init(void);
void SimController_SetState(const SimControllerState *state);
void SimController_SetCommand(float v_set, float x_set, float leg_set, float roll_set, float yaw_set);
void SimController_Step(float dt);
void SimController_GetOutput(SimControllerOutput *output);

#ifdef __cplusplus
}
#endif

#endif
