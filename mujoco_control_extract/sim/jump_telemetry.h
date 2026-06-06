#ifndef JUMP_TELEMETRY_H
#define JUMP_TELEMETRY_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct JumpTelemetry JumpTelemetry;

JumpTelemetry *JumpTelemetry_Create(const char *output_prefix);
void JumpTelemetry_Record(JumpTelemetry *telemetry,
                          double time_s,
                          double base_z_m,
                          double wheel_clearance_m,
                          const double command_torque_nm[6],
                          const double applied_torque_nm[6],
                          int jump_active,
                          int jump_phase,
                          int airborne);
int JumpTelemetry_Write(JumpTelemetry *telemetry);
void JumpTelemetry_Destroy(JumpTelemetry *telemetry);

#ifdef __cplusplus
}
#endif

#endif
