#ifndef JUMP_TELEMETRY_H
#define JUMP_TELEMETRY_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct JumpTelemetry JumpTelemetry;

JumpTelemetry *JumpTelemetry_Create(const char *output_prefix,
                                    double support_takeoff_threshold_n);
void JumpTelemetry_Record(JumpTelemetry *telemetry,
                          double time_s,
                          double base_z_m,
                          double wheel_clearance_m,
                          const double wheel_clearance_lr_m[2],
                          double base_accel_z_mps2,
                          const double base_rpy_rad[3],
                          const double vmc_support_force_raw_n[2],
                          const double vmc_support_force_filtered_n[2],
                          const int vmc_airborne[2],
                          const double contact_normal_force_n[2],
                          int contact_airborne,
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
