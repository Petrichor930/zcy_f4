
#ifndef _MOTOR_PID_PARAM_H_
#define _MOTOR_PID_PARAM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "pid.h"

extern pid_struct_t wheel1_pid;
extern pid_struct_t wheel2_pid;
extern pid_struct_t wheel3_pid;
extern pid_struct_t wheel4_pid;

extern pid_struct_t chassis_vx_pid;
extern pid_struct_t chassis_vy_pid;
extern pid_struct_t chassis_wz_pid;
extern pid_struct_t chassis_yaw_pid;
extern pid_struct_t chassis_follow_pid;

extern pid_struct_t joint1_pid;

extern const pid_struct_t joint7_rpm_pid;
extern const pid_struct_t joint7_ang_pid;

#ifdef __cplusplus
}
#endif
#endif // ! _MOTOR_PID_PARAM_H_
