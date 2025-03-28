#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define RPM_LOOP 1
#define ANG_LOOP 0

#include "util.h"

/**
 * @struct pid_struct_t
 * @brief struct of pid parameters
 */
typedef struct _pid_struct_t {
    float kp;         ///< Kp
    float ki;         ///< Ki
    float kd;         ///< Kd
    float i_max;      ///< limit maximum of absolute value of integral
    float out_max;    ///< limit maximum of absolute value of output
    float k_deadband; ///< set the deadband

    int deadband_zero_output;
    float err[2]; ///< error and last error

    float p_out;
    float i_out;
    float d_out;
} pid_struct_t;

typedef struct {
    float T;
    float bias;
    float last_in;
} feedforwar_struct_t;

void pid_init(pid_struct_t *pid, float kp, float ki, float kd, float i_max, float out_max,
              float deadband);

void pid_reset(pid_struct_t *pid);

float pid_calc(pid_struct_t *pid, float ref, float cur);
float pid_calc_deadband(pid_struct_t *pid, float ref, float cur);
float pid_dual_loop(pid_struct_t pid[2], float err, float err_out);

float forward_feed(float in, feedforwar_struct_t *FF);

#ifdef __cplusplus
}
#endif