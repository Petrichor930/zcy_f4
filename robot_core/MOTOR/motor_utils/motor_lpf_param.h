
#ifndef _MOTOR_LPF_PARAM_H_
#define _MOTOR_LPF_PARAM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "drv_conf.h"
#include "motor_math.h"

extern const M_LPF_t RF_wheel_lpf_val;
extern const M_LPF_t RB_wheel_lpf_val;
extern const M_LPF_t LF_wheel_lpf_val;
extern const M_LPF_t LB_wheel_lpf_val;

extern const M_LPF_t joint_lpf_val;


#ifdef __cplusplus
}
#endif
#endif // !_MOTOR_LPF_PARAM_H_
