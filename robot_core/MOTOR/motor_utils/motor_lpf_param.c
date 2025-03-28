
#include "motor_lpf_param.h"
//fc 截止频率
//fs 采样频率
const M_LPF_t RF_wheel_lpf_val = { .fc = CHASSIS_CTRL_FREQ / 2.56f, .ts = CHASSIS_CTRL_FREQ };
const M_LPF_t LF_wheel_lpf_val = { .fc = CHASSIS_CTRL_FREQ / 2.56f, .ts = CHASSIS_CTRL_FREQ };
const M_LPF_t LB_wheel_lpf_val = { .fc = CHASSIS_CTRL_FREQ / 2.56f, .ts = CHASSIS_CTRL_FREQ };
const M_LPF_t RB_wheel_lpf_val = { .fc = CHASSIS_CTRL_FREQ / 2.56f, .ts = CHASSIS_CTRL_FREQ };

const M_LPF_t joint_lpf_val = { .fc = ARM_CTRL_FREQ / 2.56f, .ts = ARM_CTRL_FREQ };

