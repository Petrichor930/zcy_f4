#ifndef _FILTER_H
#define _FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "util.h"
//IIR filter channel number (2 or 3 refer to the order)

#define MAX_FILTER_CH_2             4
#define MAX_FILTER_CH_3             4
#define MAX_FILTER_CH_5             3
#define MAX_ONE_EURO_FILTER_CH      3

#define CHASSIS_FILTER3_CH1         0
#define CHASSIS_FILTER3_CH2         1
#define CHASSIS_FILTER3_CH3         2

#define GIMBAL_PITCH_FILTER2_RPM_CH 0
#define GIMBAL_YAW_FILTER2_RPM_CH   1
#define SHOOT_LEFT_FILTER2_RPM_CH   2
#define SHOOT_RIGHT_FILTER2_RPM_CH  3

#define SMAPLE_LS_TASKS_TICK        (5) // 2ms

typedef struct {
    float orig_val; // orignal value,原始值
    float fltr_val; // filter value,滤波值
    float ts;       // 采用间隔时间
    float fc;       // 截止频率
    float a;        // 比例系数,init会自动更新
} LPF_t;

typedef struct {
    float a[3]; // 系数
    float b[3]; // 系数
    float x[3]; // 输入值
    float y[3]; // 输出值
} ButterworthLPF_t;

//+-----------------------------------------------------------------------------+
//| 低通滤波                                                                    |
//+-----------------------------------------------------------------------------+
void LPF_init(LPF_t *filter, float ts, float fc);
float LPF_update(LPF_t *filter, float new_value);

//+-----------------------------------------------------------------------------+
//| 巴特沃斯低通滤波                                                            |
//+-----------------------------------------------------------------------------+
void ButterworthLPF_init(ButterworthLPF_t *filter, float cutoff_freq, float sample_rate);
float ButterworthLPF_update(ButterworthLPF_t *filter, float new_value);

float mean_filter_2(float x_i);
float iir_filter_2(float x, unsigned int ch);
float iir_filter_3(float x, unsigned int ch);

float fir_filter_5(float x, unsigned int ch);
float one_euro_filter(float x, float dt, unsigned int ch);

#ifdef __cplusplus
}
#endif
#endif
