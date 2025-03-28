#ifndef __UTIL_H
#define __UTIL_H
#include "math.h"
#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

//gcc的math有pi，但是keil5没有
#ifndef M_PI
#define M_PI 3.1415926536897932384626f
#endif

// You don't need to worry about the size of the input, just use it
#define LIMIT_MIN_MAX(x, M1, M2)                                              \
    (x) = (M1 < M2) ? (((x) <= (M1)) ? (M1) : (((x) >= (M2)) ? (M2) : (x))) : \
                      (((x) <= (M2)) ? (M2) : (((x) >= (M1)) ? (M1) : (x)))

// You don't need to worry about the size of the input, just use it
#define IS_WITHIN_RANGE(x, M1, M2)                      \
    ({                                                  \
        typeof(x) _x = (x);                             \
        typeof(M1) _m1 = (M1);                          \
        typeof(M2) _m2 = (M2);                          \
        typeof(_m1) _min_val = (_m1 < _m2) ? _m1 : _m2; \
        typeof(_m2) _max_val = (_m1 < _m2) ? _m2 : _m1; \
        (_x >= _min_val && _x <= _max_val);             \
    })

// REDUCE_PRECISION
#define REDUCE_PRECISION(input, precision) ((fabs(input) <= precision) ? 0 : input)

// 获取是否存在加速度
#define GET_ACCELERATE(current, speed)     ((current < -3000.0 && fabs(speed) < 60.0) ? 1 : 0)

static inline float reducePrecision(float input, float precision)
{
    return (fabsf(input) <= precision) ? 0.0f : input;
}

static inline double reduce_precision(double input, double precision)
{
    if (fabs(input) <= precision) {
        return 0.0;
    } else {
        return input;
    }
}


//+-----------------------------------------------------------------------------+
//| 限幅函数                                                                    |
//+-----------------------------------------------------------------------------+
#define SATURATE(_IN, _MIN, _MAX) \
    {                             \
        if (_IN < _MIN)           \
            _IN = _MIN;           \
        else if (_IN > _MAX)      \
            _IN = _MAX;           \
    }

// When compiling with LIMIT_MIN_MAX but it does not pass, please use the following function
static inline float limitMinMax(float x, float a, float b)
{
    if (a <= b) {
        if (x <= a) {
            return a;
        } else if (x >= b) {
            return b;
        } else {
            return x;
        }
    } else {
        if (x <= b) {
            return b;
        } else if (x >= a) {
            return a;
        } else {
            return x;
        }
    }
}

// 定义一个内联函数用于优化程序性能
static inline float CLAMP(float input, float max)
{
    if (input > max)
        return (max);
    else if (input < -max)
        return -(max);
    else
        return input;
}
static inline int GET_SIGN(float input)
{
    if (input >= 0)
        return 1;
    else
        return -1;
}

// gravity compensation
static inline int Gra_Com(float input)
{
    if (input >= 0)
        return 1;
    else
        return 0;
}

/**
 *  @brief	基于angle_range的获取有方向的劣弧
 *  @attention   注意ang1和ang2需要共量程!
 */
#define get_minor_arc(ang1, ang2, angle_range) \
    (fmodf(((ang1) - (ang2) + (angle_range) * 1.5f), (angle_range)) - (angle_range) / 2.f)
/**
 *  @brief  基于angle_range的获取有方向的优弧
 *  @attention  注意ang1和ang2需要共量程!
 */
#define get_major_arc(ang1, ang2, angle_range) \
    (2 * angle_range - get_minor_arc((ang1), (ang2), (angle_range)))


//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

// static float invSqrt(float x) {
//	float halfx = 0.5f * x;
//	float y = x;
//	long i = *(long*)&y;
//	i = 0x5F375A86 - (i>>1);
//	y = *(float*)&i;
//	y = y * (1.5f - (halfx * y * y));
//	return y;
// }
static inline float fast_inv_sqrt(float number)
{
    const float x2 = number * 0.5F;
    const float threehalfs = 1.5F;

    union {
        float f;
        unsigned int i;
    } conv = { .f = number };
    conv.i = 0x5F375A86 - (conv.i >> 1);
    conv.f *= (threehalfs - (x2 * conv.f * conv.f));
    conv.f *= (threehalfs - (x2 * conv.f * conv.f));
    conv.f *= (threehalfs - (x2 * conv.f * conv.f));
    return conv.f;
}


#define T_ACC_CNT 100
static inline float s_curve(float v_max, float cnt)
{
    float cntcnt;
    if (cnt < 0.0f) {
        cnt = -cnt;
        v_max = -v_max;
    }
    cntcnt = cnt / ((float)T_ACC_CNT);
    if (cnt < T_ACC_CNT / 2.0f) {
        return 2.0f * v_max * (cntcnt * cntcnt);
    } else if (cnt < T_ACC_CNT) {
        cntcnt = cntcnt - 1.0f;
        return v_max * (1.0f - 2.0f * (cntcnt * cntcnt));
    } else
        return v_max;
}

/**
 * @description:  Full range data mapping , inverting the curve
 * @param {float} in
 * @param {uint32_t} max_acc    max acclerate
 * @param {float} in_max must be a positive number
 * @param {float} in_min must be a negative number
 * @return {*}
 */
static inline float full_s_curve(float in, unsigned int acc_tim, float in_min, float in_max)
{
    float cntcnt;
    // if (in < 0.0f)
    // {
    // 	in = -in;
    // 	in_max = -in_max;
    // }
    cntcnt = in / ((float)acc_tim);
    if (in >= 0) {
        if (in < acc_tim / 2.0f) {
            return 2.0f * in_max * (cntcnt * cntcnt);
        } else if (in < acc_tim) {
            cntcnt = cntcnt - 1.0f;
            return in_max * (1.0f - 2.0f * (cntcnt * cntcnt));
        } else
            return in_max;
    } else {
        if (fabs(in) < acc_tim / 2.0f) {
            return 2.0f * in_min * (cntcnt * cntcnt);
        } else if (fabs(in) < acc_tim) {
            cntcnt = cntcnt - 1.0f;
            return in_min * (1.0f - 2.0f * (cntcnt * cntcnt));
        } else
            return in_min;
    }
}

#define ACC_FILTER_MAX_CH 3
static inline float get_acc(float current, int ch)
{
    static float last_val[ACC_FILTER_MAX_CH][3];
    float result;
    if (ch >= ACC_FILTER_MAX_CH)
        return 0.0f;
    // refer to the Backward differentiation formula
    result = (11 * current - 18 * last_val[ch][2] + 9 * last_val[ch][1] - 2 * last_val[ch][0]) /
             6.0f;
    last_val[ch][0] = last_val[ch][1];
    last_val[ch][1] = last_val[ch][2];
    last_val[ch][2] = current;
    return result;
}

#define SMOOTH_CH 2
static inline float smooth_acc(float cur, float ref, unsigned int time, unsigned int ch)
{
    static unsigned int temp[SMOOTH_CH] = { 0 };
    temp[ch]++;
    float tmp = ref - cur;
    float step = tmp / time;
    if (temp[ch] >= time) {
        temp[ch] = time;
    }
    if (temp[ch] <= time * 0.5f)
        return step * step * ref * 2.f;
    else {
        step = step - 1.f;
        return ref - (1.0f - (step * step * ref * 2.f));
    }
}

#define CH_NUM   4
#define YAW_CH   0
#define PITCH_CH 1
static inline float s_curve_acc(float k_now, float k_ref, float acc_time, short ch)
{
    static int s_time[CH_NUM];
    static float temp_t[CH_NUM];
    static float t_ratio;
    static float delta_k;
    static short s_flag[CH_NUM];
    if (temp_t[ch] != k_now) {
        s_flag[ch] = 0;
        temp_t[ch] = k_now;
        s_time[ch] = 0;
    }
    if (s_flag[ch] == 0) {
        s_time[ch]++;

        t_ratio = s_time[ch] / acc_time;
        delta_k = k_ref - k_now;

        if (s_time[ch] <= acc_time / 2) {
            return delta_k * (2.0f * t_ratio * t_ratio) + k_now;
        } else if (s_time[ch] < acc_time) {
            t_ratio = t_ratio - 1.0f;
            return delta_k * (1.0f - 2.0f * t_ratio * t_ratio) + k_now;
        } else {
            s_flag[ch] = 1;
            s_time[ch] = 0;
            return k_ref;
        }
    } else if (s_flag[ch] == 1) {
        return k_ref;
    }
    return k_ref;
}

#ifdef __cplusplus
}
#endif
#endif
