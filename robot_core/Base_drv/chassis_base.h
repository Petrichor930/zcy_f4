#ifndef _CHASSIS_BASE_H
#define _CHASSIS_BASE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "drv_conf.h"
#include HAL_INCLUDE
#include "motor_headers.h"

/**
 * @struct chassis_state_t
 * @brief speed of chassis motion, v_x,v_y in m/s,w_z in rpm
 */
/*整车底盘状态结构体：X轴速度，Y轴速度，Z轴速度*/
typedef struct {
    float v_x; ///< velocity of x axis(by default it is the forward direction) in m/s
    float v_y; ///< velocity of y axis(by default it is the right direction) in m/s
    float w_z; ///< angular velocity of z axis(by default it is CW direction) in rad/s
} chassis_state_t;

/**
 * @struct chassis_wheels_dir_t
 * @brief direction of 4 wheels
 */
typedef struct {
    int16_t DIR_RF; ///< motor of the right front
    int16_t DIR_LF; ///< motor of the left front
    int16_t DIR_LB; ///< motor of the left back
    int16_t DIR_RB; ///< motor of the right back
} chassis_wheels_dir_t;

/**
 * @struct chassis_wheels_state_t
 * @brief speed of 4 wheels in rpm or 4 wheels outputs
 */
/*电机四轮�?速结构体*/
typedef struct {
    float M_RF; ///< motor of the right front
    float M_LF; ///< motor of the left front
    float M_LB; ///< motor of the left back
    float M_RB; ///< motor of the right back
} chassis_wheels_state_t;

/*麦克纳姆轮的尺寸*/
// reference: https://www.robomaster.com/zh-CN/products/components/detail/135
// diameter of mecanum wheel in meter
#define D_WHEEL      (0.1525f)
// circumference of wheel in meter
#define C_WHEEL      (PI * D_WHEEL)

/*底盘机械参数定义，
两者分别为左或右前轮到旋转中心（云台Yaw轴）的俯视水平距离和垂直距离之和
和左或右后轮到旋转中心（云台Yaw轴）的俯视水平距离和垂直距离之和
*/
#define KXY_FRONT    0.354f ///< front wheels X+Y(distance,m)
#define KXY_BACK     0.354f ///< bask wheels X+Y(distance,m)

#define reverse_rate  1.16065f // KXY_FRONT / (D_WHEEL / 2.f)

/*
	mecanum wheels
	L=left,R=right,F=front,B=back
	motor id & position
	1   0
	2   3
*/
chassis_state_t mecanum_forward(const chassis_wheels_state_t *s_wheel);
chassis_wheels_state_t mecanum_reverse(const chassis_state_t *s_chassis);
void filter_wheels_raw_data(motors_t *p_motors, chassis_wheels_state_t *current_wheels);
HAL_StatusTypeDef chassis_pid_ctrl(motors_t *p_motors, pid_struct_t pPID[4],
                                   chassis_wheels_state_t *current_wheel,
                                   chassis_state_t *ref_state, chassis_state_t *cur_state);


#ifdef __cplusplus
}
#endif
#endif
