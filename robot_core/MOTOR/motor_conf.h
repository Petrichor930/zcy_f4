/**
 * @file motor_conf_sample.h
 *
 *
 * @brief 电机库主要配置示例文件
 * @note 根据下面的修改指引 创建副本并改名为motor_conf.h于根目录
 *
 * @copyright SCNU-PIONEER (c) 2024-2025
 *
 */
#ifndef MOTOR_CONF_H_
#define MOTOR_CONF_H_


#include "drv_conf.h"
#ifndef HAL_INCLUDE
#error "HAL_INCLUDE is not defined"
#else
#include HAL_INCLUDE // 本宏来自 drv_conf:stm32h7xx_hal.h
#endif
#ifndef CAN_INCLUDE
#error "CAN_INCLUDE is not defined"
#else
#include CAN_INCLUDE // 本宏来自 drv_conf:fdcan.h
#endif               // ! CAN_INCLUDE

#include "motor_def.h"

#ifdef __cplusplus
extern "C" {
#endif


#define MOTOR_CONF_FILE      "motor_conf.h"

#define WHEELS_NUM           4 // 轮毂电机数量

#define USE_TEST_FILE        0 // 是否使用测试电机

/*--------------------------------*/
#define GIMBAL_MOTORS_HCAN   hcan1
#define GIMBAL_MOTORS_NUM    0
#define GIMBAL_MOTORS_INDEX  0
/*--------------------------------*/
#define SHOOTER_MOTORS_HCAN  hcan2
#define SHOOTER_MOTORS_NUM   0
#define SHOOTER_MOTORS_INDEX 0
/*--------------------------------*/
#define ARM_MOTORS_HCAN      hcan2
#define ARM_MOTORS_NUM       7
#define ARM_MOTORS_INDEX     0
/*--------------------------------*/
#define CHASSIS_MOTORS_HCAN  hcan1
#define CHASSIS_MOTORS_NUM   4
#define CHASSIS_MOTORS_INDEX ARM_MOTORS_NUM - ARM_MOTORS_INDEX
/*--------------------------------*/
#define TEST_MOTORS_HCAN     hcan1
#if USE_TEST_FILE
#define TEST_MOTORS_NUM 1
#else
#define TEST_MOTORS_NUM 0
#endif
#define TEST_MOTORS_INDEX    (ARM_MOTORS_NUM + CHASSIS_MOTORS_NUM) // init_index:9
/*--------------------------------*/

//  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━ 电机数量面板 ━━━━━━━━━━━━━━━━━━━━━━━━━
#define NUM_OF_RM_QUAD_MOTOR (5)
#define NUM_OF_LK_MOTOR      0
#define NUM_OF_LK_QUAD_MOTOR 0
#define NUM_OF_DM_MOTOR      (5)
#define NUM_OF_DM_QUAD_MOTOR 0
#define NUM_OF_UT_MOTOR      (1)

#define NUM_OF_ALL_MOTOR                                                               \
    (NUM_OF_RM_QUAD_MOTOR + NUM_OF_LK_MOTOR + NUM_OF_LK_QUAD_MOTOR + NUM_OF_DM_MOTOR + \
     NUM_OF_DM_QUAD_MOTOR + NUM_OF_UT_MOTOR)

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ 电机id ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#define WHEEL_RF_ID 1
#define WHEEL_LF_ID 2
#define WHEEL_LB_ID 3
#define WHEEL_RB_ID 4

#define JOINT1_ID   1
#define JOINT2_ID   2
#define JOINT3_ID   3
#define JOINT4_ID   4
#define JOINT5_ID   5
#define JOINT6_ID   6
#define JOINT7_ID   7

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ 所有电机结构体 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
typedef union {
    struct {
        motor_t joint1;
        motor_t joint2;
        motor_t joint3;
        motor_t joint4;
        motor_t joint5;
        motor_t joint6;
        motor_t joint7;

        motor_t RF_wheel; // 右前方麦轮电机
        motor_t LF_wheel; // 左前方麦轮电机
        motor_t LB_wheel; // 左后方麦轮电机
        motor_t RB_wheel; // 右后方麦轮电机

#if USE_TEST_FILE
        motor_t test; // 测试用电机
#endif
    };

    motor_t _[NUM_OF_ALL_MOTOR]; // 方便遍历用
    // 需要在上面写好不同类型的电机数量 否则NUM_OF_ALL_MOTOR将不会正常工作
} motors_t;


#if USE_TEST_FILE
// #warning "Test Motor is enabled!"
#endif
#if NUM_OF_ALL_MOTOR != GIMBAL_MOTORS_NUM + SHOOTER_MOTORS_NUM + CHASSIS_MOTORS_NUM + \
                                ARM_MOTORS_NUM + TEST_MOTORS_NUM
#error "NUM_OF_ALL_MOTOR is not correct!"
// 此项报错 则说明机器人 应启用电机个数 与 实际存在电机数量 不匹配
#endif


#ifdef __cplusplus
}
#endif
/*-----------------------------------------------------------------*/

#endif
