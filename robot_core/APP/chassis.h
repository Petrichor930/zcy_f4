#ifndef _CHASSIS_H_
#define _CHASSIS_H_

#include "chassis_base.h" // 包含底盘基础定义
#include "pid.h"
#include "motor_headers.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h" // FreeRTOS 定时器头文件
#include "Remote_Control.h"


#include <math.h>
#include <string.h>

// 底盘参数定义
#define S_CURVE_VX_ACC 1.4f // X轴加速度
#define S_CURVE_VY_ACC 2.2f // Y轴加速度
#define S_CURVE_WZ_ACC 2.8f // Z轴角加速度

#define MAX_VX_SPEED 2.0f   // X轴最大速度
#define MAX_VY_SPEED 2.0f   // Y轴最大速度
#define MAX_WZ_SPEED 6.0f   // Z轴最大角速度

#define MAX_VX_SHIFT_SPEED 0.5f  // X轴平移最大速度
#define MAX_VY_SHIFT_SPEED 1.4f  // Y轴平移最大速度
#define MAX_WZ_SHIFT_SPEED 12.0f // Z轴旋转最大速度

// 底盘状态枚举
typedef enum {
    CHASSIS_STOP = 1u, // 失能
    CHASSIS_ENABLE,    // 使能
    CHASSIS_LAUNCHING  // 缓启动
} chassis_tag_t;

// 底盘控制结构体
typedef struct {
    pid_struct_t wheels_pid[4]; // 四轮PID控制器
    pid_struct_t chassis_pid;   // 底盘PID控制器

    chassis_state_t expected_state; // 期望状态zs
    chassis_state_t current_state;  // 当前状态
    chassis_wheels_state_t current_wheels; // 当前四轮速度
    uint8_t chassis_reset_flag; // 底盘复位标志
    float exp_yaw; // 期望偏航角
    float cur_yaw; // 当前偏航角
    motors_t *motors; // 电机指针
    chassis_tag_t tag; // 底盘状态标签

    QueueHandle_t yaw_queue; // 用于接收偏航角数据的队列
    QueueHandle_t rocker_queue; // 用于接收摇杆数据的队列
    SemaphoreHandle_t mutex; // 用于保护共享资源的互斥锁
    TimerHandle_t chassis_timer; // FreeRTOS 定时器句柄
} Chassis;

// 函数声明
void Chassis_Init(Chassis *chassis);
void Chassis_Update(Chassis *chassis);
void Chassis_Loop(Chassis *chassis, const RC_ctrl_t *rc_ctrl);void Chassis_State_clear(Chassis *chassis, chassis_state_t *state);
void Chassis_Update_tag(Chassis *chassis, chassis_tag_t C_TAG);
uint8_t Chassis_Is_reseted(Chassis *chassis);

#endif