
#include "init_task.h"
#include "tim.h" 
#include "can.h" 
#include "FreeRTOS.h"
#include "task.h"

uint32_t chassis_cnt = 0;
uint16_t chassis_freq = 0;

volatile xSemaphoreHandle xMutex_update_rc_last_key;   // 用于保护update_rc_last_key函数
volatile xSemaphoreHandle xMutex_set_all_motor_output; // 用于保护set_all_motor_output函数

QueueHandle_t imuQueue;
SemaphoreHandle_t xChassisSemaphore;
QueueHandle_t rcQueue; // 遥控器数据队列

Chassis chassis;

void chassis_task(void);

void robot_init(void)
{
    __disable_irq(); // 失能中断

    // 初始化 CAN
    uint32_t can_filter_id[4] = {0x201, 0x202, 0x203, 0x204}; // 示例滤波器ID
    can_user_init(&hcan1, can_filter_id, CAN_FILTER_FIFO0);
    can_user_init(&hcan2, can_filter_id, CAN_FILTER_FIFO1);

    HAL_TIM_Base_Start_IT(&htim2); 
    remote_control_init();

    xMutex_set_all_motor_output = xSemaphoreCreateMutex();
    xMutex_update_rc_last_key = xSemaphoreCreateMutex();


    chassis.rocker_queue = xQueueCreate(10, sizeof(RC_ctrl_t *)); // 创建遥控器数据队列
    chassis.mutex = xSemaphoreCreateMutex(); // 创建互斥锁

    // 创建二值信号量
    xChassisSemaphore = xSemaphoreCreateBinary();

    // 创建底盘任务

    xTaskCreate((TaskFunction_t)chassis_task, "Chassis_Task", 128, &xChassisSemaphore, 2, NULL);

    __enable_irq();

    vTaskStartScheduler(); // 开启任务调度
}

void chassis_task(void)
{

    static  RC_ctrl_t *rc_ctrl;

    while (1) {
        // 等待信号量
        if (xSemaphoreTake(xChassisSemaphore, portMAX_DELAY) == pdTRUE) {
            // 更新底盘状态
            Chassis_Update(&chassis);

            // 获取摇杆数据
            // if (xQueueReceive(chassis.rocker_queue, &rocker, 0) == pdPASS) {
            //     // 运行底盘主循环
                Chassis_Loop(&chassis, rc_ctrl);
            // }
        }
    }
}