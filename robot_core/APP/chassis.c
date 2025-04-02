#include "chassis.h"
// #include "stm_log.h"
// #include "vofa.h"
// #include "robot.h"
#include "limits.h"
#include "motor_ctrl.h"
#include "util.h"
#include <math.h>


extern CAN_HandleTypeDef CHASSIS_MOTORS_HCAN;

// 初始化底盘
void Chassis_Init(Chassis *chassis) {
    // 初始化PID控制器
    memcpy(&chassis->wheels_pid[0], &wheel1_pid, sizeof(pid_struct_t));
    memcpy(&chassis->wheels_pid[1], &wheel1_pid, sizeof(pid_struct_t));
    memcpy(&chassis->wheels_pid[2], &wheel1_pid, sizeof(pid_struct_t));
    memcpy(&chassis->wheels_pid[3], &wheel1_pid, sizeof(pid_struct_t));

    memcpy(&chassis->chassis_pid, &chassis_follow_pid, sizeof(pid_struct_t));

    // 注册电机
    motors_t *motors = get_motors_ptr();
    RM_QUAD_motor_register(&motors->RF_wheel, motor_model_RM_QUAD_M3508, QUAD_CURR, COM_CAN,
                           (uint32_t *)&CHASSIS_MOTORS_HCAN, WHEEL_RF_ID, RR_M3508,
                           &RF_wheel_lpf_val, M_CHASSIS, 0, 0);
    RM_QUAD_motor_register(&motors->LF_wheel, motor_model_RM_QUAD_M3508, QUAD_CURR, COM_CAN,
                           (uint32_t *)&CHASSIS_MOTORS_HCAN, WHEEL_LF_ID, RR_M3508,
                           &LF_wheel_lpf_val, M_CHASSIS, 0, 0);
    RM_QUAD_motor_register(&motors->LB_wheel, motor_model_RM_QUAD_M3508, QUAD_CURR, COM_CAN,
                           (uint32_t *)&CHASSIS_MOTORS_HCAN, WHEEL_LB_ID, RR_M3508,
                           &LB_wheel_lpf_val, M_CHASSIS, 0, 0);
    RM_QUAD_motor_register(&motors->RB_wheel, motor_model_RM_QUAD_M3508, QUAD_CURR, COM_CAN,
                           (uint32_t *)&CHASSIS_MOTORS_HCAN, WHEEL_RB_ID, RR_M3508,
                           &RB_wheel_lpf_val, M_CHASSIS, 0, 0);

    // 初始化状态变量
    memset(&chassis->current_wheels, 0, sizeof(chassis_wheels_state_t));
    memset(&chassis->expected_state, 0, sizeof(chassis_state_t));
    memset(&chassis->current_state, 0, sizeof(chassis_state_t));
    chassis->chassis_reset_flag = 0;
    chassis->exp_yaw = 0;
    chassis->cur_yaw = 0;

    
    //  ? STM_LOGI("Chassis: initialized");
}

// 更新底盘状态
void Chassis_Update(Chassis *chassis) {
    // 从队列中获取偏航角数
        filter_wheels_raw_data(chassis->motors, &chassis->current_wheels);
 
}

// 底盘主循环
void Chassis_Loop(Chassis *chassis, const RC_ctrl_t *rc_ctrl) {
    // 获取互斥锁以保护共享资源
    if (xSemaphoreTake(chassis->mutex, portMAX_DELAY) == pdTRUE) {
        switch (chassis->tag) {
            case CHASSIS_STOP:
                Chassis_State_clear(chassis, &chassis->expected_state);
                shutdown_motors_of_structure(CHASSIS_MOTORS);
                break;

            case CHASSIS_LAUNCHING:
                Chassis_Init(chassis);
                break;

            case CHASSIS_ENABLE:
                chassis->current_state = mecanum_forward(&chassis->current_wheels);
                chassis->exp_yaw -= rc_ctrl->mouse.press_l * 0.0030;
                float diff_yaw = get_minor_arc(chassis->exp_yaw, chassis->cur_yaw, 360.f);
                chassis->expected_state.v_x = s_curve(MAX_VX_SPEED, rc_ctrl->mouse.y);
                chassis->expected_state.v_y = s_curve(MAX_VY_SPEED, rc_ctrl->mouse.x);
                chassis->expected_state.w_z = pid_calc_deadband(&chassis->chassis_pid, -(float)diff_yaw, 0);

                chassis_pid_ctrl(chassis->motors, chassis->wheels_pid, &chassis->current_wheels, &chassis->expected_state, &chassis->current_state);
                break;
        }
        // 释放互斥锁
        xSemaphoreGive(chassis->mutex);
    }
}

// 清除底盘状态
void Chassis_State_clear(Chassis *chassis, chassis_state_t *state) {
    for (int i = 0; i < 4; i++) {
        pid_reset(&chassis->wheels_pid[i]);
    }
    pid_reset(&chassis->chassis_pid);
    state->v_x = 0, state->v_y = 0, state->w_z = 0;
    memset(&chassis->current_wheels, 0, sizeof(chassis_wheels_state_t));
    chassis->chassis_reset_flag = 0;
}

// 更新底盘状态标签
void Chassis_Update_tag(Chassis *chassis, chassis_tag_t C_TAG) {
    if (xSemaphoreTake(chassis->mutex, portMAX_DELAY) == pdTRUE) {
        chassis->tag = C_TAG;
        xSemaphoreGive(chassis->mutex);
    }
}

// 检查底盘是否复位
uint8_t Chassis_Is_reseted(Chassis *chassis) {
    return chassis->chassis_reset_flag;
}

