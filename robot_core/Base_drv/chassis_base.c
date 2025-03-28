#include "dev_RM.h"
#include "motor_pid_param.h"
#include "pid.h"
#include <math.h>
// #include "filter.h"
#include <string.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "motor_ctrl.h"
#include "chassis_base.h"
#include "motor_headers.h"

#include "filter.h"
extern CAN_HandleTypeDef CHASSIS_MOTORS_HCAN;

/**
 * @brief mecanum forward calcuation
 * @param[in]  s_wheel     pointer to chassis_wheels_state_t (4 wheel rpm)
 * @return chassis_state_t including v_x,v_y,w_z
 */
/*麦克纳姆轮运动学正向解算，即从四轮轮速换算到整车速度*/
chassis_state_t mecanum_forward(const chassis_wheels_state_t *s_wheel)
{
    chassis_state_t s_chassis;
    s_chassis.v_x =
            C_WHEEL * (s_wheel->M_LF - s_wheel->M_RF + s_wheel->M_LB - s_wheel->M_RB) / 4 / 60;
    s_chassis.v_y = C_WHEEL *
                    (KXY_BACK * s_wheel->M_RF + KXY_BACK * s_wheel->M_LF -
                     KXY_FRONT * s_wheel->M_LB - KXY_FRONT * s_wheel->M_RB) /
                    (2 * (KXY_BACK + KXY_FRONT)) / 60;
    s_chassis.w_z = (D_WHEEL / 2) *
                    (s_wheel->M_RF + s_wheel->M_LF + s_wheel->M_LB + s_wheel->M_RB) /
                    (2 * (KXY_BACK + KXY_FRONT)) * (2.f * PI) / 60.f;
    return s_chassis;
}

/**
 * @brief mecanum reverse calcuation
 * @param[in]  s_chassis    pointer to chassis_state_t (v_x,v_y,w_z)
 * @return chassis_wheels_state_t (4 wheels speed)
 */
/*麦克纳姆轮运动学逆向解算，即从整车速度换算到四轮轮速 轮速单位:rpm,wz单位:rad/s*/
chassis_wheels_state_t mecanum_reverse(const chassis_state_t *s_chassis)
{
    chassis_wheels_state_t s_wheel;
    s_wheel.M_RF = (60 / C_WHEEL) * (-s_chassis->v_x + s_chassis->v_y) +
                   KXY_FRONT / (D_WHEEL / 2) * s_chassis->w_z * 60.f / (2.f * PI);
    s_wheel.M_LF = (60 / C_WHEEL) * (s_chassis->v_x + s_chassis->v_y) +
                   KXY_FRONT / (D_WHEEL / 2) * s_chassis->w_z * 60.f / (2.f * PI);
    s_wheel.M_LB = (60 / C_WHEEL) * (s_chassis->v_x - s_chassis->v_y) +
                   KXY_BACK / (D_WHEEL / 2) * s_chassis->w_z * 60.f / (2.f * PI);
    s_wheel.M_RB = (60 / C_WHEEL) * (-s_chassis->v_x - s_chassis->v_y) +
                   KXY_BACK / (D_WHEEL / 2) * s_chassis->w_z * 60.f / (2.f * PI);
    return s_wheel;
}

/**
  * @brief update the global varible using DJI_motor_data with IIR filter
  * @param[in]  DJI_motor_data     pointer to the RM_RM_raw_motor_data_t of 4 wheels,
						   id and position should match
  * @param[out]  current_wheels pointer to where the caller intended to store the result
							in the type chassis_wheels_state_t
  * @return void
  */
/*电机数据滤波*/
 void filter_wheels_raw_data(motors_t *p_motors, chassis_wheels_state_t *current_wheels)
 {
     current_wheels->M_RF = iir_filter_3((float)p_motors->RF_wheel.real.rpm, WHEEL_RF_ID - 1);
     current_wheels->M_LF = iir_filter_3((float)p_motors->LF_wheel.real.rpm, WHEEL_LF_ID - 1);
     current_wheels->M_LB = iir_filter_3((float)p_motors->LB_wheel.real.rpm, WHEEL_LB_ID - 1);
     current_wheels->M_RB = iir_filter_3((float)p_motors->RB_wheel.real.rpm, WHEEL_RB_ID - 1);
 }

/**
 * @brief chassis motion control process
 * @param[in]  pPID           pointer to the pid_struct_t of 4 wheels speed
 * @param[in]  current_state  pointer to chassis_wheels_state_t (4 wheel speed)
 * @param[in]  ref_state      expected v_x,v_y,w_z in chassis_state_t
 * @return HAL_OK if transmission was success otherwise HAL_ERROR
 * @attention make sure to limit the output value in (-30000,30000)
 */
extern xSemaphoreHandle xMutex_set_all_motor_output;
chassis_wheels_state_t wheels_ref;
float diff_speed[4] = { 0 };
float exp_T_ff[4] = { 0 };
HAL_StatusTypeDef chassis_pid_ctrl(motors_t *p_motors, pid_struct_t pPID[4],
                                   chassis_wheels_state_t *current_wheel,
                                   chassis_state_t *ref_state, chassis_state_t *cur_state)
{
    HAL_StatusTypeDef rslt = HAL_ERROR;

    wheels_ref = mecanum_reverse(ref_state);

    diff_speed[0] = pid_calc(&pPID[0], wheels_ref.M_RF, current_wheel->M_RF);
    diff_speed[1] = pid_calc(&pPID[1], wheels_ref.M_LF, current_wheel->M_LF);
    diff_speed[2] = pid_calc(&pPID[2], wheels_ref.M_LB, current_wheel->M_LB);
    diff_speed[3] = pid_calc(&pPID[3], wheels_ref.M_RB, current_wheel->M_RB);

    for (uint8_t i = 0; i < WHEELS_NUM; i++) {
        exp_T_ff[i] = diff_speed[i] * C620_CURR_MAX / C620_CURR_DATA_MAX * Kn_M3508;
        LOAD_MOTOR_LPFTFF(p_motors->_[CHASSIS_MOTORS_INDEX + i], exp_T_ff[i]);
    }

    if (xSemaphoreTake(xMutex_set_all_motor_output, portMAX_DELAY) == pdTRUE) {
        rslt |= set_all_motor_output(CHASSIS_MOTORS);
        xSemaphoreGive(xMutex_set_all_motor_output);
    }
    return rslt;
}
