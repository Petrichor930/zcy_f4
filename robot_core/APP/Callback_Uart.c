/**
 * @file Callback_Uart.c
 * @brief 串口回调重写函数
 * @copyright SCNU-PIONEER (c) 2024-2025
 */
 #include "Remote_Control.h"
 #include <stdint.h>
 #include "drv_conf.h"
 #include "motor_headers.h"
 #include "Callback_Uart.h"
//  #include "teach_pendant.h"
 
 // Teach_pendant 的实例需要在 C 中通过全局变量实现
//  extern Teach_pendant teach_pendant;
 
 /**
  * @brief UART 接收事件回调函数
  * @param huart UART 句柄
  * @param Size 接收到的数据大小
  */
 void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
 {
     // ━━━━━━━━━━━━━━━━━━━━━━━━━━━ dji_dt7遥控器dbus回调 ━━━━━━━━━━━━━━━━━━━━━━━━
     if (huart->Instance == RC_UART) {
         rc_uart_idle_handle();
     }
 
 #ifdef USE_VT_RC_UART
     // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ 图传串口回调 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
     if (huart->Instance == VT_RC_UART) {
         vt_uart_idle_handle(&VT_RC_UART_HANDLE);
     }
 #endif
 
//  #ifdef USE_REFEREE
//      // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ 裁判系统串口回调 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
//      if (huart->Instance == REFEREE_UART) {
//          referee_uart_idle_handle(huart);
//      }
//  #endif
 
//      // ━━━━━━━━━━━━━━━━━━━━━━━━━━ 自定义控制器调试串口回调 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━
//      if (huart->Instance == EXTENSION_UART) {
//          teach_pendant_idle_callback(&EXTENSION_UART_HANDLE);
//      }
 
     // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━ RS485串口DMA接受回调 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━
     if (huart->Instance == UNITREE_UART) {
         parse_UT_motor_data(&UNITREE_UART_HANDLE);
     }
 }
