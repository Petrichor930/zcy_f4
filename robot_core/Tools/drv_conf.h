#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stdbool.h"
// #include "stdlib.h"
// #include "string.h"

#define HAL_INCLUDE "stm32f4xx_hal.h"
#define CAN_INCLUDE "drv_can.h"

#define PROJECT_NAME "robot"
#define PROJECT_VERSION "2.0.0"

#define RAM_PERSIST

#define MASTER 1
#define MACHINE_TYPE MASTER

#define ENGINEER 2
#define ROBOT_TYPE ENGINEER

#define USE_OS

#define CORE_FREQ         550  // Mhz
#define ARM_CTRL_FREQ     500  // 500hz
#define CHASSIS_CTRL_FREQ 250  //250hz
#define IMU_CTRL_FREQ     500  //500hz
#define DETECT_TIM_FREQ   10  //10hz

#define USE_LOG
#define USE_DEBUG  //开启调试，有对应的hardfault处理
/* #undef READY_FOR_BATTLE */

#define USE_HAL_TICK   //HAL TICK
/* #undef USE_TEMP_CTRL */
#define USE_BEEP       //蜂鸣器
#define USE_WS2812     //灯珠

#define USE_REFEREE  //裁判系统 usart1
#define USE_UNITREE_UART  //宇树电机串口转RS485 usart2
#define USE_RC_UART       //遥控 dbus uart5
#define USE_VOFA          //vofa uart7
/* #undef USE_CC_UART */
#define USE_EXTENSION_UART      //拓展板通信 uart9
/* #undef USE_VT_RC_UART */


#define USE_IMU

/*-----------------------------------------------------------------*/
#ifdef USE_HAL_TICK
#define  HALTICK_TIM TIM4 
#endif

#ifdef USE_TEMP_CTRL
#define TEMP_TIM_HANDLE  htim3
#define TEMP_TIM_CHANNAL TIM_CHANNEL_4
#endif

#ifdef USE_BEEP
#define BEEP_TIM_HANDLE  htim12
#define BEEP_TIM_CHANNAL TIM_CHANNEL_2
#endif

#ifdef USE_WS2812
#define WS2812_SPI_HANDLE  hspi6
#endif

#ifdef USE_REFEREE
#define REFEREE_UART        USART1
#define REFEREE_UART_HANDLE huart1
#define REFEREE_ASYNC_SEND  0
#endif

#ifdef USE_UNITREE_UART
#define UNITREE_UART        USART2
#define UNITREE_UART_HANDLE huart2
#define UNITREE_DMA_HANDLE  hdma_usart2_rx
#endif

#ifdef USE_RC_UART
#define RC_UART        USART3
#define RC_UART_HANDLE huart3
#endif

#ifdef USE_VOFA
#define VOFA_UART          UART7
#define VOFA_UART_HANDLE   huart7
#define VOFA_DMA_TX_HANDLE hdma_uart7_tx
#endif

#ifdef USE_EXTENSION_UART
#define EXTENSION_UART          UART9
#define EXTENSION_UART_HANDLE   huart9
#define EXTENSION_DMA_TX_HANDLE hdma_uart9_tx 
#define EXTENSION_DMA_RX_HANDLE hdma_uart9_rx 
#endif

#ifdef USE_VT_RC_UART
#define VT_RC_UART        USART10
#define VT_RC_UART_HANDLE huart10
#define VT_RC_DMA_HANDLE hdma_usart10_rx
#endif

#ifdef USE_IMU
#define IMU_SPI hspi2
#endif

#ifdef __cplusplus
}
#endif
