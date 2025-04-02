#ifndef __CALLBACK_UART_H
#define __CALLBACK_UART_H

#include "usart.h"

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

#endif
