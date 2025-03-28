#ifndef _UART_DRV_H
#define _UART_DRV_H

#include "drv_conf.h"
#include HAL_INCLUDE

#define UART_TIME_OUT 5

#if defined(STM32H723xx)
HAL_StatusTypeDef uart_recv_dma_H7multibuffer_init(UART_HandleTypeDef *huart, uint32_t *DstAddress,
                                                   uint32_t *SecondMemAddress, uint32_t DataLength);
#endif


uint32_t uart_send(UART_HandleTypeDef *huart, uint8_t *p_data, uint16_t size);
HAL_StatusTypeDef uart_send_async(UART_HandleTypeDef *huart, uint8_t *p_data, uint16_t size);

HAL_StatusTypeDef uart_recv_dma_init(UART_HandleTypeDef *huart, uint8_t *rx_buffer, uint16_t len);

#endif
