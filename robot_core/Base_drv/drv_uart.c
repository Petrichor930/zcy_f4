#include "drv_uart.h"

#define RX_DR_REG(huart) ((huart)->Instance->RDR)

// https://zhuanlan.zhihu.com/p/720966722
// tips: H7的串口设置不要把overrun和idle事件混用，否则会导致数据丢失 反正默认别勾上overrun就行了
HAL_StatusTypeDef uart_recv_dma_H7multibuffer_init(UART_HandleTypeDef *huart, uint32_t *DstAddress,
                                                   uint32_t *SecondMemAddress, uint32_t DataLength)
{
    huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
    huart->RxEventType = HAL_UART_RXEVENT_IDLE;
    huart->RxXferSize = DataLength;
    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    return HAL_DMAEx_MultiBufferStart(huart->hdmarx, (uint32_t)&huart->Instance->DR,         //?
                                      (uint32_t)DstAddress, (uint32_t)SecondMemAddress, DataLength);
}

/**
 * @brief      enable uart receive uart it and initialize DMA
 * @param[in]  huart   pointer to the uart handle
 * @param[in]  rx_buffer   pointer to the receive buffer
 * @param[in]  len   buffer length
 * @return     set HAL_OK or HAL_BUSY
 */
/*串口初始化DMA缓冲区函数*/
HAL_StatusTypeDef uart_recv_dma_init(UART_HandleTypeDef *huart, uint8_t *rx_buffer, uint16_t len)
{
    HAL_UART_StateTypeDef state = huart->RxState;
    if (state == HAL_UART_STATE_READY) {
        __HAL_LOCK(huart);
        huart->pRxBuffPtr = (uint8_t *)rx_buffer;
        huart->RxXferSize = len;
        huart->ErrorCode = HAL_UART_ERROR_NONE;

        /* Enable the DMA Stream */                                              //?
        HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, (uint32_t)rx_buffer, len);
        // HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, (uint32_t)pData, Size);
        /*
		 * Enable the DMA transfer for the receiver request by setting the DMAR bit
		 * in the UART CR3 register
		 */
        SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_IDLEFLAG(huart);

        //__HAL_UART_ENABLE_IT(huart,UART_IT_IDLE);

        __HAL_UNLOCK(huart);

        return HAL_OK;
    } else {
        return HAL_BUSY;
    }
}

/**
 * @brief      send a couple of characters to uart
 * @param[in]  huart   pointer to the uart handle
 * @param[in]  p_data   pointer to the data buffer
 * @param[in]  size   data length to send
 * @return     the remaining size unsend, zero for success
 */
uint32_t uart_send(UART_HandleTypeDef *huart, uint8_t *p_data, uint16_t size)
{
    uint32_t start_tick = HAL_GetTick();
    while (size) {                //?
        WRITE_REG(huart->Instance->DR, *p_data);
        while (__HAL_UART_GET_FLAG(huart, UART_FLAG_TXE) != SET) {
            if (HAL_GetTick() - start_tick > UART_TIME_OUT)
                return size;
        }
        size--;
        p_data++;
    }
    return 0;
    // HAL_UART_Transmit(huart,pData,size,UART_TIME_OUT);
}

/**
 * @brief      send a couple of characters to uart (non-blocking)
 * @param[in]  huart   pointer to the uart handle
 * @param[in]  p_data   pointer to the data buffer
 * @param[in]  size   data length to send
 * @return      HAL status
 */
HAL_StatusTypeDef uart_send_async(UART_HandleTypeDef *huart, uint8_t *p_data, uint16_t size)
{
    return HAL_UART_Transmit_DMA(huart, p_data, size);
}

