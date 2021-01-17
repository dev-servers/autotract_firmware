#pragma once

#include <errors.h>
#include <stm32f4xx_hal.h>
#include <main.h>

void init_gpio();
void init_uart(UART_HandleTypeDef *handle, DMA_HandleTypeDef *rxdma,
               DMA_HandleTypeDef *txdma, USART_TypeDef *usart_inst,
               DMA_Stream_TypeDef *rxdma_inst, DMA_Stream_TypeDef *txdma_inst,
               uint32_t rx_channel, uint32_t tx_channel);
void init_clocks();