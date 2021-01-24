#pragma once

#include <stm32f4xx_hal.h>
#include <ros/node_handle.h>
#include <hardware.h>

struct ROSSerial {
  public:
    ROSSerial();
    void init();
    void reset_rbuf();
    int read();
    void flush();
    void write(uint8_t *data, int length);
    uint32_t time();
    static void init_uart(UART_HandleTypeDef *handle, DMA_HandleTypeDef *rxdma,
                          DMA_HandleTypeDef *txdma, USART_TypeDef *usart_inst,
                          DMA_Stream_TypeDef *rxdma_inst,
                          DMA_Stream_TypeDef *txdma_inst, uint32_t rx_channel,
                          uint32_t tx_channel);

    UART_HandleTypeDef _uarth;

  protected:
    DMA_HandleTypeDef rxdma;
    DMA_HandleTypeDef txdma;
    const static uint32_t tbuflen = 512;
    const static uint32_t rbuflen = 512;
    uint8_t tbuf[tbuflen];
    uint8_t rbuf[rbuflen];

    uint32_t rind;
    uint32_t twind, tfind;

    inline uint32_t getRdmaInd(void) {
        return (rbuflen - __HAL_DMA_GET_COUNTER(_uarth.hdmarx)) & (rbuflen - 1);
    }
};

typedef ros::NodeHandle_<ROSSerial> NodeHandle;