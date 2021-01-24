#include <rosserial.h>

ROSSerial::ROSSerial() : rind(0), twind(0), tfind(0) {}
void ROSSerial::init_uart(UART_HandleTypeDef *handle, DMA_HandleTypeDef *rxdma,
                          DMA_HandleTypeDef *txdma, USART_TypeDef *usart_inst,
                          DMA_Stream_TypeDef *rxdma_inst,
                          DMA_Stream_TypeDef *txdma_inst, uint32_t rx_channel,
                          uint32_t tx_channel) {

    assert_param(IS_DMA_CHANNEL(rx_channel));
    assert_param(IS_DMA_CHANNEL(tx_channel));
    assert_param(IS_DMA_STREAM_ALL_INSTANCE(rxdma_inst));
    assert_param(IS_DMA_STREAM_ALL_INSTANCE(txdma_inst));

    // enabled clocks
    ROSSERIAL_UART_CLK_EN();
    ROSSERIAL_UART_DMA_CLK_EN();
    ROSSERIAL_GPIO_PORT_CLK_EN();

    // init rosserial uart pins
    GPIO_InitTypeDef GPIO_Config;
    GPIO_Config.Mode = GPIO_MODE_AF_PP;
    GPIO_Config.Pull = GPIO_NOPULL;
    GPIO_Config.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_Config.Alternate = ROSSERUAL_UART_PIN_AF;
    GPIO_Config.Pin = ROSSERIAL_UART_RX_PIN | ROSSERIAL_UART_TX_PIN;
    HAL_GPIO_Init(ROSERIAL_UART_GPIO_PORT, &GPIO_Config);

    // RX DMA Channel config
    rxdma->Instance = rxdma_inst;
    rxdma->Init.Channel = rx_channel;
    rxdma->Init.Direction = DMA_PERIPH_TO_MEMORY;
    rxdma->Init.Mode = DMA_CIRCULAR;
    rxdma->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    rxdma->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    rxdma->Init.MemInc = DMA_MINC_ENABLE;
    rxdma->Init.PeriphInc = DMA_PINC_DISABLE;

    // TX DMA Channel config
    txdma->Instance = txdma_inst;
    txdma->Init.Channel = tx_channel;
    txdma->Init.Direction = DMA_MEMORY_TO_PERIPH;
    txdma->Init.Mode = DMA_NORMAL;
    txdma->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    txdma->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    txdma->Init.MemInc = DMA_MINC_ENABLE;
    txdma->Init.PeriphInc = DMA_PINC_DISABLE;
    txdma->Init.Priority = DMA_PRIORITY_LOW;
    txdma->Init.FIFOMode = DMA_FIFOMODE_DISABLE;

    // USART config
    handle->Init.BaudRate = 115200;
    handle->Init.WordLength = UART_WORDLENGTH_8B;
    handle->Init.Mode = UART_MODE_TX_RX;
    handle->Init.Parity = UART_PARITY_NONE;
    handle->Init.StopBits = UART_STOPBITS_1;
    handle->Init.WordLength = UART_WORDLENGTH_8B;
    handle->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    handle->Init.OverSampling = UART_OVERSAMPLING_16;
    handle->Instance = usart_inst;

    // Init all
    HAL_StatusTypeDef s;
    s = HAL_DMA_Init(rxdma);
    if (s != HAL_OK) {
        Error_Handler();
    }
    s = HAL_DMA_Init(txdma);
    if (s != HAL_OK) {
        Error_Handler();
    }
    txdma->Parent = handle;
    rxdma->Parent = handle;
    handle->hdmarx = rxdma;
    handle->hdmatx = txdma;
    s = HAL_UART_Init(handle);
    if (s != HAL_OK) {
        Error_Handler();
    }
    HAL_NVIC_SetPriority(ROSSERIAL_UART_RXDMA_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ROSSERIAL_UART_RXDMA_IRQn);
    HAL_NVIC_SetPriority(ROSSERIAL_UART_TXDMA_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ROSSERIAL_UART_TXDMA_IRQn);
    HAL_NVIC_SetPriority(ROSSERIAL_UART_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ROSSERIAL_UART_IRQn);
}
void ROSSerial::init() {
    init_uart(&_uarth, &rxdma, &txdma, ROSSERIAL_UART,
              ROSSERIAL_UART_RXDMA_STREAM, ROSSERIAL_UART_TXDMA_STREAM,
              ROSSERIAL_UART_RXDMA_CHANNEL, ROSSERIAL_UART_TXDMA_CHANNEL);
    reset_rbuf();
}
void ROSSerial::reset_rbuf() { HAL_UART_Receive_DMA(&_uarth, rbuf, rbuflen); }
int ROSSerial::read() {
    int c = -1;
    if (rind != getRdmaInd()) {
        c = rbuf[rind++];
        rind &= rbuflen - 1;
    }
    return c;
}
void ROSSerial::flush(void) {
    static bool mutex = false;

    if ((_uarth.gState == HAL_UART_STATE_READY) && !mutex) {
        mutex = true;

        if (twind != tfind) {
            uint16_t len = tfind < twind ? twind - tfind : tbuflen - tfind;
            HAL_UART_Transmit_DMA(&_uarth, &(tbuf[tfind]), len);
            tfind = (tfind + len) & (tbuflen - 1);
        }
        mutex = false;
    }
}
void ROSSerial::write(uint8_t *data, int length) {
    int n = length;
    n = n <= (int)tbuflen ? n : tbuflen;

    int n_tail = n <= (int)(tbuflen - twind) ? n : tbuflen - twind;
    memcpy(&(tbuf[twind]), data, n_tail);
    twind = (twind + n) & (tbuflen - 1);

    if (n != n_tail) {
        memcpy(tbuf, &(data[n_tail]), n - n_tail);
    }

    flush();
}
uint32_t ROSSerial::time() { return HAL_GetTick(); }