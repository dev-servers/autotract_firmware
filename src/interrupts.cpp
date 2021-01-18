#include <main.h>
#include <stm32f4xx_hal.h>

extern UART_HandleTypeDef rosserial_handle;
extern AppThreadsCounters app;
extern "C" void SysTick_Handler(void) {
    HAL_IncTick();
    app.inc_counters();
}
extern "C" void Hardfault_Handler(void) {
    for (;;)
        ;
}
extern "C" void NMI_Handler(void) {}
extern "C" void MemManage_Handler(void) {
    for (;;)
        ;
}

extern "C" void BusFault_Handler(void) {
    for (;;)
        ;
}

extern "C" void UsageFault_Handler(void) {
    for (;;)
        ;
}

extern "C" void SVC_Handler(void) {}

extern "C" void DebugMon_Handler(void) {}

extern "C" void PendSV_Handler(void) {}

// RX Stream
extern "C" void ROSSERIAL_UART_RXDMA_IRQHandler(void) {
    HAL_DMA_IRQHandler(rosserial_handle.hdmarx);
}
// TX Stream
extern "C" void ROSSERIAL_UART_TXDMA_IRQHandler(void) {
    HAL_DMA_IRQHandler(rosserial_handle.hdmatx);
}
// USART Handler
extern "C" void ROSSERIAL_UART_IRQHandler(void) {
    HAL_UART_IRQHandler(&rosserial_handle);
}