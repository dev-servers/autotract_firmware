#include <stm32f4xx_hal.h>

extern UART_HandleTypeDef rosserial_handle;

extern "C" void SysTick_Handler(void) { HAL_IncTick(); }
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
extern "C" void DMA1_Stream1_IRQHandler(void) {
    HAL_DMA_IRQHandler(rosserial_handle.hdmarx);
}
// TX Stream
extern "C" void DMA1_Stream3_IRQHandler(void) {
    HAL_DMA_IRQHandler(rosserial_handle.hdmatx);
}
// USART Handler
extern "C" void USART3_IRQHandler(void) {
    HAL_UART_IRQHandler(&rosserial_handle);
}