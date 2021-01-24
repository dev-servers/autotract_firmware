#include <main.h>
#include <stm32f4xx_hal.h>

extern App app;
extern Steering steering;

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

// ROSSerial DMA RX Stream
extern "C" void ROSSERIAL_UART_RXDMA_IRQHandler(void) {
    HAL_DMA_IRQHandler(app.node_handle.getHardware()->_uarth.hdmarx);
}
// ROSSerial DMA TX Stream
extern "C" void ROSSERIAL_UART_TXDMA_IRQHandler(void) {
    HAL_DMA_IRQHandler(app.node_handle.getHardware()->_uarth.hdmatx);
}
// ROSSerial USART Handler
extern "C" void ROSSERIAL_UART_IRQHandler(void) {
    HAL_UART_IRQHandler(&(app.node_handle.getHardware()->_uarth));
}
// Stepper pulse timer handle
extern "C" void STEPPER_PULSE_TIM_HANDLER(void) {
    HAL_TIM_IRQHandler(&(app.steering.stepper._tim_handle));
}
extern "C" void ENCODER_TIM_HANDLER(void) {
    HAL_TIM_IRQHandler(&(app.steering.encoder._tim_handle));
}