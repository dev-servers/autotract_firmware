#pragma once
#include <stm32f4xx_hal.h>
#include <errors.h>
/* Target Definitions
    Application specific definitation for the target board. add a definition in
   the CMakeLists.txt file with the unique board name and use that in an ifdef
   block to define board specific directives start with board number so it's
   easy to check that the uploading firmware is the correct one while debugging,
   board number is constexpr to show up in the debugger
*/

#define MAX_STEPPER_INSTANCES 1
#ifdef NUCLEO
constexpr uint32_t BOARD = 1;
#define SWD_PORT_CLK_EN __HAL_RCC_GPIOA_CLK_ENABLE
#define ROSSERIAL_UART USART3
#define ROSSERIAL_UART_IRQn USART3_IRQn
#define ROSSERIAL_UART_CLK_EN __HAL_RCC_USART3_CLK_ENABLE
#define ROSSERIAL_UART_RXDMA_STREAM DMA1_Stream1
#define ROSSERIAL_UART_TXDMA_STREAM DMA1_Stream3
#define ROSSERIAL_UART_RXDMA_CHANNEL DMA_CHANNEL_4
#define ROSSERIAL_UART_TXDMA_CHANNEL DMA_CHANNEL_4
#define ROSSERIAL_UART_RXDMA_IRQn DMA1_Stream1_IRQn
#define ROSSERIAL_UART_TXDMA_IRQn DMA1_Stream3_IRQn
#define ROSSERIAL_UART_DMA_CLK_EN __HAL_RCC_DMA1_CLK_ENABLE
#define ROSSERUAL_UART_PIN_AF GPIO_AF7_USART3
#define ROSSERIAL_UART_RX_PIN GPIO_PIN_9
#define ROSSERIAL_UART_TX_PIN GPIO_PIN_8
#define ROSERIAL_UART_GPIO_PORT GPIOD
#define ROSSERIAL_GPIO_PORT_CLK_EN __HAL_RCC_GPIOD_CLK_ENABLE
#define ROSSERIAL_UART_IRQHandler USART3_IRQHandler
#define ROSSERIAL_UART_RXDMA_IRQHandler DMA1_Stream1_IRQHandler
#define ROSSERIAL_UART_TXDMA_IRQHandler DMA1_Stream3_IRQHandler
#define HEARTBEAT_LED_PIN GPIO_PIN_14
#define HEARTBEAT_LED_PORT GPIOB
#define HEARTBEAT_LED_PORT_CLK_EN __HAL_RCC_GPIOB_CLK_ENABLE
#define STATUS_LED_PIN GPIO_PIN_0
#define STATUS_LED_PORT GPIOB
#define STATUS_LED_PORT_CLK_EN __HAL_RCC_GPIOB_CLK_ENABLE

#define STEPPER_PULSE_TIM TIM3
#define STEPPER_PULSE_AF GPIO_AF2_TIM3
#define STEPPER_PULSE_TIM_CH TIM_CHANNEL_2
#define STEPPER_PULSE_TIM_HANDLER TIM3_IRQHandler
#define STEPPER_PULSE_TIM_IRQn TIM3_IRQn
#define STEPPER_PULSE_PIN GPIO_PIN_5
#define STEPPER_PULSE_PORT GPIOB

#define STEPPER_DIR_PIN GPIO_PIN_4
#define STEPPER_DIR_PORT GPIOB

#define STEPPER_EN_PIN GPIO_PIN_3
#define STEPPER_EN_PORT GPIOB

#define STEPPER_ALM_PIN GPIO_PIN_4
#define STEPPER_ALM_PORT GPIOB

#define STEPPER_PORTS_CLK_EN __HAL_RCC_GPIOB_CLK_ENABLE
#define STEPPER_TIM_CLK_EN __HAL_RCC_TIM3_CLK_ENABLE

#define ENCODER_TIM TIM4
#define ENCODER_TIM_CLK_EN __HAL_RCC_TIM4_CLK_ENABLE
#define ENCODER_TIM_HANDLER TIM4_IRQHandler
#define ENCODER_TIM_IRQn TIM4_IRQn
#define ENCODER_CHA TIM_CHANNEL_1
#define ENCODER_CHB TIM_CHANNEL_2
#define ENCODER_A_PIN GPIO_PIN_6
#define ENCODER_B_PIN GPIO_PIN_13
#define ENCODER_AF GPIO_AF2_TIM4
#define ENCODER_A_PORT GPIOB
#define ENCODER_B_PORT GPIOD
#define ENCODER_PORTS_CLK_EN()                                                 \
    do {                                                                       \
        __HAL_RCC_GPIOB_CLK_ENABLE();                                          \
        __HAL_RCC_GPIOD_CLK_ENABLE();                                          \
    } while (0);

#endif

#ifdef BLACKPILL
constexpr uint32_t BOARD = 1;
#define SWD_PORT_CLK_EN __HAL_RCC_GPIOA_CLK_ENABLE
#define ROSSERIAL_UART USART6
#define ROSSERIAL_UART_CLK_EN __HAL_RCC_USART6_CLK_ENABLE
#define ROSSERIAL_UART_IRQn USART6_IRQn
#define ROSSERIAL_UART_RXDMA_STREAM DMA2_Stream1
#define ROSSERIAL_UART_TXDMA_STREAM DMA2_Stream6
#define ROSSERIAL_UART_RXDMA_CHANNEL DMA_CHANNEL_4
#define ROSSERIAL_UART_TXDMA_CHANNEL DMA_CHANNEL_4
#define ROSSERIAL_UART_RXDMA_IRQn DMA2_Stream1_IRQn
#define ROSSERIAL_UART_TXDMA_IRQn DMA2_Stream6_IRQn
#define ROSSERIAL_UART_DMA_CLK_EN __HAL_RCC_DMA2_CLK_ENABLE
#define ROSSERUAL_UART_PIN_AF GPIO_AF8_USART6
#define ROSSERIAL_UART_RX_PIN GPIO_PIN_12
#define ROSSERIAL_UART_TX_PIN GPIO_PIN_11
#define ROSERIAL_UART_GPIO_PORT GPIOA
#define ROSSERIAL_GPIO_PORT_CLK_EN __HAL_RCC_GPIOA_CLK_ENABLE
#define ROSSERIAL_UART_IRQHandler USART6_IRQHandler
#define ROSSERIAL_UART_RXDMA_IRQHandler DMA2_Stream1_IRQHandler
#define ROSSERIAL_UART_TXDMA_IRQHandler DMA2_Stream6_IRQHandler

#define HEARTBEAT_LED_PIN GPIO_PIN_13
#define HEARTBEAT_LED_PORT GPIOC
#define HEARTBEAT_LED_PORT_CLK_EN __HAL_RCC_GPIOC_CLK_ENABLE
#define STATUS_LED_PIN GPIO_PIN_0
#define STATUS_LED_PORT GPIOB
#define STATUS_LED_PORT_CLK_EN __HAL_RCC_GPIOB_CLK_ENABLE

#define STEPPER_PULSE_TIM TIM3
#define STEPPER_PULSE_TIM_CH TIM_CHANNEL_2
#define STEPPER_PULSE_TIM_HANDLER TIM3_IRQHandler
#define STEPPER_PULSE_TIM_IRQn TIM3_IRQn
#define STEPPER_PULSE_AF GPIO_AF2_TIM3
#define STEPPER_PULSE_PIN GPIO_PIN_5
#define STEPPER_PULSE_PORT GPIOB

#define STEPPER_DIR_PIN GPIO_PIN_4
#define STEPPER_DIR_PORT GPIOB

#define STEPPER_EN_PIN GPIO_PIN_3
#define STEPPER_EN_PORT GPIOB

#define STEPPER_ALM_PIN GPIO_PIN_4
#define STEPPER_ALM_PORT GPIOB

#define STEPPER_PORTS_CLK_EN __HAL_RCC_GPIOB_CLK_ENABLE
#define STEPPER_TIM_CLK_EN __HAL_RCC_TIM3_CLK_ENABLE

#define ENCODER_TIM TIM4
#define ENCODER_TIM_CLK_EN __HAL_RCC_TIM4_CLK_ENABLE
#define ENCODER_TIM_HANDLER TIM4_IRQHandler
#define ENCODER_TIM_IRQn TIM4_IRQn
#define ENCODER_CHA TIM_CHANNEL_1
#define ENCODER_CHB TIM_CHANNEL_2
#define ENCODER_A_PIN GPIO_PIN_6
#define ENCODER_B_PIN GPIO_PIN_7
#define ENCODER_AF GPIO_AF2_TIM4
#define ENCODER_A_PORT GPIOB
#define ENCODER_B_PORT GPIOB
#define ENCODER_PORTS_CLK_EN()                                                 \
    do {                                                                       \
        __HAL_RCC_GPIOB_CLK_ENABLE();                                          \
    } while (0);

#endif

void init_clocks();