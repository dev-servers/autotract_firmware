#pragma once
#include <cstdio>
#include <init.h>
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int64.h>
#include <std_msgs/String.h>
#include <stm32f4xx_hal.h>

/* Target Definitions
    Application specific definitation for the target board. add a definition in
   the CMakeLists.txt file with the unique board name and use that in an ifdef
   block to define board specific directives start with board number so it's
   easy to check that the uploading firmware is the correct one while debugging,
   board number is constexpr to show up in the debugger
*/
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

#define STEPPER_STEP_TIM TIM1
#define STEPPER_STEP_TIM_CH TIM_CHANNEL_3
#define STEPPER_STEP_PIN GPIO_PIN_10
#define STEPPER_STEP_PORT GPIOA

#define STEPPER_DIR_PIN GPIO_PIN_9
#define STEPPER_DIR_PORT GPIOB

#define STEPPER_EN_PIN GPIO_PIN_5
#define STEPPER_EN_PORT GPIOB

#define STEPPER_ALM_PIN GPIO_PIN_4
#define STEPPER_ALM_PORT GPIOB

#define STEPPER_PORTS_CLK_EN()                                                 \
    do {                                                                       \
        __HAL_RCC_GPIOA_CLK_ENABLE();                                          \
        __HAL_RCC_GPIOB_CLK_ENABLE();                                          \
    } while (0);

#define ENCODER_TIM TIM1
#define ENCODER_CHA TIM_CHANNEL_1
#define ENCODER_CHB TIM_CHANNEL_2
#define ENCODER_A_PIN GPIO_PIN_15
#define ENCODER_B_PIN GPIO_PIN_3
#define ENCODER_A_PORT GPIOA
#define ENCODER_B_PORT GPIOB
#define ENCODER_PORTS_CLK_EN()                                                 \
    do {                                                                       \
        __HAL_RCC_GPIOA_CLK_ENABLE();                                          \
        __HAL_RCC_GPIOB_CLK_ENABLE();                                          \
    } while (0);

#endif

#ifdef BLACKPILL
constexpr uint32_t BOARD = 1;
#define SWD_PORT_CLK_EN __HAL_RCC_GPIOA_CLK_ENABLE
#define ROSSERIAL_UART USART1
#define ROSSERIAL_UART_CLK_EN __HAL_RCC_USART1_CLK_ENABLE
#define ROSSERIAL_UART_IRQn USART1_IRQn
#define ROSSERIAL_UART_RXDMA_STREAM DMA2_Stream2
#define ROSSERIAL_UART_TXDMA_STREAM DMA2_Stream7
#define ROSSERIAL_UART_RXDMA_CHANNEL DMA_CHANNEL_4
#define ROSSERIAL_UART_TXDMA_CHANNEL DMA_CHANNEL_4
#define ROSSERIAL_UART_RXDMA_IRQn DMA2_Stream2_IRQn
#define ROSSERIAL_UART_TXDMA_IRQn DMA2_Stream7_IRQn
#define ROSSERIAL_UART_DMA_CLK_EN __HAL_RCC_DMA2_CLK_ENABLE
#define ROSSERUAL_UART_PIN_AF GPIO_AF7_USART1
#define ROSSERIAL_UART_RX_PIN GPIO_PIN_7
#define ROSSERIAL_UART_TX_PIN GPIO_PIN_6
#define ROSERIAL_UART_GPIO_PORT GPIOB
#define ROSSERIAL_GPIO_PORT_CLK_EN __HAL_RCC_GPIOB_CLK_ENABLE
#define ROSSERIAL_UART_IRQHandler USART1_IRQHandler
#define ROSSERIAL_UART_RXDMA_IRQHandler DMA2_Stream2_IRQHandler
#define ROSSERIAL_UART_TXDMA_IRQHandler DMA2_Stream7_IRQHandler

#define HEARTBEAT_LED_PIN GPIO_PIN_13
#define HEARTBEAT_LED_PORT GPIOC
#define HEARTBEAT_LED_PORT_CLK_EN __HAL_RCC_GPIOC_CLK_ENABLE
#define STATUS_LED_PIN GPIO_PIN_0
#define STATUS_LED_PORT GPIOB
#define STATUS_LED_PORT_CLK_EN __HAL_RCC_GPIOB_CLK_ENABLE

#define STEPPER_STEP_PIN GPIO_PIN_8
#define STEPPER_STEP_PORT GPIOB

#define STEPPER_DIR_PIN GPIO_PIN_9
#define STEPPER_DIR_PORT GPIOB

#define STEPPER_EN_PIN GPIO_PIN_5
#define STEPPER_EN_PORT GPIOB

#define STEPPER_ALM_PIN GPIO_PIN_4
#define STEPPER_ALM_PORT GPIOB

#define STEPPER_PORTS_CLK_EN __HAL_RCC_GPIOB_CLK_ENABLE

#define ENCODER_TIM TIM1
#define ENCODER_CHA TIM_CHANNEL_1
#define ENCODER_CHB TIM_CHANNEL_2
#define ENCODER_A_PIN GPIO_PIN_15
#define ENCODER_B_PIN GPIO_PIN_3
#define ENCODER_A_PORT GPIOA
#define ENCODER_B_PORT GPIOB
#define ENCODER_PORTS_CLK_EN()                                                 \
    do {                                                                       \
        __HAL_RCC_GPIOA_CLK_ENABLE();                                          \
        __HAL_RCC_GPIOB_CLK_ENABLE();                                          \
    } while (0);

#endif

/*
    ROS subsriber callbacks declrations
*/
void rcv_steering_cmd(const std_msgs::Int64 &new_steering_angle);
void rcv_zero_cmd(const std_msgs::Empty &zero);
void rcv_manual_cmd(const std_msgs::Bool &manual);
/*
Application pseudo thread counters, just a neat way to define and control the
super loop delays and counters
*/

constexpr uint32_t ROS_SPIN_PERIOD = 20;    // 50 hz
constexpr uint32_t HEARTBEAT_PERIOD = 1000; // 1 hz
struct AppThreadsCounters {
  public:
    uint32_t ros_spin_counter;
    uint32_t heartbeat_counter;
    AppThreadsCounters() {
        ros_spin_counter = 0;
        heartbeat_counter = 0;
    }
    void inc_counters() {
        ros_spin_counter++;
        heartbeat_counter++;
    }
};
