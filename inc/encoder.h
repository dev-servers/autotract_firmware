#pragma once
#include <stm32f4xx_hal.h>
#include <errors.h>
#include <hardware.h>
struct Encoder {
  public:
    Encoder(TIM_TypeDef *tim_inst, GPIO_TypeDef *a_port, uint32_t a_pin,
            GPIO_TypeDef *b_port, uint32_t b_pin);
    void init();
    int32_t get_steps();
    void set_zero();
    static void init_tim(TIM_HandleTypeDef *tim, TIM_TypeDef *tim_inst);
    static void init_gpio(GPIO_TypeDef *a_port, uint32_t a_pin,
                          GPIO_TypeDef *b_port, uint32_t b_pin);
    TIM_HandleTypeDef _tim_handle;

  private:
    static void tim_update_handler(TIM_HandleTypeDef *htim);
    TIM_TypeDef *_tim_inst;
    GPIO_TypeDef *_a_port;
    uint32_t _a_pin;
    GPIO_TypeDef *_b_port;
    uint32_t _b_pin;
};
