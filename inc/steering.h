#pragma once

#include <stm32f4xx_hal.h>
#include <encoder.h>
#include <stepper.h>

#define STEPS_PER_REV 1000

struct Steering {
  public:
    Steering(TIM_TypeDef *stepper_tim_inst, uint32_t stepper_tim_pulse_channel,
             GPIO_TypeDef *stepper_pulse_port, uint32_t stepper_pulse_pin,
             GPIO_TypeDef *stepper_dir_port, uint32_t stepper_dir_pin,
             GPIO_TypeDef *stepper_en_port, uint32_t stepper_en_pin,
             TIM_TypeDef *encoder_tim_inst, GPIO_TypeDef *encoder_a_port,
             uint32_t encoder_a_pin, GPIO_TypeDef *encoder_b_port,
             uint32_t encoder_b_pin);
    void init();
    void set_angle(double angle);
    void get_angle();
    void zero_angle();
    void set_manual(bool man);
    uint32_t angle;
    bool manual;
    Stepper stepper;
    Encoder encoder;
};