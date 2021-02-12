#pragma once
#include <stm32f4xx_hal.h>
#include <errors.h>
#include <hardware.h>

enum class StepperDirection : uint8_t {
    ClockWise,
    CounterClockWise,
};

struct Stepper {
   public:
    static Stepper* instances[MAX_STEPPER_INSTANCES];
    static uint32_t instances_count;
    Stepper(TIM_TypeDef* tim_inst,
            TIM_TypeDef* cnt_tim_inst,
            uint32_t tim_pulse_channel,
            uint32_t cnt_tim_channel,
            uint32_t cnt_angle_channel,
            GPIO_TypeDef* pulse_port,
            uint32_t pulse_pin,
            GPIO_TypeDef* dir_port,
            uint32_t dir_pin,
            GPIO_TypeDef* en_port,
            uint32_t en_pin);
    void init();
    void set_speed(uint32_t steps_per_sec);
    void pulse_n_bang(uint32_t steps, StepperDirection dir);
    void pulse_n_tim(uint32_t steps, StepperDirection dir);
    void enable();
    void disable();
    double get_angle();

    static void init_gpio(GPIO_TypeDef* pulse_port,
                          uint32_t pulse_pin,
                          GPIO_TypeDef* dir_port,
                          uint32_t dir_pin,
                          GPIO_TypeDef* en_port,
                          uint32_t en_pin);
    static void init_tim(TIM_HandleTypeDef* tim,
                         TIM_HandleTypeDef* cnt_tim,
                         TIM_TypeDef* tim_inst,
                         TIM_TypeDef* cnt_tim_inst,
                         uint32_t pulse_tim_channel,
                         uint32_t cnt_tim_channel,
                         uint32_t cnt_angle_channel);
    TIM_HandleTypeDef _tim_handle;
    TIM_HandleTypeDef _cnt_tim_handle;
    bool busy;

   private:
    static void tim_update_handler(TIM_HandleTypeDef* tim);
    static void cnt_tim_oc_handler(TIM_HandleTypeDef* tim);
    void pulse_update();
    void cnt_update(uint32_t channel);
    int step_counter;
    int position;
    int step;
    int next_angle_steps;
    uint32_t min_period;
    uint32_t max_period;
    uint32_t current_period;
    TIM_TypeDef* _tim_inst;
    TIM_TypeDef* _cnt_tim_inst;
    uint32_t _tim_pulse_channel;
    uint32_t _cnt_tim_channel;
    uint32_t _cnt_angle_channel;
    GPIO_TypeDef* _pulse_port;
    uint32_t _pulse_pin;
    GPIO_TypeDef* _dir_port;
    uint32_t _dir_pin;
    GPIO_TypeDef* _en_port;
    uint32_t _en_pin;
    uint32_t delay;
};
