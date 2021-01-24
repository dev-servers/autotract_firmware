#include <steering.h>

Steering::Steering(TIM_TypeDef *stepper_tim_inst,
                   uint32_t stepper_tim_pulse_channel,
                   GPIO_TypeDef *stepper_pulse_port, uint32_t stepper_pulse_pin,
                   GPIO_TypeDef *stepper_dir_port, uint32_t stepper_dir_pin,
                   GPIO_TypeDef *stepper_en_port, uint32_t stepper_en_pin,
                   TIM_TypeDef *encoder_tim_inst, GPIO_TypeDef *encoder_a_port,
                   uint32_t encoder_a_pin, GPIO_TypeDef *encoder_b_port,
                   uint32_t encoder_b_pin)
    : angle(0), stepper(stepper_tim_inst, stepper_tim_pulse_channel,
                        stepper_pulse_port, stepper_pulse_pin, stepper_dir_port,
                        stepper_dir_pin, stepper_en_port, stepper_en_pin),
      encoder(encoder_tim_inst, encoder_a_port, encoder_a_pin, encoder_b_port,
              encoder_b_pin) {}
void Steering::init() {
    stepper.init();
    encoder.init();
}