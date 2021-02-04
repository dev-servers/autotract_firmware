#include <steering.h>

// number of stepper steps per half a turn
constexpr double ENCODER_STEPS_PER_PI = 500;
constexpr double STEPPER_STEPS_PER_PI = 500;
constexpr double STEPPER_STEPS_PER_ENCODER_STEPS =
    STEPPER_STEPS_PER_PI / ENCODER_STEPS_PER_PI;

Steering::Steering(TIM_TypeDef *stepper_tim_inst,
                   uint32_t stepper_tim_pulse_channel,
                   GPIO_TypeDef *stepper_pulse_port, uint32_t stepper_pulse_pin,
                   GPIO_TypeDef *stepper_dir_port, uint32_t stepper_dir_pin,
                   GPIO_TypeDef *stepper_en_port, uint32_t stepper_en_pin,
                   TIM_TypeDef *encoder_tim_inst, GPIO_TypeDef *encoder_a_port,
                   uint32_t encoder_a_pin, GPIO_TypeDef *encoder_b_port,
                   uint32_t encoder_b_pin)
    : angle(0), manual(true),
      stepper(stepper_tim_inst, stepper_tim_pulse_channel, stepper_pulse_port,
              stepper_pulse_pin, stepper_dir_port, stepper_dir_pin,
              stepper_en_port, stepper_en_pin),
      encoder(encoder_tim_inst, encoder_a_port, encoder_a_pin, encoder_b_port,
              encoder_b_pin) {}
void Steering::init() {
    stepper.init();
    set_manual(true);
    encoder.init();
}
void Steering::set_angle(double new_angle) {
    double delta_angle = new_angle - angle;
    int32_t target_steps =
        delta_angle * STEPPER_STEPS_PER_PI * STEPPER_STEPS_PER_ENCODER_STEPS;
    if (delta_angle < 0) {
        uint32_t steps = -1 * target_steps;
        stepper.pulse_n_tim(steps, StepperDirection::CounterClockWise);
    } else {
        uint32_t steps = target_steps;
        stepper.pulse_n_tim(steps, StepperDirection::ClockWise);
    }
    angle = new_angle;
}
void Steering::set_manual(bool man) {
    manual = man;
    if (manual)
        stepper.disable();
    else
        stepper.enable();
}
void Steering::get_angle() {
    // int32_t current_steps = encoder.get_steps();
    // angle = (double)current_steps / ENCODER_STEPS_PER_PI;
}
void Steering::zero_angle() {
    encoder.set_zero();
    angle = 0;
}