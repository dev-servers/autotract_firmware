#include <steering.h>

// number of stepper steps per half a turn
constexpr double ENCODER_STEPS_PER_PI = 500;
constexpr double STEPPER_STEPS_PER_PI = 36000;
constexpr double STEPPER_STEPS_PER_DEGREE = STEPPER_STEPS_PER_PI / 180;
constexpr double STEPPER_STEPS_PER_ENCODER_STEPS =
    STEPPER_STEPS_PER_PI / ENCODER_STEPS_PER_PI;

/**
 * @brief Construct a new Steering:: Steering object
 *
 * @param stepper_tim_inst
 * @param stepper_cnt_tim_inst
 * @param stepper_tim_pulse_channel
 * @param stepper_cnt_tim_channel
 * @param stepper_cnt_angle_tim_channel
 * @param stepper_pulse_port
 * @param stepper_pulse_pin
 * @param stepper_dir_port
 * @param stepper_dir_pin
 * @param stepper_en_port
 * @param stepper_en_pin
 * @param encoder_tim_inst
 * @param encoder_a_port
 * @param encoder_a_pin
 * @param encoder_b_port
 * @param encoder_b_pin
 */
Steering::Steering(TIM_TypeDef* stepper_tim_inst,
                   TIM_TypeDef* stepper_cnt_tim_inst,
                   uint32_t stepper_tim_pulse_channel,
                   uint32_t stepper_cnt_tim_channel,
                   uint32_t stepper_cnt_angle_tim_channel,
                   GPIO_TypeDef* stepper_pulse_port,
                   uint32_t stepper_pulse_pin,
                   GPIO_TypeDef* stepper_dir_port,
                   uint32_t stepper_dir_pin,
                   GPIO_TypeDef* stepper_en_port,
                   uint32_t stepper_en_pin,
                   TIM_TypeDef* encoder_tim_inst,
                   GPIO_TypeDef* encoder_a_port,
                   uint32_t encoder_a_pin,
                   GPIO_TypeDef* encoder_b_port,
                   uint32_t encoder_b_pin)
    : angle(0),
      manual(true),
      stepper(stepper_tim_inst,
              stepper_cnt_tim_inst,
              stepper_tim_pulse_channel,
              stepper_cnt_tim_channel,
              stepper_cnt_angle_tim_channel,
              stepper_pulse_port,
              stepper_pulse_pin,
              stepper_dir_port,
              stepper_dir_pin,
              stepper_en_port,
              stepper_en_pin),
      encoder(encoder_tim_inst,
              encoder_a_port,
              encoder_a_pin,
              encoder_b_port,
              encoder_b_pin) {}

/**
 * @brief
 *
 */
void Steering::init() {
    stepper.init();
    set_manual(true);
    // encoder.init();
}
/**
 * @brief
 *
 * @param new_angle
 */
void Steering::set_angle(double new_angle) {
    if (!manual && !stepper.busy) {
        double delta_angle = (new_angle - angle) / 180;
        int32_t target_steps = delta_angle * STEPPER_STEPS_PER_PI;
        if (delta_angle < 0) {
            uint32_t steps = -1 * target_steps;
            stepper.pulse_n_tim(steps, StepperDirection::CounterClockWise);
        } else if (delta_angle > 0) {
            uint32_t steps = target_steps;
            stepper.pulse_n_tim(steps, StepperDirection::ClockWise);
        }
        angle = new_angle;
    }
}
/**
 * @brief
 *
 * @param man
 */
void Steering::set_manual(bool man) {
    manual = man;
    if (manual)
        stepper.disable();
    else
        stepper.enable();
}
/**
 * @brief
 *
 */
void Steering::get_angle() {
    // angle = stepper.get_angle();
    // int32_t current_steps = encoder.get_steps();
    // angle = (double)current_steps / ENCODER_STEPS_PER_PI;
}
/**
 * @brief
 *
 */
void Steering::zero_angle() {
    encoder.set_zero();
    angle = 0;
}