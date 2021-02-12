#include <stepper.h>

constexpr uint32_t STEPS_PER_ROT = 72000;
constexpr uint32_t STEPS_PER_DEGREE = STEPS_PER_ROT / 360;
constexpr uint32_t channel_lut[5] = {TIM_CHANNEL_1, TIM_CHANNEL_2,
                                     TIM_CHANNEL_3, 0, TIM_CHANNEL_4};
uint32_t Stepper::instances_count = 0;
Stepper* Stepper::instances[MAX_STEPPER_INSTANCES];
/**
 * @brief Construct a new Stepper:: Stepper object
 *
 * @param tim_inst Pulse timer instance
 * @param cnt_tim_inst Pulse counter timer instance
 * @param tim_pulse_channel Pulse timer output channel
 * @param cnt_tim_channel Pulse counter timer compare channel
 * @param pulse_port Stepper pulse GPIO port
 * @param pulse_pin Stepper pulse GPIO pin
 * @param dir_port Stepper direction GPIO port
 * @param dir_pin Stepper direction GPIO pin
 * @param en_port Stepper enable GPIO port
 * @param en_pin Stepper enable GPIO pin
 */
Stepper::Stepper(TIM_TypeDef* tim_inst,
                 TIM_TypeDef* cnt_tim_inst,
                 uint32_t tim_pulse_channel,
                 uint32_t cnt_tim_channel,
                 uint32_t cnt_angle_channel,
                 GPIO_TypeDef* pulse_port,
                 uint32_t pulse_pin,
                 GPIO_TypeDef* dir_port,
                 uint32_t dir_pin,
                 GPIO_TypeDef* en_port,
                 uint32_t en_pin)
    : busy(false),
      step_counter(0),
      position(0),
      step(STEPS_PER_DEGREE),
      next_angle_steps(0),
      min_period(400),
      max_period(4000),
      current_period(400),
      _tim_inst(tim_inst),
      _cnt_tim_inst(cnt_tim_inst),
      _tim_pulse_channel(tim_pulse_channel),
      _cnt_tim_channel(cnt_tim_channel),
      _cnt_angle_channel(cnt_angle_channel),
      _pulse_port(pulse_port),
      _pulse_pin(pulse_pin),
      _dir_port(dir_port),
      _dir_pin(dir_pin),
      _en_port(en_port),
      _en_pin(en_pin) {
    delay = 100;
    Stepper::instances_count++;
    Stepper::instances[Stepper::instances_count - 1] = this;
}
/**
 * @brief Initialize the stepper interface, should be called after all clocks
 * are initilized. Doesn't enable clocks or interrupts
 *
 */
void Stepper::init() {
    init_gpio(_pulse_port, _pulse_pin, _dir_port, _dir_pin, _en_port, _en_pin);
    init_tim(&_tim_handle, &_cnt_tim_handle, _tim_inst, _cnt_tim_inst,
             _tim_pulse_channel, _cnt_tim_channel, _cnt_angle_channel);
}
/**
 * @brief Sets the speed of the output as pulses per second
 *
 * @param steps_per_sec
 */
void Stepper::set_speed(uint32_t steps_per_sec) {
    // FIXME this is broken and untested
    // cycles per half step
    delay = (SystemCoreClock / 2) / (steps_per_sec / 2);
    uint32_t n_cycles = delay * 2;
    // if n_cycles fits in 16 bits
    // no prescaler and period = n_cycles
    if (n_cycles < (1 << 16)) {
        _tim_handle.Init.Period = n_cycles;
        _tim_handle.Init.Prescaler = 0;
    } else {
        // if n_cycles doesn't fit in 16 bits
        // n_cycles = period * (prescaler + 1)
        _tim_handle.Init.Prescaler = (n_cycles >> 16);
        _tim_handle.Init.Period = delay / (_tim_handle.Init.Prescaler + 1);
    }
    HAL_TIM_OC_Init(&_tim_handle);
    __HAL_TIM_SetCompare(&_tim_handle, _tim_pulse_channel, delay);
    delay /= 10;
}
/**
 * @brief Bit bangs steps and dir to a stepper, should only be used for testing
 *
 * @param steps Number of pulses to generate
 * @param dir Direction of rotation
 */
void Stepper::pulse_n_bang(uint32_t steps, StepperDirection dir) {
    GPIO_InitTypeDef g;
    g.Pin = _pulse_pin;
    g.Mode = GPIO_MODE_OUTPUT_OD;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = STEPPER_PULSE_AF;
    HAL_GPIO_Init(_pulse_port, &g);
    if (dir == StepperDirection::ClockWise) {
        HAL_GPIO_WritePin(_dir_port, _dir_pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(_dir_port, _dir_pin, GPIO_PIN_RESET);
    }
    while (steps--) {
        HAL_GPIO_WritePin(_pulse_port, _pulse_pin, GPIO_PIN_RESET);
        for (uint32_t d = 0; d < delay; d++)
            __NOP();
        HAL_GPIO_WritePin(_pulse_port, _pulse_pin, GPIO_PIN_SET);
        for (uint32_t d = 0; d < delay; d++)
            __NOP();
    }
    g.Mode = GPIO_MODE_AF_PP;
    HAL_GPIO_Init(_pulse_port, &g);
}
/**
 * @brief Starts the pulse pwm timer and the pulse counting timer with compare
 * set to steps and sets the direction pin, this should be used with timer
 * callbacks to stop the timers after all pulses are finished
 *
 * @param steps Number of pulses to generate
 * @param dir Direction of rotation
 */
void Stepper::pulse_n_tim(uint32_t steps, StepperDirection dir) {
    if (dir == StepperDirection::ClockWise) {
        HAL_GPIO_WritePin(_dir_port, _dir_pin, GPIO_PIN_SET);
        step = -1 * STEPS_PER_DEGREE;
    } else {
        HAL_GPIO_WritePin(_dir_port, _dir_pin, GPIO_PIN_RESET);
        step = STEPS_PER_DEGREE;
    }
    busy = true;
    step_counter = steps;
    next_angle_steps = STEPS_PER_DEGREE + STEPS_PER_DEGREE;
    __HAL_TIM_SetCompare(&_cnt_tim_handle, _cnt_tim_channel, steps);
    __HAL_TIM_SetCompare(&_cnt_tim_handle, _cnt_angle_channel,
                         STEPS_PER_DEGREE);
    HAL_TIM_OC_Start_IT(&_cnt_tim_handle, _cnt_tim_channel);
    HAL_TIM_OC_Start_IT(&_cnt_tim_handle, _cnt_angle_channel);
    HAL_TIM_PWM_Start_IT(&_tim_handle, _tim_pulse_channel);
}
/**
 * @brief Sets the enable pin of the stepper driver
 *
 */
void Stepper::enable() {
    HAL_GPIO_WritePin(_en_port, _en_pin, GPIO_PIN_SET);
}
/**
 * @brief Resets the enable pin of the stepper driver
 *
 */
void Stepper::disable() {
    HAL_GPIO_WritePin(_en_port, _en_pin, GPIO_PIN_RESET);
}
/**
 * @brief Callback for pulse timer update events
 *
 */
void Stepper::pulse_update() {
    step_counter--;
}
/**
 * @brief Callback for pulse counter timer output compare event, stops both
 * timers
 *
 */
void Stepper::cnt_update(uint32_t channel) {
    if (channel == _cnt_tim_channel) {
        HAL_TIM_OC_Stop_IT(&_tim_handle, _tim_pulse_channel);
        HAL_TIM_OC_Stop_IT(&_cnt_tim_handle, _cnt_tim_channel);
        HAL_TIM_OC_Stop_IT(&_cnt_tim_handle, _cnt_angle_channel);
        __HAL_TIM_SetCounter(&_tim_handle, 0);
        __HAL_TIM_SetCounter(&_cnt_tim_handle, 0);
        busy = false;
    } else if (channel == _cnt_angle_channel) {
        position += step;
        __HAL_TIM_SetCompare(&_cnt_tim_handle, _cnt_angle_channel,
                             next_angle_steps);
        if (step_counter < STEPS_PER_DEGREE) {
            next_angle_steps += (STEPS_PER_DEGREE - step_counter);
            step_counter -= (STEPS_PER_DEGREE - step_counter);
        } else {
            next_angle_steps += STEPS_PER_DEGREE;
            step_counter -= STEPS_PER_DEGREE;
        }
    }
}
/**
 * @brief Initialize GPIO for generic stepper step/direction/enable interface
 *
 * @param pulse_port Pulse GPIO port
 * @param pulse_pin Pulse GPIO pin
 * @param dir_port Direction GPIO port
 * @param dir_pin Direction GPIO pin
 * @param en_port Enable GPIO port
 * @param en_pin Enable GPIO pin
 */
void Stepper::init_gpio(GPIO_TypeDef* pulse_port,
                        uint32_t pulse_pin,
                        GPIO_TypeDef* dir_port,
                        uint32_t dir_pin,
                        GPIO_TypeDef* en_port,
                        uint32_t en_pin) {
    GPIO_InitTypeDef g;
    g.Pin = pulse_pin;
    g.Mode = GPIO_MODE_AF_OD;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = STEPPER_PULSE_AF;
    HAL_GPIO_Init(pulse_port, &g);
    g.Pin = dir_pin;
    g.Mode = GPIO_MODE_OUTPUT_OD;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(dir_port, &g);
    g.Pin = en_pin;
    HAL_GPIO_Init(en_port, &g);
}
/**
 * @brief Initialize pulse timer and pulse counter timer for master slave mode,
 * this is not generic for all timers
 *
 * @param tim Pulse timer handle pointer
 * @param cnt_tim Counter timer handle pointer
 * @param tim_inst Pulse timer instance
 * @param cnt_tim_inst Counter timer instance
 * @param pulse_tim_channel Pulse timer pwm channel
 * @param cnt_tim_channel Counter timer compare channel
 */
void Stepper::init_tim(TIM_HandleTypeDef* tim,
                       TIM_HandleTypeDef* cnt_tim,
                       TIM_TypeDef* tim_inst,
                       TIM_TypeDef* cnt_tim_inst,
                       uint32_t pulse_tim_channel,
                       uint32_t cnt_tim_channel,
                       uint32_t cnt_angle_channel) {
    TIM_MasterConfigTypeDef tim_master_config = {0};
    TIM_SlaveConfigTypeDef tim_slave_config = {0};
    TIM_OC_InitTypeDef tim_oc_config = {0};

    // Initilize the base timer unit
    // defaulting to 10000 steps per second for 50MHz timer clock
    tim->Instance = tim_inst;
    tim->Init.Prescaler = 9;
    tim->Init.Period = 500;
    tim->Init.RepetitionCounter = 1;
    tim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    tim->Init.CounterMode = TIM_COUNTERMODE_UP;
    tim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(tim) != HAL_OK) {
        Error_Handler();
    }
    // Set the update callback
    // tim->PWM_PulseFinishedCallback = tim_update_handler;
    // Set the trigger output to update event (overflow), each update event
    // marks that 1 pulse has been generated
    // tim_master_config.MasterOutputTrigger = TIM_TRGO_RESET;
    tim_master_config.MasterOutputTrigger = TIM_TRGO_UPDATE;
    tim_master_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(tim, &tim_master_config) !=
        HAL_OK) {
        Error_Handler();
    }
    // Initialize pulse timer PWM channel with 50% duty cycle
    tim_oc_config.OCMode = TIM_OCMODE_PWM1;
    tim_oc_config.Pulse = 250;
    tim_oc_config.OCPolarity = TIM_OCPOLARITY_HIGH;
    tim_oc_config.OCFastMode = TIM_OCFAST_DISABLE;
    tim_oc_config.OCIdleState = TIM_OCIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(tim, &tim_oc_config, pulse_tim_channel) !=
        HAL_OK) {
        Error_Handler();
    }
    // Initilize counter timer base unit, and initialize timer for output
    // compare operation
    cnt_tim->Instance = cnt_tim_inst;
    cnt_tim->Init.Prescaler = 0;
    cnt_tim->Init.CounterMode = TIM_COUNTERMODE_UP;
    cnt_tim->Init.Period = 65535;
    cnt_tim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    cnt_tim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(cnt_tim) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_OC_Init(cnt_tim) != HAL_OK) {
        Error_Handler();
    }
    cnt_tim->OC_DelayElapsedCallback = cnt_tim_oc_handler;
    // set slave mode to external clock mode 1 and the source to ITR2
    // FIXME this is specific for TIM3 as pulse timer and TIM4 as counter timer
    tim_slave_config.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
    tim_slave_config.InputTrigger = TIM_TS_ITR2;
    if (HAL_TIM_SlaveConfigSynchro(cnt_tim, &tim_slave_config) != HAL_OK) {
        Error_Handler();
    }
    // No trigger output or master mode
    tim_master_config.MasterOutputTrigger = TIM_TRGO_RESET;
    tim_master_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(cnt_tim, &tim_master_config) !=
        HAL_OK) {
        Error_Handler();
    }
    // Initialize the output compare for the counter timer for use in pure
    // timing mode
    tim_oc_config.OCMode = TIM_OCMODE_TIMING;
    tim_oc_config.Pulse = 0;
    tim_oc_config.OCPolarity = TIM_OCPOLARITY_HIGH;
    tim_oc_config.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_OC_ConfigChannel(cnt_tim, &tim_oc_config,
                                 cnt_tim_channel | cnt_angle_channel) !=
        HAL_OK) {
        Error_Handler();
    }
}
/**
 * @brief Get the current stepper angle
 *
 * @return double
 */
double Stepper::get_angle() {
    return (double)position / STEPS_PER_DEGREE;
}
/**
 * @brief Trampoline function to call the pulse_update for each
 * stepper instance
 *
 * @param tim pointer to timer handle
 */
void Stepper::tim_update_handler(TIM_HandleTypeDef* tim) {
    for (uint32_t i = 0; i < Stepper::instances_count; i++) {
        if (&(Stepper::instances[i]->_tim_handle) == tim) {
            Stepper::instances[i]->pulse_update();
        }
    }
}
/**
 * @brief Trampoline function to call the cnt_update for each stepper instance
 *
 * @param tim
 */
void Stepper::cnt_tim_oc_handler(TIM_HandleTypeDef* tim) {
    for (uint32_t i = 0; i < Stepper::instances_count; i++) {
        if (&(Stepper::instances[i]->_cnt_tim_handle) == tim) {
            Stepper::instances[i]->cnt_update(channel_lut[tim->Channel >> 1]);
        }
    }
}