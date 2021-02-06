#include <stepper.h>

uint32_t Stepper::instances_count = 0;
Stepper *Stepper::instances[MAX_STEPPER_INSTANCES];

Stepper::Stepper(TIM_TypeDef *tim_inst, uint32_t tim_pulse_channel,
                 GPIO_TypeDef *pulse_port, uint32_t pulse_pin,
                 GPIO_TypeDef *dir_port, uint32_t dir_pin,
                 GPIO_TypeDef *en_port, uint32_t en_pin)
    : step_counter(0), min_period(400), max_period(10000),
      current_period(10000), _tim_inst(tim_inst),
      _tim_pulse_channel(tim_pulse_channel), _pulse_port(pulse_port),
      _pulse_pin(pulse_pin), _dir_port(dir_port), _dir_pin(dir_pin),
      _en_port(en_port), _en_pin(en_pin) {
    delay = 100;
    Stepper::instances_count++;
    Stepper::instances[Stepper::instances_count - 1] = this;
}

void Stepper::init() {
    init_gpio(_pulse_port, _pulse_pin, _dir_port, _dir_pin, _en_port, _en_pin);
    init_tim(&_tim_handle, _tim_inst, _tim_pulse_channel);
}
void Stepper::set_speed(uint32_t steps_per_sec) {
    // cycles per half step
    delay = SystemCoreClock / (steps_per_sec / 2);
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
    __HAL_TIM_SET_COMPARE(&_tim_handle, _tim_pulse_channel, delay);
    delay /= 10;
}
void Stepper::pulse_n_bang(uint32_t steps, StepperDirection dir) {
    GPIO_InitTypeDef g;
    g.Pin = _pulse_pin;
    g.Mode = GPIO_MODE_OUTPUT_OD;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = STEPPER_PULSE_AF;
    HAL_GPIO_Init(_pulse_port, &g);
    // TODO to be checked
    if (dir == StepperDirection::ClockWise) {
        HAL_GPIO_WritePin(_dir_port, _dir_pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(_dir_port, _dir_pin, GPIO_PIN_RESET);
    }
    while (steps--) {
        // set low
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
void Stepper::pulse_n_tim(uint32_t steps, StepperDirection dir) {
    // TODO to be checked
    if (dir == StepperDirection::ClockWise) {
        HAL_GPIO_WritePin(_dir_port, _dir_pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(_dir_port, _dir_pin, GPIO_PIN_RESET);
    }
    step_counter = steps;
    HAL_TIM_OC_Start_IT(&_tim_handle, _tim_pulse_channel);
}
void Stepper::enable() { HAL_GPIO_WritePin(_en_port, _en_pin, GPIO_PIN_SET); }
void Stepper::disable() {
    HAL_GPIO_WritePin(_en_port, _en_pin, GPIO_PIN_RESET);
}
void Stepper::pulse_update() {
    step_counter--;
    if (step_counter == 0) {
        current_period = 10000;
        __HAL_TIM_SetAutoreload(&_tim_handle, current_period);
        __HAL_TIM_SetCompare(&_tim_handle, _tim_pulse_channel,
                             current_period / 2);
        HAL_TIM_OC_Stop_IT(&_tim_handle, _tim_pulse_channel);
    } else {
        if (current_period > min_period)
            current_period -= 200;
        __HAL_TIM_SetAutoreload(&_tim_handle, current_period);
        __HAL_TIM_SetCompare(&_tim_handle, _tim_pulse_channel,
                             current_period / 2);
    }
}
void Stepper::init_gpio(GPIO_TypeDef *pulse_port, uint32_t pulse_pin,
                        GPIO_TypeDef *dir_port, uint32_t dir_pin,
                        GPIO_TypeDef *en_port, uint32_t en_pin) {

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
void Stepper::init_tim(TIM_HandleTypeDef *tim, TIM_TypeDef *tim_inst,
                       uint32_t tim_pulse_channel) {
    TIM_MasterConfigTypeDef tim_master_config = {0};
    TIM_OC_InitTypeDef tim_oc_config = {0};

    tim->Instance = tim_inst;
    tim->Init.Prescaler = 99;
    tim->Init.Period = 400;
    tim->Init.RepetitionCounter = 1;
    tim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    tim->Init.CounterMode = TIM_COUNTERMODE_UP;
    tim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(tim) != HAL_OK) {
        Error_Handler();
    }
    tim->OC_DelayElapsedCallback = tim_update_handler;
    tim_master_config.MasterOutputTrigger = TIM_TRGO_RESET;
    tim_master_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(tim, &tim_master_config) !=
        HAL_OK) {
        Error_Handler();
    }
    tim_oc_config.OCMode = TIM_OCMODE_PWM1;
    tim_oc_config.Pulse = 200;
    tim_oc_config.OCPolarity = TIM_OCPOLARITY_HIGH;
    tim_oc_config.OCFastMode = TIM_OCFAST_DISABLE;
    tim_oc_config.OCIdleState = TIM_OCIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(tim, &tim_oc_config, tim_pulse_channel) !=
        HAL_OK) {
        Error_Handler();
    }
}
// trampoline
void Stepper::tim_update_handler(TIM_HandleTypeDef *tim) {
    for (uint32_t i = 0; i < Stepper::instances_count; i++) {
        if (&(Stepper::instances[i]->_tim_handle) == tim) {
            Stepper::instances[i]->pulse_update();
        }
    }
}