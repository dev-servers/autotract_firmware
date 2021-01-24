#include <encoder.h>

Encoder::Encoder(TIM_TypeDef *tim_inst, GPIO_TypeDef *a_port, uint32_t a_pin,
                 GPIO_TypeDef *b_port, uint32_t b_pin)
    : _tim_inst(tim_inst), _a_port(a_port), _a_pin(a_pin), _b_port(b_port),
      _b_pin(b_pin) {}
void Encoder::init() {
    init_gpio(_a_port, _a_pin, _b_port, _b_pin);
    init_tim(&_tim_handle, _tim_inst);
    set_zero();
}
int32_t Encoder::get_steps() {
    uint16_t zero_offset = _tim_handle.Init.Period >> 1;
    int32_t cnt = __HAL_TIM_GetCounter(&_tim_handle);
    return cnt - zero_offset;
}
void Encoder::set_zero() {
    // the assumption is that the zero value is in the middle of the period
    // to allow for left and right turns of the steering to corrospond to
    // positive and negative deltas in the steps
    uint16_t zero_offset = _tim_handle.Init.Period >> 1;
    __HAL_TIM_SetCounter(&_tim_handle, zero_offset);
}
void Encoder::init_tim(TIM_HandleTypeDef *tim, TIM_TypeDef *tim_inst) {
    TIM_Encoder_InitTypeDef tim_encoder_config = {0};
    TIM_MasterConfigTypeDef tim_master_config = {0};

    tim->Instance = tim_inst;
    tim->Init.Prescaler = 0;
    tim->Init.CounterMode = TIM_COUNTERMODE_UP;
    tim->Init.Period = 65535;
    tim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    tim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    tim_encoder_config.EncoderMode = TIM_ENCODERMODE_TI12;
    tim_encoder_config.IC1Polarity = TIM_ENCODERINPUTPOLARITY_RISING;
    tim_encoder_config.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    tim_encoder_config.IC1Prescaler = TIM_ICPSC_DIV1;
    tim_encoder_config.IC1Filter = 0;
    tim_encoder_config.IC2Polarity = TIM_ENCODERINPUTPOLARITY_RISING;
    tim_encoder_config.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    tim_encoder_config.IC2Prescaler = TIM_ICPSC_DIV1;
    tim_encoder_config.IC2Filter = 0;
    if (HAL_TIM_Encoder_Init(tim, &tim_encoder_config) != HAL_OK) {
        Error_Handler();
    }
    tim_master_config.MasterOutputTrigger = TIM_TRGO_RESET;
    tim_master_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(tim, &tim_master_config) !=
        HAL_OK) {
        Error_Handler();
    }
    HAL_TIM_Encoder_Start_IT(tim, TIM_CHANNEL_ALL);
}
void Encoder::init_gpio(GPIO_TypeDef *a_port, uint32_t a_pin,
                        GPIO_TypeDef *b_port, uint32_t b_pin) {
    GPIO_InitTypeDef g = {0};
    g.Pin = a_pin;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Alternate = ENCODER_AF;
    HAL_GPIO_Init(a_port, &g);
    g.Pin = b_pin;
    HAL_GPIO_Init(b_port, &g);
}