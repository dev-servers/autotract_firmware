#include <app.h>

const char *steering_cmd_topic = "autotract/steering/position_cmd";
const char *steering_position_topic = "autotract/steering/position";
const char *steering_manual_topic = "autotract/steering/manual";
const char *zero_cmd_topic = "autotract/steering/zero_cmd";
const char *manual_cmd_topic = "autotract/steering/manual_cmd";

constexpr uint32_t ROS_SPIN_PERIOD = 20;         // 50 hz
constexpr uint32_t HEARTBEAT_PERIOD = 1000;      // 1 hz
constexpr uint32_t STEERING_POS_PUB_PERIOD = 20; // 50 hz
constexpr uint32_t MANUAL_PUB_PERIOD = 20;       // 50 hz
constexpr uint32_t MOTOR_TEST_PERIOD = 1000;     // 1 hz
constexpr uint32_t UART_TEST_PERIOD = 500;
double angle = 5;
uint8_t buff[1] = {'0'};
App::App()
    : steering_position_msg(), steering_manual_msg(),
      pub_steering(steering_position_topic, &steering_position_msg),
      pub_manual(steering_manual_topic, &steering_manual_msg),
      sub_steering_cmd(steering_cmd_topic, rcv_steering_cmd),
      sub_zero_cmd(zero_cmd_topic, rcv_zero_cmd),
      sub_manual_cmd(manual_cmd_topic, rcv_manual_cmd), node_handle(),
      steering(STEPPER_PULSE_TIM, STEPPER_PULSE_TIM_CH, STEPPER_PULSE_PORT,
               STEPPER_PULSE_PIN, STEPPER_DIR_PORT, STEPPER_DIR_PIN,
               STEPPER_EN_PORT, STEPPER_EN_PIN, ENCODER_TIM, ENCODER_A_PORT,
               ENCODER_A_PIN, ENCODER_B_PORT, ENCODER_B_PIN),
      ros_spin_counter(0), heartbeat_counter(0), steering_pos_pub_counter(0),
      manual_pub_counter(0) {}
void App::inc_counters() {
    ros_spin_counter++;
    heartbeat_counter++;
    steering_pos_pub_counter++;
    // motor_test_counter++;
    uart_test_counter++;
}
void App::init() {
    init_rcc();
    init_nvic();
    init_swd();
    init_heartbeat();
    steering.init();

    node_handle.advertise(pub_steering);
    node_handle.advertise(pub_manual);
    node_handle.subscribe(sub_steering_cmd);
    node_handle.subscribe(sub_zero_cmd);
    node_handle.subscribe(sub_manual_cmd);
    node_handle.initNode();
    steering.stepper.set_speed(1000);
}
void App::init_nvic() {
    // HAL_NVIC_EnableIRQ(ROSSERIAL_UART_IRQn);
    // HAL_NVIC_EnableIRQ(ROSSERIAL_UART_RXDMA_IRQn);
    // HAL_NVIC_EnableIRQ(ROSSERIAL_UART_TXDMA_IRQn);
    HAL_NVIC_EnableIRQ(STEPPER_PULSE_TIM_IRQn);
    HAL_NVIC_EnableIRQ(ENCODER_TIM_IRQn);
}
void App::init_rcc() {
    init_clocks();
    HAL_SYSTICK_Config(SystemCoreClock / 1000);
    HAL_Init();
    HEARTBEAT_LED_PORT_CLK_EN();
    STEPPER_TIM_CLK_EN();
    STEPPER_PORTS_CLK_EN();
    ENCODER_TIM_CLK_EN();
    ENCODER_PORTS_CLK_EN();
}
void App::init_swd() { SWD_PORT_CLK_EN(); }
void App::init_heartbeat() {
    GPIO_InitTypeDef GPIO_Config;
    GPIO_Config.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Config.Pull = GPIO_NOPULL;
    GPIO_Config.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_Config.Pin = HEARTBEAT_LED_PIN;
    HAL_GPIO_Init(HEARTBEAT_LED_PORT, &GPIO_Config);
}
void App::run() {
    run_heartbeat();
    // run_uart_test_dma();
    run_ros_spin();
    run_pub_steering_pos();
    run_pub_manual();
}
void App::run_motor_test() {
    if (motor_test_counter >= MOTOR_TEST_PERIOD) {
        motor_test_counter = 0;
        angle = -1 * angle;
        steering.set_angle(angle);
    }
}
void App::run_uart_test_blocking() {
    if (uart_test_counter >= UART_TEST_PERIOD) {
        uart_test_counter = 0;
        HAL_UART_Receive(&(node_handle.getHardware()->_uarth), buff, 1, 100);
        HAL_UART_Transmit(&(node_handle.getHardware()->_uarth), buff, 1, 100);
    }
}
void App::run_uart_test_it() {
    if (uart_test_counter >= UART_TEST_PERIOD) {
        uart_test_counter = 0;
        HAL_UART_Receive_IT(&(node_handle.getHardware()->_uarth), buff, 1);
        HAL_UART_Transmit_IT(&(node_handle.getHardware()->_uarth), buff, 1);
    }
}
void App::run_uart_test_dma() {
    if (uart_test_counter >= UART_TEST_PERIOD) {
        uart_test_counter = 0;
        uint8_t data[1];
        node_handle.getHardware()->reset_rbuf();
        data[0] = node_handle.getHardware()->read();
        node_handle.getHardware()->write(data, 1);
    }
}
void App::run_heartbeat() {
    if (heartbeat_counter >= HEARTBEAT_PERIOD) {
        heartbeat_counter = 0;
        HAL_GPIO_TogglePin(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN);
    }
}
void App::run_ros_spin() {
    if (ros_spin_counter >= ROS_SPIN_PERIOD) {
        ros_spin_counter = 0;
        node_handle.spinOnce();
    }
}
void App::run_pub_steering_pos() {
    if (steering_pos_pub_counter >= STEERING_POS_PUB_PERIOD) {
        steering_pos_pub_counter = 0;
        steering.get_angle();
        int64_t angle = (int64_t)(steering.angle * 100);
        steering_position_msg.data = angle;
        pub_steering.publish(&steering_position_msg);
    }
}
void App::run_pub_manual() {
    if (manual_pub_counter >= MANUAL_PUB_PERIOD) {
        manual_pub_counter = 0;
        steering_manual_msg.data = steering.manual;
        pub_manual.publish(&steering_manual_msg);
    }
}