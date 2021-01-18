#include <main.h>

UART_HandleTypeDef rosserial_handle;
DMA_HandleTypeDef rxdma;
DMA_HandleTypeDef txdma;
ros::NodeHandle nh;

std_msgs::Int64 steering_position_msg;
ros::Publisher pub_steering("autotract/steering/position",
                            &steering_position_msg);
ros::Subscriber<std_msgs::Int64>
    sub_steering_cmd("autotract/steering/position_cmd", rcv_steering_cmd);
ros::Subscriber<std_msgs::Empty> sub_zero_cmd("autotract/steering/zero_cmd",
                                              rcv_zero_cmd);
ros::Subscriber<std_msgs::Bool> sub_manual_cmd("autotract/steering/manual_cmd",
                                               rcv_manual_cmd);
AppThreadsCounters app;
uint32_t count = 0;
int main(void) {
    HAL_Init();
    init_clocks();
    HAL_SYSTICK_Config(SystemCoreClock / 1000);
    init_gpio();
    init_uart(&rosserial_handle, &rxdma, &txdma, ROSSERIAL_UART,
              ROSSERIAL_UART_RXDMA_STREAM, ROSSERIAL_UART_TXDMA_STREAM,
              ROSSERIAL_UART_RXDMA_CHANNEL, ROSSERIAL_UART_TXDMA_CHANNEL);
    nh.initNode();
    nh.advertise(pub_steering);
    nh.subscribe(sub_steering_cmd);
    nh.subscribe(sub_zero_cmd);
    nh.subscribe(sub_manual_cmd);
    for (;;) {
        if (app.ros_spin_counter >= ROS_SPIN_PERIOD) {
            app.ros_spin_counter = 0;
            steering_position_msg.data = count;
            pub_steering.publish(&steering_position_msg);
            nh.spinOnce();
            count++;
        }
        if (app.heartbeat_counter >= HEARTBEAT_PERIOD) {
            app.heartbeat_counter = 0;
            HAL_GPIO_TogglePin(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN);
        }
    }
    return 0;
}

void rcv_steering_cmd(const std_msgs::Int64 &new_steering_angle) {}
void rcv_zero_cmd(const std_msgs::Empty &zero) { count = 0; }
void rcv_manual_cmd(const std_msgs::Bool &manual) {}