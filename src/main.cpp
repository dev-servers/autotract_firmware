#include <main.h>

char hello[15] = "hello world!\r\n";
uint8_t rbuf[5];
UART_HandleTypeDef rosserial_handle;
DMA_HandleTypeDef rxdma;
DMA_HandleTypeDef txdma;
ros::NodeHandle nh;
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

int main(void)
{
    int board = BOARD;
    HAL_Init();
    init_clocks();
    HAL_SYSTICK_Config(SystemCoreClock / 1000);
    init_gpio();
    init_uart(&rosserial_handle, &rxdma, &txdma, ROSSERIAL_UART, ROSSERIAL_UART_RXDMA_STREAM,
              ROSSERIAL_UART_TXDMA_STREAM, ROSSERIAL_UART_RXDMA_CHANNEL, ROSSERIAL_UART_TXDMA_CHANNEL);
    nh.initNode();
    nh.advertise(chatter);
    for (;;)
    {
        if ((HAL_GetTick() % 500) == 0)
        {
            HAL_GPIO_TogglePin(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN);
            HAL_UART_Receive(&rosserial_handle, rbuf, 1, 400);
            HAL_UART_Transmit_IT(&rosserial_handle, rbuf, 1);
            // str_msg.data = hello;
            // chatter.publish(&str_msg);
        }
        nh.spinOnce();
        HAL_Delay(1);
    }
    return 0;
}