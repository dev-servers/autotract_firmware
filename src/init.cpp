#include <init.h>

void init_uart(UART_HandleTypeDef *handle, DMA_HandleTypeDef *rxdma,
               DMA_HandleTypeDef *txdma, USART_TypeDef *usart_inst,
               DMA_Stream_TypeDef *rxdma_inst, DMA_Stream_TypeDef *txdma_inst,
               uint32_t rx_channel, uint32_t tx_channel)
{

    assert_param(IS_DMA_CHANNEL(rx_channel));
    assert_param(IS_DMA_CHANNEL(tx_channel));
    assert_param(IS_DMA_STREAM_ALL_INSTANCE(rxdma_inst));
    assert_param(IS_DMA_STREAM_ALL_INSTANCE(txdma_inst));

    // enabled clocks
    ROSSERIAL_UART_CLK_EN();
    ROSSERIAL_UART_DMA_CLK_EN();
    ROSSERIAL_GPIO_PORT_CLK_EN();

    // init rosserial uart pins
    GPIO_InitTypeDef GPIO_Config;
    GPIO_Config.Mode = GPIO_MODE_AF_PP;
    GPIO_Config.Pull = GPIO_NOPULL;
    GPIO_Config.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_Config.Alternate = ROSSERUAL_UART_PIN_AF;
    GPIO_Config.Pin = ROSSERIAL_UART_RX_PIN | ROSSERIAL_UART_TX_PIN;
    HAL_GPIO_Init(ROSERIAL_UART_GPIO_PORT, &GPIO_Config);

    // RX DMA Channel config
    rxdma->Instance = rxdma_inst;
    rxdma->Init.Channel = rx_channel;
    rxdma->Init.Direction = DMA_PERIPH_TO_MEMORY;
    rxdma->Init.Mode = DMA_CIRCULAR;
    rxdma->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    rxdma->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    rxdma->Init.MemInc = DMA_MINC_ENABLE;
    rxdma->Init.PeriphInc = DMA_PINC_DISABLE;
    rxdma->Parent = handle;

    // TX DMA Channel config
    txdma->Instance = txdma_inst;
    txdma->Init.Channel = tx_channel;
    txdma->Init.Direction = DMA_MEMORY_TO_PERIPH;
    txdma->Init.Mode = DMA_NORMAL;
    txdma->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    txdma->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    txdma->Init.MemInc = DMA_MINC_ENABLE;
    txdma->Init.PeriphInc = DMA_PINC_DISABLE;
    txdma->Parent = handle;

    // USART config
    handle->Init.BaudRate = 57600;
    handle->Init.WordLength = UART_WORDLENGTH_8B;
    handle->Init.Mode = UART_MODE_TX_RX;
    handle->Init.Parity = UART_PARITY_NONE;
    handle->Init.StopBits = UART_STOPBITS_1;
    handle->Init.WordLength = UART_WORDLENGTH_8B;
    handle->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    handle->Init.OverSampling = UART_OVERSAMPLING_16;
    handle->hdmarx = rxdma;
    handle->hdmatx = txdma;
    handle->Instance = usart_inst;

    // Init all
    volatile HAL_StatusTypeDef s;
    s = HAL_DMA_Init(rxdma);
    if (s != HAL_OK)
    {
        Error_Handler();
    }
    s = HAL_DMA_Init(txdma);
    if (s != HAL_OK)
    {
        Error_Handler();
    }
    s = HAL_UART_Init(handle);
    if (s != HAL_OK)
    {
        Error_Handler();
    }
    HAL_NVIC_SetPriority(ROSSERIAL_UART_RXDMA_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ROSSERIAL_UART_RXDMA_IRQn);
    HAL_NVIC_SetPriority(ROSSERIAL_UART_TXDMA_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(ROSSERIAL_UART_TXDMA_IRQn);
    HAL_NVIC_SetPriority(ROSSERIAL_UART_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(ROSSERIAL_UART_IRQn);
}

void init_gpio()
{
    GPIO_InitTypeDef GPIO_Config;
    // Init SWD
    SWD_PORT_CLK_EN();
    // Init HEARTBEAT LED
    GPIO_Config.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Config.Pull = GPIO_NOPULL;
    GPIO_Config.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_Config.Pin = HEARTBEAT_LED_PIN;
    HEARTBEAT_LED_PORT_CLK_EN();
    HAL_GPIO_Init(HEARTBEAT_LED_PORT, &GPIO_Config);
    // Init STATUS LED
    GPIO_Config.Pin = STATUS_LED_PIN;
    HEARTBEAT_LED_PORT_CLK_EN();
    HAL_GPIO_Init(STATUS_LED_PORT, &GPIO_Config);
}
#ifdef NUCLEO
void init_clocks()
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
     */
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 200;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
    {
        Error_Handler();
    }
}
#endif
#ifdef BLACKPILL
void init_clocks()
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
     */
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 25;
    RCC_OscInitStruct.PLL.PLLN = 200;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
    {
        Error_Handler();
    }
}
#endif