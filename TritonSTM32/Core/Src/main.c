#include "main.h"
#include "stdbool.h"

#include "bno055_stm32.h"

#include "cJSON.h"

// System Handles
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
UART_HandleTypeDef huart3;
PCD_HandleTypeDef hpcd_USB_FS;

// System Function Prototypes
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_TIM1_Init(void);

// Custom Function Prototypes
void clear_buffer(uint8_t *buffer, uint32_t size);

// LEDs
const GPIO_TypeDef *LED_PORT[4] = {GPIOA, GPIOA, GPIOA, GPIOB};
const uint16_t LED_PIN[4] = {GPIO_PIN_10, GPIO_PIN_9, GPIO_PIN_8, GPIO_PIN_15};

// UART Custom Variables
#define BUFFER_SIZE 1024
volatile uint8_t UART3_RX_buf[BUFFER_SIZE] = {0};
volatile int8_t UART3_RX_byte = 0;
volatile uint32_t RX_index = 0;

// ESC Custom Variables (Timer 2 at 50Hz)
const uint32_t ESC_CHANNELS[4] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4};
volatile uint32_t ESC_Values[4] = {1500, 1500, 1500, 1500};
volatile uint32_t ESC_Prev_Values[4] = {1500, 1500, 1500, 1500};

// SSR Custom Variables (Timer 4 at 50Hz)
const uint32_t SSR_CHANNELS[2] = {TIM_CHANNEL_1, TIM_CHANNEL_2};
volatile uint8_t SSR_enabled[2] = {0, 0};
double SST_duty_cycle[2] = {0.0, 0.0};

// RC Custom Variables (Input Capture from RC Receiver) (Timer 3)
const uint32_t RC_CHANNELS[4] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4};
volatile uint32_t RC_Values[4] = {1500, 1500, 1500, 1500};
volatile uint8_t RC_mode = 0;

// BNO055 Custom Variables
volatile uint8_t BNO055_requested = 0;
volatile uint32_t imu_poll_timer = 0;
const double imu_poll_period = 1000;
bno055_vector_t DirVector;
bno055_vector_t AccelVector;
bno055_vector_t GyroVector;
bno055_vector_t MagVector;

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  MX_USB_PCD_Init();

  // Enable Timers
  __HAL_RCC_TIM1_CLK_ENABLE();
  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();
  __HAL_RCC_TIM4_CLK_ENABLE();

  // Enable BNO055
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

  // Initialize ESC PWM channels
  HAL_TIM_PWM_Start(&htim2, ESC_CHANNELS[0]);
  HAL_TIM_PWM_Start(&htim2, ESC_CHANNELS[1]);
  HAL_TIM_PWM_Start(&htim2, ESC_CHANNELS[2]);
  HAL_TIM_PWM_Start(&htim2, ESC_CHANNELS[3]);

  __HAL_TIM_SET_COMPARE(&htim2, ESC_CHANNELS[0], 2000);
  __HAL_TIM_SET_COMPARE(&htim2, ESC_CHANNELS[1], 2000);
  __HAL_TIM_SET_COMPARE(&htim2, ESC_CHANNELS[2], 2000);
  __HAL_TIM_SET_COMPARE(&htim2, ESC_CHANNELS[3], 2000);
  HAL_Delay(1000);
  __HAL_TIM_SET_COMPARE(&htim2, ESC_CHANNELS[0], 1000);
  __HAL_TIM_SET_COMPARE(&htim2, ESC_CHANNELS[1], 1000);
  __HAL_TIM_SET_COMPARE(&htim2, ESC_CHANNELS[2], 1000);
  __HAL_TIM_SET_COMPARE(&htim2, ESC_CHANNELS[3], 1000);
  HAL_Delay(1000);
  __HAL_TIM_SET_COMPARE(&htim2, ESC_CHANNELS[0], 1500);
  __HAL_TIM_SET_COMPARE(&htim2, ESC_CHANNELS[1], 1500);
  __HAL_TIM_SET_COMPARE(&htim2, ESC_CHANNELS[2], 1500);
  __HAL_TIM_SET_COMPARE(&htim2, ESC_CHANNELS[3], 1500);

  // Initialize SSR PWM channels
  HAL_TIM_PWM_Start(&htim4, SSR_CHANNELS[0]);
  HAL_TIM_PWM_Start(&htim4, SSR_CHANNELS[1]);

  __HAL_TIM_SET_COMPARE(&htim4, SSR_CHANNELS[0], 0);
  __HAL_TIM_SET_COMPARE(&htim4, SSR_CHANNELS[1], 0);

  // Initialize RC Input Capture channels
  HAL_TIM_IC_Start_IT(&htim3, RC_CHANNELS[0]);
  HAL_TIM_IC_Start_IT(&htim3, RC_CHANNELS[1]);
  HAL_TIM_IC_Start_IT(&htim3, RC_CHANNELS[2]);
  HAL_TIM_IC_Start_IT(&htim3, RC_CHANNELS[3]);

  // Initialize BNO055
  bno055_assignI2C(&hi2c1);
  bno055_setup();
  bno055_setOperationModeNDOF();

  // Initialize UART
  clear_buffer(UART3_RX_buf, BUFFER_SIZE);
  HAL_UART_Receive_IT(&huart3, &UART3_RX_byte, 1);

  while (1)
  {
    if (RC_mode) // Write RC channel values if in RC mode
    {
      __HAL_TIM_SET_COMPARE(&htim2, RC_CHANNELS[0], RC_Values[0]);
      __HAL_TIM_SET_COMPARE(&htim2, RC_CHANNELS[1], RC_Values[1]);
      __HAL_TIM_SET_COMPARE(&htim2, RC_CHANNELS[2], RC_Values[2]);
      __HAL_TIM_SET_COMPARE(&htim2, RC_CHANNELS[3], RC_Values[3]);
      HAL_GPIO_WritePin(LED_PORT[0], LED_PIN[0], GPIO_PIN_SET);
      HAL_GPIO_WritePin(LED_PORT[1], LED_PIN[1], GPIO_PIN_RESET);
    }
    else
    {
      for (int i = 0; i < 4; i++) // Write ESC values if not in RC mode
      {
        if (ESC_Prev_Values[i] != ESC_Values[i])
        {
          __HAL_TIM_SET_COMPARE(&htim2, ESC_CHANNELS[i], ESC_Values[i]);
          ESC_Prev_Values[i] = ESC_Values[i];
        }
      }
      HAL_GPIO_WritePin(LED_PORT[0], LED_PIN[0], GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED_PORT[1], LED_PIN[1], GPIO_PIN_SET);
    }

    if (HAL_GetTick() - imu_poll_timer > imu_poll_period) // Poll BNO055 at 1/imu_poll_period frequency
    {
      imu_poll_timer = HAL_GetTick();
      DirVector = bno055_getVectorEuler();
      AccelVector = bno055_getVectorAccelerometer();
      GyroVector = bno055_getVectorGyroscope();
      MagVector = bno055_getVectorMagnetometer();
    }
    if (BNO055_requested == 1) // Print BNO055 JSON to UART3
    {
      cJSON *root = cJSON_CreateObject();
      cJSON *imu = cJSON_CreateObject();

      cJSON_AddItemToObject(root, "IMU", imu);
      cJSON_AddNumberToObject(imu, "pitch", (double)DirVector.z);
      cJSON_AddNumberToObject(imu, "roll", (double)DirVector.y);
      cJSON_AddNumberToObject(imu, "heading", (double)DirVector.x);
      cJSON_AddNumberToObject(imu, "accel_x", (double)AccelVector.x);
      cJSON_AddNumberToObject(imu, "accel_y", (double)AccelVector.y);
      cJSON_AddNumberToObject(imu, "accel_z", (double)AccelVector.z);
      cJSON_AddNumberToObject(imu, "gyro_x", (double)GyroVector.x);
      cJSON_AddNumberToObject(imu, "gyro_y", (double)GyroVector.y);
      cJSON_AddNumberToObject(imu, "gyro_z", (double)GyroVector.z);
      cJSON_AddNumberToObject(imu, "mag_x", (double)MagVector.x);
      cJSON_AddNumberToObject(imu, "mag_y", (double)MagVector.x);
      cJSON_AddNumberToObject(imu, "mag_z", (double)MagVector.x);

      char *json_response = cJSON_Print(root);

      HAL_UART_Transmit_IT(&huart3, (uint8_t *)json_response, strlen(json_response));

      free(json_response);
      cJSON_Delete(root);
      BNO055_requested = 0;
    }
  }
}

void clear_buffer(uint8_t *buffer, uint32_t size)
{
  for (uint32_t i = 0; i < size; i++)
  {
    buffer[i] = 0;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    UART3_RX_buf[RX_index] = UART3_RX_byte;
    RX_index++;

    if (RX_index > BUFFER_SIZE - 2)
    {
      // buffer ended
      RX_index = 0;
      clear_buffer(UART3_RX_buf, BUFFER_SIZE);
    }

    if (UART3_RX_byte == '\n')
    {
      // end of message, start processing
      /*
      {
        "GET_IMU": 1,
        "SET_RC_MODE": 0,
        "SET_CONVEYOR_MODE": 0,
        "SET_CONVEYOR_DUTY_CYCLE": 0.5,
        "ESC1": 2000,
        "ESC2": 1500,
        "ESC3": 1500,
        "ESC4": 2000
      }
      */
      cJSON *command_json = cJSON_Parse((char *)UART3_RX_buf);
      if (command_json != NULL)
      {
        if (cJSON_GetObjectItem(command_json, "SET_RC_MODE") != NULL)
        {
          RC_mode = cJSON_GetObjectItem(command_json, "SET_RC_MODE")->valueint;
          if (RC_mode != 1 && RC_mode != 0)
          {
            RC_mode = 0;
            HAL_UART_Transmit(&huart3, (uint8_t *)"INVALID RC MODE\r\n", 18, 10);
          }
        }
        if (cJSON_GetObjectItem(command_json, "GET_IMU") != NULL)
        {
          BNO055_requested = cJSON_GetObjectItem(command_json, "GET_IMU")->valueint;
          if (BNO055_requested != 1 && BNO055_requested != 0)
          {
            BNO055_requested = 0;
            HAL_UART_Transmit(&huart3, (uint8_t *)"INVALID BNO055 REQUEST\r\n", 25, 10);
          }
        }
        if (cJSON_GetObjectItem(command_json, "SET_CONVEYOR_MODE") != NULL)
        {
          SSR_enabled[0] = cJSON_GetObjectItem(command_json, "SET_CONVEYOR_MODE")->valueint;
          SSR_enabled[1] = cJSON_GetObjectItem(command_json, "SET_CONVEYOR_MODE")->valueint;
          if (SSR_enabled[0] != 1 && SSR_enabled[0] != 0)
          {
            SSR_enabled[0] = 0;
            SSR_enabled[1] = 0;
            HAL_UART_Transmit(&huart3, (uint8_t *)"INVALID SSR MODE\r\n", 19, 10);
          }
        }
        if (cJSON_GetObjectItem(command_json, "SET_CONVEYOR_DUTY_CYCLE") != NULL)
        {
          SST_duty_cycle[0] = cJSON_GetObjectItem(command_json, "SET_CONVEYOR_DUTY_CYCLE")->valuedouble;
          SST_duty_cycle[1] = cJSON_GetObjectItem(command_json, "SET_CONVEYOR_DUTY_CYCLE")->valuedouble;
          if (SST_duty_cycle[0] < 0 || SST_duty_cycle[0] > 1)
          {
            SST_duty_cycle[0] = 0;
            SST_duty_cycle[1] = 0;
            HAL_UART_Transmit(&huart3, (uint8_t *)"INVALID SSR DUTY CYCLE\r\n", 25, 10);
          }
        }
        if (cJSON_GetObjectItem(command_json, "ESC1") != NULL)
        {
          ESC_Values[0] = cJSON_GetObjectItem(command_json, "ESC1")->valueint;
          if (ESC_Values[0] < 1000 || ESC_Values[0] > 2000)
          {
            ESC_Values[0] = 1500;
            HAL_UART_Transmit(&huart3, (uint8_t *)"INVALID ESC1 VALUE\r\n", 21, 10);
          }
        }
        if (cJSON_GetObjectItem(command_json, "ESC2") != NULL)
        {
          ESC_Values[1] = cJSON_GetObjectItem(command_json, "ESC2")->valueint;
          if (ESC_Values[1] < 1000 || ESC_Values[1] > 2000)
          {
            ESC_Values[1] = 1500;
            HAL_UART_Transmit(&huart3, (uint8_t *)"INVALID ESC2 VALUE\r\n", 21, 10);
          }
        }
        if (cJSON_GetObjectItem(command_json, "ESC3") != NULL)
        {
          ESC_Values[2] = cJSON_GetObjectItem(command_json, "ESC3")->valueint;
          if (ESC_Values[2] < 1000 || ESC_Values[2] > 2000)
          {
            ESC_Values[2] = 1500;
            HAL_UART_Transmit(&huart3, (uint8_t *)"INVALID ESC3 VALUE\r\n", 21, 10);
          }
        }
        if (cJSON_GetObjectItem(command_json, "ESC4") != NULL)
        {
          ESC_Values[3] = cJSON_GetObjectItem(command_json, "ESC4")->valueint;
          if (ESC_Values[3] < 1000 || ESC_Values[3] > 2000)
          {
            ESC_Values[3] = 1500;
            HAL_UART_Transmit(&huart3, (uint8_t *)"INVALID ESC4 VALUE\r\n", 21, 10);
          }
        }
      }

      cJSON_Delete(command_json);
      RX_index = 0;
      clear_buffer(UART3_RX_buf, BUFFER_SIZE);
    }
    HAL_UART_Receive_IT(&huart3, &UART3_RX_byte, 1);
  }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance != TIM3)
  {
    return;
  }

  static uint32_t rc_rising_edge[4] = {0};
  static uint8_t rc_edge_state[4] = {0}; // 0: waiting rising, 1: waiting falling

  uint8_t ch = 0;
  switch (htim->Channel)
  {
  case HAL_TIM_ACTIVE_CHANNEL_1:
    ch = 0;
    break;
  case HAL_TIM_ACTIVE_CHANNEL_2:
    ch = 1;
    break;
  case HAL_TIM_ACTIVE_CHANNEL_3:
    ch = 2;
    break;
  case HAL_TIM_ACTIVE_CHANNEL_4:
    ch = 3;
    break;
  default:
    return;
  }

  uint32_t captured_value = HAL_TIM_ReadCapturedValue(htim, RC_CHANNELS[ch]);

  if (rc_edge_state[ch] == 0)
  {
    rc_rising_edge[ch] = captured_value;
    __HAL_TIM_SET_CAPTUREPOLARITY(htim, RC_CHANNELS[ch], TIM_INPUTCHANNELPOLARITY_FALLING);
    rc_edge_state[ch] = 1;
  }
  else
  {
    uint32_t pulse;
    if (captured_value >= rc_rising_edge[ch])
    {
      pulse = captured_value - rc_rising_edge[ch];
    }
    else
    {
      pulse = htim->Init.Period - rc_rising_edge[ch] + captured_value;
    }

    RC_Values[ch] = pulse;
    __HAL_TIM_SET_CAPTUREPOLARITY(htim, RC_CHANNELS[ch], TIM_INPUTCHANNELPOLARITY_RISING);
    rc_edge_state[ch] = 0;
  }
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 47;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 47;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 47;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  // huart3.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS; // PCB error
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */
}

/**
 * @brief USB Initialization Function
 * @param None
 * @retval None
 */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12 | GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB12 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Configure PB13 and PB14 as input due PCB Error
  GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL; // Or use GPIO_PULLUP / GPIO_PULLDOWN if needed
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
