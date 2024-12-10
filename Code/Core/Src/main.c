/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim14;

/* USER CODE BEGIN PV */
uint8_t sine_table[TABLE_SIZE];
uint8_t square_table[TABLE_SIZE];
uint8_t triangle_table[TABLE_SIZE];

volatile WaveformType current_waveform = WAVEFORM_SINE;

volatile bool activate_cap1_state; // Capacitor 1 -> 10uF
volatile bool activate_cap2_state; // Capacitor 2 -> 1uF
volatile bool activate_cap3_state; // Capacitor 3 -> 100nF
volatile bool activate_cap4_state; // Capacitor 4 -> 10nF
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM14_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
void DAC_SetValue(uint8_t value);
void GenerateSineTable();
void GenerateSquareTable();
void GenerateTriangleTable();
void POT_SetValue(uint8_t value);
void CycleWaveform();
void set_RC_filter(uint8_t pot_value, bool cap1, bool cap2, bool cap3, bool cap4); // Pour faire des ajustements facilement
void set_capacitors(bool cap1, bool cap2, bool cap3, bool cap4);
void adjust_pot_based_on_TIM14(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void GenerateSineTable() {
    for(int i = 0; i < TABLE_SIZE; i++)
        sine_table[i] = (uint8_t)((sin(2 * PI * i / TABLE_SIZE) + 1) * (127.5/2)); // Scale to 0-255
}

void GenerateSquareTable() {
    for(int i = 0; i < TABLE_SIZE; i++)
        square_table[i] = (i < (TABLE_SIZE / 2)) ? 128 : 0;
}

void GenerateTriangleTable() {
    for(int i = 0; i <= TABLE_SIZE/2; i++)
        	triangle_table[i] = i * (256/TABLE_SIZE);    // Rising edge

    for(int i = TABLE_SIZE/2; i < TABLE_SIZE; i++)
    	triangle_table[i] = 255 - (i * (256/TABLE_SIZE));    	// Falling edge
}

void CycleWaveform() {
    current_waveform = (current_waveform + 1) % WAVEFORM_COUNT;
}

void DAC_SetValue(uint8_t value) {
    uint16_t data = 0;

    data |= MCP4901_BUFFER;
    data |= MCP4901_GAIN2X;
    data |= MCP4901_ACTIVE;
    data |= (value << 4); // Place the 8-bit data in bits 11-4

    // Drive the CS pin low (active low)
    HAL_GPIO_WritePin(GPIOB, CS_Pin, GPIO_PIN_RESET);

    // Transmit data over SPI
    HAL_SPI_Transmit(&hspi1, (uint8_t*)&data, 1, HAL_MAX_DELAY);

    // Then raise CS (causes the data to be latched into the DAC's input register)
    HAL_GPIO_WritePin(GPIOB, CS_Pin, GPIO_PIN_SET);
}

void POT_SetValue(uint8_t value) {
	if(value > 128)
		value = 127;

	uint16_t data = 0;

	data |= value;

	// Enable (NCS low)
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET);

	HAL_SPI_Transmit(&hspi2, (uint8_t*)&data, 1, HAL_MAX_DELAY);

	// Disable (NCS high)
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET);
}

/**
 * @brief Sets the RC filter by configuring the potentiometer and activating/deactivating capacitors.
 *
 * @param pot_value    The desired potentiometer value (0 to 127).
 * @param cap1         Activate Capacitor 1 if true; deactivate if false.
 * @param cap2         Activate Capacitor 2 if true; deactivate if false.
 * @param cap3         Activate Capacitor 3 if true; deactivate if false.
 * @param cap4         Activate Capacitor 4 if true; deactivate if false.
 */
void set_RC_filter(uint8_t pot_value, bool cap1, bool cap2, bool cap3, bool cap4) {
    // Limit potentiometer value to 127 (7-bit resolution)
    if(pot_value > 127)
        pot_value = 127;

    // Set the potentiometer to the specified value
    POT_SetValue(pot_value);

    // Activate or deactivate Capacitor 1 (PB0)
    if(cap1)
        HAL_GPIO_WritePin(GPIOB, SW1_Pin, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(GPIOB, SW1_Pin, GPIO_PIN_RESET);

    // Activate or deactivate Capacitor 2 (PB3)
    if(cap2)
        HAL_GPIO_WritePin(GPIOB, SW2_Pin, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(GPIOB, SW2_Pin, GPIO_PIN_RESET);

    // Activate or deactivate Capacitor 3 (PB8)
    if(cap3)
        HAL_GPIO_WritePin(GPIOB, SW3_Pin, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(GPIOB, SW3_Pin, GPIO_PIN_RESET);

    // Activate or deactivate Capacitor 4 (PB4)
    if(cap4)
        HAL_GPIO_WritePin(GPIOB, SW4_Pin, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(GPIOB, SW4_Pin, GPIO_PIN_RESET);
}

void set_capacitors(bool cap1, bool cap2, bool cap3, bool cap4) {
    // Update global capacitor state variables
    activate_cap1_state = cap1;
    activate_cap2_state = cap2;
    activate_cap3_state = cap3;
    activate_cap4_state = cap4;

    // Activate or deactivate Capacitor 1 (SW1_Pin = PB0 = D6)
    if (cap1) {
        HAL_GPIO_WritePin(GPIOB, SW1_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOB, SW1_Pin, GPIO_PIN_RESET);
    }

    // Activate or deactivate Capacitor 2 (SW2_Pin = PB3 = D13)
    if (cap2) {
        HAL_GPIO_WritePin(GPIOB, SW2_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOB, SW2_Pin, GPIO_PIN_RESET);
    }

    // Activate or deactivate Capacitor 3 (SW3_Pin = PB8 = D8)
    if (cap3) {
        HAL_GPIO_WritePin(GPIOB, SW3_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOB, SW3_Pin, GPIO_PIN_RESET);
    }

    // Activate or deactivate Capacitor 4 (SW4_Pin = PB4 = D12)
    if (cap4) {
        HAL_GPIO_WritePin(GPIOB, SW4_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOB, SW4_Pin, GPIO_PIN_RESET);
    }
}

void adjust_pot_based_on_TIM14(void) {
    // Retrieve the current ARR value from TIM14
    uint32_t current_ARR = htim14.Instance->ARR;

    uint8_t pot_value = 0; // Default potentiometer value

    // Note: Higher ARR (lower SPI rate) -> Higher pot_value (higher R, lower f_c)

    switch(current_waveform) {
        case WAVEFORM_SQUARE:
            // No filtering: deactivate all capacitors and set potentiometer to lowest value
            pot_value = 0; // Lowest potentiometer value
            set_capacitors(false, false, false, false);
            POT_SetValue(pot_value);
            break;

        case WAVEFORM_SINE:
            if (current_ARR <= 400)
                pot_value = 100;
            else if (current_ARR <= 800)
                pot_value = 80;
            else if (current_ARR <= 1200)
                pot_value = 60;
            else if (current_ARR <= 1600)
                pot_value = 40;
            else if (current_ARR <= 2000)
                pot_value = 20;
            else
                pot_value = 127;

            // On doit remettre l'états des capa vu qu'elles sont reset quand current_waveform == WAVEFORM_SQUARE
            activate_cap1_state = false;
            activate_cap2_state = false;
            activate_cap3_state = true;
            activate_cap4_state = false;

            set_capacitors(activate_cap1_state, activate_cap2_state, activate_cap3_state, activate_cap4_state);
            POT_SetValue(pot_value);
            break;

        case WAVEFORM_TRIANGLE:
            if (current_ARR <= 400)
                pot_value = 0;
            else if (current_ARR <= 800)
                pot_value = 10;
            else if (current_ARR <= 1200)
                pot_value = 10;
            else
                pot_value = 10;

            activate_cap1_state = false;
            activate_cap2_state = false;
            activate_cap3_state = true;
            activate_cap4_state = false;

            set_capacitors(activate_cap1_state, activate_cap2_state, activate_cap3_state, activate_cap4_state);
            POT_SetValue(pot_value);
            break;

        default:
            // boh ?
            break;
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  GenerateSineTable();
  GenerateSquareTable();
  GenerateTriangleTable();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_TIM14_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim14);

  // Config initiale des capa. (et des variables globales)
  //-----------------------------
  activate_cap1_state = false;    // sur la breadboard -> 10nF
  activate_cap2_state = false;     // 100nF
  activate_cap3_state = true;
  activate_cap4_state = false;

  // Set capacitors (without changing the potentiometer)
  set_capacitors(activate_cap1_state,
		  	  	 activate_cap2_state,
				 activate_cap3_state,
				 activate_cap4_state);
  //-----------------------------
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 8;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 200;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CS_Pin|SW1_Pin|GPIO_PIN_2|SW2_Pin
                          |SW4_Pin|SW3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CS_Pin SW1_Pin PB2 SW2_Pin
                           SW4_Pin SW3_Pin */
  GPIO_InitStruct.Pin = CS_Pin|SW1_Pin|GPIO_PIN_2|SW2_Pin
                          |SW4_Pin|SW3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : T_VCP_TX_Pin T_VCP_RX_Pin */
  GPIO_InitStruct.Pin = T_VCP_TX_Pin|T_VCP_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//Interrupt for increasing frequency when user push a button. Do exactly the same thing for decreasing it.
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_6)
		if(TIM14 -> ARR < 2000)
			TIM14 -> ARR += 50; //adjustable, one idea is to divide the frequency band in n slices.

	if(GPIO_Pin == GPIO_PIN_7)
		if(TIM14 -> ARR > 200)
			TIM14 -> ARR -= 50;

	if(GPIO_Pin == GPIO_PIN_12)
		CycleWaveform();

	if(AUTO_POT_AJUST)
		adjust_pot_based_on_TIM14();
}
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

#ifdef  USE_FULL_ASSERT
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
