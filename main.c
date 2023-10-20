/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
#include "stm32f0xx.h"
#include <lcd_stm32f0.c>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// TODO: Add values for below variables
#define NS 128      // Number of samples in LUT
#define TIM2CLK 8000000   // STM Clock frequency
#define F_SIGNAL 500 // Frequency of output analog signal
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint32_t prev_millis = 0;
uint32_t curr_millis = 0;


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim2_ch1;


/* USER CODE BEGIN PV */
// TODO: Add code for global variables, including LUTs

uint32_t currentLUTType = 1;

uint32_t Sin_LUT[NS] = {0, 25.303, 50.591, 75.848, 101.06, 126.21, 151.28, 176.26,
						201.13, 225.88, 250.49, 274.94, 299.23, 323.34, 347.24, 370.94,
						394.41, 417.63, 440.6, 463.3, 485.72, 507.84, 529.65, 551.14,
						572.29, 593.09, 613.52, 633.58, 653.25, 672.53, 691.39, 709.83,
						727.83, 745.39, 762.49, 779.13, 795.29, 810.96, 826.13, 840.81,
						854.96, 868.59, 881.7, 894.26, 906.27, 917.73, 928.63, 938.96,
						948.72, 957.9, 966.48, 974.48, 981.88, 988.69, 994.88, 1000.5,
						1005.4, 1009.8, 1013.5, 1016.7, 1019.2, 1021, 1022.3, 1022.9,
						1022.9, 1022.3, 1021, 1019.2, 1016.7, 1013.5, 1009.8, 1005.4,
						1000.5, 994.88, 988.69, 981.88, 974.48, 966.48, 957.9, 948.72,
						938.96, 928.63, 917.73, 906.27, 894.26, 881.7, 868.59, 854.96,
						840.81, 826.13, 810.96, 795.29, 779.13, 762.49, 745.39, 727.83,
						709.83, 691.39, 672.53, 653.25, 633.58, 613.52, 593.09, 572.29,
						551.14, 529.65, 507.84, 485.72, 463.3, 440.6, 417.63, 394.41,
						370.94, 347.24, 323.34, 299.23, 274.94, 250.49, 225.88, 201.13,
						176.26, 151.28, 126.21, 101.06, 75.848, 50.591, 25.303, 1.2528e-13
};

uint32_t saw_LUT[NS] = {0, 8.0551, 16.11, 24.165, 32.22, 40.276, 48.331, 56.386,
		64.441, 72.496, 80.551, 88.606, 96.661, 104.72, 112.77,
		120.83, 128.88, 136.94, 144.99, 153.05, 161.1, 169.16, 177.21,
		185.27, 193.32, 201.38, 209.43, 217.49, 225.54, 233.6, 241.65,
		249.71, 257.76, 265.82, 273.87, 281.93, 289.98, 298.04, 306.09,
		314.15, 322.2, 330.26, 338.31, 346.37, 354.43, 362.48, 370.54,
		378.59, 386.65, 394.7, 402.76, 410.81, 418.87, 426.92, 434.98,
		443.03, 451.09, 459.14, 467.2, 475.25, 483.31, 491.36, 499.42,
		507.47, 515.53, 523.58, 531.64, 539.69, 547.75, 555.8, 563.86,
		571.91, 579.97, 588.02, 596.08, 604.13, 612.19, 620.24, 628.3,
		636.35, 644.41, 652.46, 660.52, 668.57, 676.63, 684.69, 692.74,
		700.8, 708.85, 716.91, 724.96, 733.02, 741.07, 749.13, 757.18,
		765.24, 773.29, 781.35, 789.4, 797.46, 805.51, 813.57, 821.62,
		829.68, 837.73, 845.79, 853.84, 861.9, 869.95, 878.01, 886.06,
		894.12, 902.17, 910.23, 918.28, 926.34, 934.39, 942.45, 950.5,
		958.56, 966.61, 974.67, 982.72, 990.78, 998.83, 1006.9, 1014.9, 1023};

uint32_t triangle_LUT[NS] = {16,32,48,64,80,96,112,128,
		144,160,176,192,208,224,240,256,
		272,288,304,320,336,352,368,384,
		400,416,432,448,464,480,496,512,
		527,543,559,575,591,607,623,639,
		655,671,687,703,719,735,751,767,
		783,799,815,831,847,863,879,895,
		911,927,943,959,975,991,1007,1023,
		1007,991,975,959,943,927,911,895,
		879,863,847,831,815,799,783,767,
		751,735,719,703,687,671,655,639,
		623,607,591,575,559,543,527,512,
		496,480,464,448,432,416,400,384,
		368,352,336,320,304,288,272,256,
		240,224,208,192,176,160,144,128,
		112,96,80,64,48,32,16,0
};

// TODO: Equation to calculate TIM2_Ticks

uint32_t TIM2_Ticks = TIM2CLK /(F_SIGNAL * NS);
uint32_t DestAddress = (uint32_t) &(TIM3->CCR3); // Write LUT TO TIM3->CCR3 to modify PWM duty cycle


/* USER CODE END PV */


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */
void EXTI0_1_IRQHandler(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  init_LCD();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
  // TODO: Start TIM3 in PWM mode on channel 3
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

  // TODO: Start TIM2 in Output Compare (OC) mode on channel 1.
  HAL_TIM_OC_Init(&htim2);
  HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_1);

  // TODO: Start DMA in IT mode on TIM2->CH1; Source is LUT and Dest is TIM3->CCR3; start with Sine LUT
  HAL_DMA_Init(&hdma_tim2_ch1);
  __HAL_LINKDMA(&htim2, hdma[TIM_DMA_ID_CC1], hdma_tim2_ch1);

  // TODO: Write current waveform to LCD ("Sine")
  delay(3000);
  	  lcd_command(CLEAR);
    //function from LCD library to display string
	char sine_text[] = "Sine";
	lcd_putstring(sine_text);

  // TODO: Enable DMA (start transfer from LUT to CCR)
	HAL_DMA_Start_IT(&hdma_tim2_ch1, (uint32_t)Sin_LUT, (uint32_t)DestAddress, NS);

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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {

  }
  LL_SetSystemCoreClock(8000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = TIM2_Ticks - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1023;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE0);

  /**/
  LL_GPIO_SetPinPull(Button0_GPIO_Port, Button0_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinMode(Button0_GPIO_Port, Button0_Pin, LL_GPIO_MODE_INPUT);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_0;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
  LL_EXTI_Init(&EXTI_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void EXTI0_1_IRQHandler(void)
{
	// TODO: De-bounce using HAL_GetTick()
	  curr_millis = HAL_GetTick();
		if (curr_millis - prev_millis >= 100){

	// TODO: Disable DMA transfer and abort IT, then start DMA in IT mode with new LUT and re-enable transfer
	// HINT: Consider using C's "switch" function to handle LUT changes

    // Determine the new LUT source based on a condition (e.g., a variable)
    uint32_t* newLUT = NULL;
    char waveformText[20];

    switch (currentLUTType) {
        case 1:
            strcpy(waveformText, "Sine");
            currentLUTType = 2;
            // Disable DMA transfer
            __HAL_TIM_DISABLE_DMA(&htim2, TIM_DMA_CC1);
            HAL_DMA_Abort_IT(&hdma_tim2_ch1);

            // Configure DMA with the new LUT
            __HAL_LINKDMA(&htim2, hdma[TIM_DMA_ID_CC1], hdma_tim2_ch1);
            HAL_DMA_Start_IT(&hdma_tim2_ch1, (uint32_t)Sin_LUT, (uint32_t)DestAddress, NS);

            // Enable DMA transfer with the new LUT
            __HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);

            // Write the current waveform type to the LCD
            lcd_command(CLEAR);
            lcd_putstring(waveformText);
            break;
        case 2:
            strcpy(waveformText, "Sawtooth");
            currentLUTType = 3;
            // Disable DMA transfer
            __HAL_TIM_DISABLE_DMA(&htim2, TIM_DMA_CC1);
            HAL_DMA_Abort_IT(&hdma_tim2_ch1);

            // Configure DMA with the new LUT
            __HAL_LINKDMA(&htim2, hdma[TIM_DMA_ID_CC1], hdma_tim2_ch1);
            HAL_DMA_Start_IT(&hdma_tim2_ch1, (uint32_t)saw_LUT, (uint32_t)DestAddress, NS);

            // Enable DMA transfer with the new LUT
            __HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);

            // Write the current waveform type to the LCD
            lcd_command(CLEAR);
            lcd_putstring(waveformText);
            break;
        case 3:
            strcpy(waveformText, "Triangular");
            currentLUTType = 1;
            // Disable DMA transfer
            __HAL_TIM_DISABLE_DMA(&htim2, TIM_DMA_CC1);
            HAL_DMA_Abort_IT(&hdma_tim2_ch1);

            // Configure DMA with the new LUT
            __HAL_LINKDMA(&htim2, hdma[TIM_DMA_ID_CC1], hdma_tim2_ch1);
            HAL_DMA_Start_IT(&hdma_tim2_ch1, (uint32_t)triangle_LUT, (uint32_t)DestAddress, NS);

            // Enable DMA transfer with the new LUT
            __HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);

            // Write the current waveform type to the LCD
            lcd_command(CLEAR);
            lcd_putstring(waveformText);
            break;

        default:
            break;
    }

	prev_millis = curr_millis;
}

	HAL_GPIO_EXTI_IRQHandler(Button0_Pin); // Clear interrupt flags
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
