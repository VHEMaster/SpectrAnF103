#include "main.h"
#include "arm_math.h"

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

#define ADC_SIZE (2048)
#define FFT_SIZE (ADC_SIZE)
#define OUT_SIZE (FFT_SIZE / 2)

static uint16_t adcBuffer[ADC_SIZE] = {0};
static int32_t fftSamples[FFT_SIZE] = {0};
static int32_t fftMagnitudes[OUT_SIZE] = {0};
static int32_t fftLevels[10];
static volatile uint8_t fftReady = 0;

static uint8_t ledLevels[10];
static float ledMaxLevels[10];
static uint8_t ledData[10];
static uint8_t loweringTimings[10];

static uint32_t BSRR_REGS_COLUMNS_PORTA[10];
static uint32_t BSRR_REGS_COLUMNS_PORTB[10];
static uint32_t BSRR_REGS_ROWS_PORTA[15];
static uint32_t BSRR_REGS_ROWS_PORTB[15];

arm_cfft_radix2_instance_q31 fft;

uint32_t columnsA = LED_C9_Pin | LED_C8_Pin | LED_C7_Pin | LED_C6_Pin | LED_C5_Pin | LED_C4_Pin | LED_C3_Pin;
uint32_t columnsB = LED_C2_Pin | LED_C1_Pin | LED_C0_Pin;
uint32_t rowsA = LED_R7_Pin | LED_R8_Pin | LED_R9_Pin | LED_R10_Pin;
uint32_t rowsB = LED_R0_Pin | LED_R1_Pin | LED_R2_Pin | LED_R3_Pin | LED_R4_Pin | LED_R5_Pin | LED_R6_Pin | LED_R11_Pin | LED_R12_Pin | LED_R13_Pin;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);

static void InitGpioTables(void)
{
	BSRR_REGS_COLUMNS_PORTA[0] = 0;
	BSRR_REGS_COLUMNS_PORTA[1] = 0;
	BSRR_REGS_COLUMNS_PORTA[2] = 0;
	BSRR_REGS_COLUMNS_PORTA[3] = LED_C3_Pin;
	BSRR_REGS_COLUMNS_PORTA[4] = LED_C4_Pin;
	BSRR_REGS_COLUMNS_PORTA[5] = LED_C5_Pin;
	BSRR_REGS_COLUMNS_PORTA[6] = LED_C6_Pin;
	BSRR_REGS_COLUMNS_PORTA[7] = LED_C7_Pin;
	BSRR_REGS_COLUMNS_PORTA[8] = LED_C8_Pin;
	BSRR_REGS_COLUMNS_PORTA[9] = LED_C9_Pin;
	
	BSRR_REGS_COLUMNS_PORTB[0] = LED_C0_Pin;
	BSRR_REGS_COLUMNS_PORTB[1] = LED_C1_Pin;
	BSRR_REGS_COLUMNS_PORTB[2] = LED_C2_Pin;
	BSRR_REGS_COLUMNS_PORTB[3] = 0;
	BSRR_REGS_COLUMNS_PORTB[4] = 0;
	BSRR_REGS_COLUMNS_PORTB[5] = 0;
	BSRR_REGS_COLUMNS_PORTB[6] = 0;
	BSRR_REGS_COLUMNS_PORTB[7] = 0;
	BSRR_REGS_COLUMNS_PORTB[8] = 0;
	BSRR_REGS_COLUMNS_PORTB[9] = 0;
	
	BSRR_REGS_ROWS_PORTA[0]  = 0;
	BSRR_REGS_ROWS_PORTA[1]  = 0;
	BSRR_REGS_ROWS_PORTA[2]  = 0;
	BSRR_REGS_ROWS_PORTA[3]  = 0;
	BSRR_REGS_ROWS_PORTA[4]  = 0;
	BSRR_REGS_ROWS_PORTA[5]  = 0;
	BSRR_REGS_ROWS_PORTA[6]  = 0;
	BSRR_REGS_ROWS_PORTA[7]  = 0;
	BSRR_REGS_ROWS_PORTA[8]  = LED_R7_Pin;
	BSRR_REGS_ROWS_PORTA[9]  = LED_R8_Pin;
	BSRR_REGS_ROWS_PORTA[10] = LED_R9_Pin;
	BSRR_REGS_ROWS_PORTA[11] = LED_R10_Pin;
	BSRR_REGS_ROWS_PORTA[12] = 0;
	BSRR_REGS_ROWS_PORTA[13] = 0;
	BSRR_REGS_ROWS_PORTA[14] = 0;
	
	BSRR_REGS_ROWS_PORTB[0]  = 0;
	BSRR_REGS_ROWS_PORTB[1]  = LED_R0_Pin;
	BSRR_REGS_ROWS_PORTB[2]  = LED_R1_Pin;
	BSRR_REGS_ROWS_PORTB[3]  = LED_R2_Pin;
	BSRR_REGS_ROWS_PORTB[4]  = LED_R3_Pin;
	BSRR_REGS_ROWS_PORTB[5]  = LED_R4_Pin;
	BSRR_REGS_ROWS_PORTB[6]  = LED_R5_Pin;
	BSRR_REGS_ROWS_PORTB[7]  = LED_R6_Pin;
	BSRR_REGS_ROWS_PORTB[8]  = 0;
	BSRR_REGS_ROWS_PORTB[9]  = 0;
	BSRR_REGS_ROWS_PORTB[10] = 0;
	BSRR_REGS_ROWS_PORTB[11] = 0;
	BSRR_REGS_ROWS_PORTB[12] = LED_R11_Pin;
	BSRR_REGS_ROWS_PORTB[13] = LED_R12_Pin;
	BSRR_REGS_ROWS_PORTB[14] = LED_R13_Pin;
	
	for(int i = 0; i < 10; i++)
	{
		BSRR_REGS_COLUMNS_PORTA[i] |= columnsA << 16;
		BSRR_REGS_COLUMNS_PORTB[i] |= columnsB << 16;
		
		BSRR_REGS_COLUMNS_PORTA[i] &= ~(BSRR_REGS_COLUMNS_PORTA[i] << 16);
		BSRR_REGS_COLUMNS_PORTB[i] &= ~(BSRR_REGS_COLUMNS_PORTB[i] << 16);
	}
	for(int i = 0; i < 15; i++)
	{
		for(int j = 0; j < i; j++)
		{
			BSRR_REGS_ROWS_PORTA[i] |= BSRR_REGS_ROWS_PORTA[j] & rowsA;
			BSRR_REGS_ROWS_PORTB[i] |= BSRR_REGS_ROWS_PORTB[j] & rowsB;
		}
		
		BSRR_REGS_ROWS_PORTA[i] |= rowsA << 16;
		BSRR_REGS_ROWS_PORTB[i] |= rowsB << 16;
		
		BSRR_REGS_ROWS_PORTA[i] &= ~(BSRR_REGS_ROWS_PORTA[i] << 16);
		BSRR_REGS_ROWS_PORTB[i] &= ~(BSRR_REGS_ROWS_PORTB[i] << 16);
	}
}

static void ProcessFFT(uint16_t * data)
{
	if(fftReady)
		return;
	
	for(int i = 0, j = 0; i <  ADC_SIZE / 2;)
	{
		fftSamples[j++] = (adcBuffer[i++] - 0x8000) * 0x10000;
		fftSamples[j++] = 0;
	}
	fftReady = 1;
	
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	ProcessFFT(&adcBuffer[ADC_SIZE / 2]);
}


void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
	ProcessFFT(&adcBuffer[0]);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint8_t column = 0;
	static uint8_t deadtime = 0;
	
	GPIOA->BSRR = (columnsA | rowsA) << 16;
	GPIOB->BSRR = (columnsB | rowsB) << 16;
	
	//Accepting ARR changes only after previous counter overflow?
	if(deadtime)
	{
		htim->Instance->ARR = 719;
	}
	else
	{
		htim->Instance->ARR = 71;
		
		GPIOA->BSRR = BSRR_REGS_ROWS_PORTA[ledData[column]];
		GPIOB->BSRR = BSRR_REGS_ROWS_PORTB[ledData[column]];
		
		GPIOA->BSRR = BSRR_REGS_COLUMNS_PORTA[column];
		GPIOB->BSRR = BSRR_REGS_COLUMNS_PORTB[column];
		
		if(++column >= 10)
			column = 0;
	}
	deadtime ^= 1;
	
}

int main(void)
{
	int index;
	
  HAL_Init();
  SystemClock_Config();
	
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
	
	InitGpioTables();
	
	arm_cfft_radix2_init_q31(&fft, FFT_SIZE / 2, 0, 1);
	
	HAL_TIM_Base_Start_IT(&htim4);
	
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcBuffer, ADC_SIZE);
	HAL_TIM_Base_Start(&htim3);
	
  while (1)
  {
		if(fftReady)
		{
			arm_cfft_radix2_q31(&fft, fftSamples);
			arm_cmplx_mag_q31(fftSamples, fftMagnitudes, OUT_SIZE);
			fftReady = 0;
			
			//Maybe it's better to refactor it in future... Seems ugly
			index = 0;
			fftLevels[index] = 0;
			for(int i=1;i<=1;i++)
				if(fftMagnitudes[i] > fftLevels[index]) fftLevels[index] = fftMagnitudes[i];
			
			index = 1;
			fftLevels[index] = 0;
			for(int i=2;i<=2;i++)
				if(fftMagnitudes[i] > fftLevels[index]) fftLevels[index] = fftMagnitudes[i];
			
			index = 2;
			fftLevels[index] = 0;
			for(int i=3;i<=4;i++)
				if(fftMagnitudes[i] > fftLevels[index]) fftLevels[index] = fftMagnitudes[i];
			
			index = 3;
			fftLevels[index] = 0;
			for(int i=5;i<=8;i++)
				if(fftMagnitudes[i] > fftLevels[index]) fftLevels[index] = fftMagnitudes[i];
			
			index = 4;
			fftLevels[index] = 0;
			for(int i=9;i<=18;i++)
				if(fftMagnitudes[i] > fftLevels[index]) fftLevels[index] = fftMagnitudes[i];
			
			index = 5;
			fftLevels[index] = 0;
			for(int i=19;i<=35;i++)
				if(fftMagnitudes[i] > fftLevels[index]) fftLevels[index] = fftMagnitudes[i];
			
			index = 6;
			fftLevels[index] = 0;
			for(int i=36;i<=57;i++)
				if(fftMagnitudes[i] > fftLevels[index]) fftLevels[index] = fftMagnitudes[i];
			
			index = 7;
			fftLevels[index] = 0;
			for(int i=58;i<=149;i++)
				if(fftMagnitudes[i] > fftLevels[index]) fftLevels[index] = fftMagnitudes[i];
			
			index = 8;
			fftLevels[index] = 0;
			for(int i=150;i<=298;i++)
				if(fftMagnitudes[i] > fftLevels[index]) fftLevels[index] = fftMagnitudes[i];
				
			index = 9;
			fftLevels[index] = 0;
			for(int i=299;i<512;i++)
				if(fftMagnitudes[i] > fftLevels[index]) fftLevels[index] = fftMagnitudes[i];
				
			for(int i=0;i<10;i++)
			{
				if(fftLevels[i] >= 30720<<10) ledLevels[i] = 14;
				else if(fftLevels[i] >= 22400<<10) ledLevels[i] = 13;
				else if(fftLevels[i] >= 16000<<10) ledLevels[i] = 12;
				else if(fftLevels[i] >= 11584<<10) ledLevels[i] = 11;
				else if(fftLevels[i] >= 8192<<10) ledLevels[i] = 10;
				else if(fftLevels[i] >= 5824<<10) ledLevels[i] = 9;
				else if(fftLevels[i] >= 4096<<10) ledLevels[i] = 8;
				else if(fftLevels[i] >= 2944<<10) ledLevels[i] = 7;
				else if(fftLevels[i] >= 2048<<10) ledLevels[i] = 6;
				else if(fftLevels[i] >= 1472<<10) ledLevels[i] = 5;
				else if(fftLevels[i] >= 1024<<10) ledLevels[i] = 4;
				else if(fftLevels[i] >= 704<<10) ledLevels[i] = 3;
				else if(fftLevels[i] >= 518<<10) ledLevels[i] = 2;
				else if(fftLevels[i] >= 365<<10) ledLevels[i] = 1;
				else ledLevels[i] = 0;
				
				if(ledLevels[i] > ledMaxLevels[i]) 
				{
					ledMaxLevels[i] = ledLevels[i];
					loweringTimings[i] = 3;
				}
				else if(loweringTimings[i] == 0)
				{
					ledMaxLevels[i] -= 0.2f;
					if(ledMaxLevels[i] < 0) ledMaxLevels[i] = 0;
				}
				else loweringTimings[i]--;
				
			}
			
			for(int i=0;i<10;i++)
				ledData[i] = ledMaxLevels[i];
		}
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_LEFT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1499;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 719;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_C9_Pin|LED_C8_Pin|LED_C7_Pin|LED_C6_Pin
                          |LED_C5_Pin|LED_C4_Pin|LED_C3_Pin|LED_R10_Pin
                          |LED_R9_Pin|LED_R8_Pin|LED_R7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_C2_Pin|LED_C1_Pin|LED_C0_Pin|LED_R13_Pin
                          |LED_R12_Pin|LED_R11_Pin|LED_R6_Pin|LED_R5_Pin
                          |LED_R4_Pin|LED_R3_Pin|LED_R2_Pin|LED_R1_Pin
                          |LED_R0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_C9_Pin LED_C8_Pin LED_C7_Pin LED_C6_Pin
                           LED_C5_Pin LED_C4_Pin LED_C3_Pin LED_R10_Pin
                           LED_R9_Pin LED_R8_Pin LED_R7_Pin */
  GPIO_InitStruct.Pin = LED_C9_Pin|LED_C8_Pin|LED_C7_Pin|LED_C6_Pin
                          |LED_C5_Pin|LED_C4_Pin|LED_C3_Pin|LED_R10_Pin
                          |LED_R9_Pin|LED_R8_Pin|LED_R7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_C2_Pin LED_C1_Pin LED_C0_Pin LED_R13_Pin
                           LED_R12_Pin LED_R11_Pin LED_R6_Pin LED_R5_Pin
                           LED_R4_Pin LED_R3_Pin LED_R2_Pin LED_R1_Pin
                           LED_R0_Pin */
  GPIO_InitStruct.Pin = LED_C2_Pin|LED_C1_Pin|LED_C0_Pin|LED_R13_Pin
                          |LED_R12_Pin|LED_R11_Pin|LED_R6_Pin|LED_R5_Pin
                          |LED_R4_Pin|LED_R3_Pin|LED_R2_Pin|LED_R1_Pin
                          |LED_R0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
