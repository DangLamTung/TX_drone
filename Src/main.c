/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "nrf24_hal.h"
#include "nrf24.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
uint16_t a[6];
float temp[6];
uint8_t buf[12];
uint16_t value,value1, value2;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t sent_pack = 0;
uint32_t lost_pckts = 0;
uint8_t status;
uint8_t TRANSMIT_ON = 0;
uint8_t ARM_ON = 0;
int count = 0;
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_TIM4_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
//  ssd1306_TestAll();
  ssd1306_Init();
  ssd1306_Fill(Black);
  ssd1306_SetCursor(2, 0);
  ssd1306_WriteString("ARM:",  Font_6x8,White);
  ssd1306_SetCursor(2, 10);
  ssd1306_WriteString("MODE:",  Font_6x8,White);

  ssd1306_UpdateScreen();
//  for(int i = 0; i < 255; i++){
//	  if(HAL_I2C_IsDeviceReady(&hi2c1, i, 3, 2) == HAL_OK){
//	  			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
////	  			I2C_ClearBusyFlagErratum(&i2c, 10);
//	      	}
//  }
//  HAL_ADC_Start_DMA(&hadc1,a,2);  // start the adc in dma mode
  // here value is the buffer, where the adc values are going to store
  // 3 is the number of values going to store == no. of channels we are using
  uint8_t ADDR[] = { 'n', 'R', 'F', '2', '4' }; // the TX address
  nRF24_SetRFChannel(90); // set RF channel to 2490MHz
  nRF24_SetDataRate(nRF24_DR_2Mbps); // 2Mbit/s data rate
  nRF24_SetCRCScheme(nRF24_CRC_1byte); // 1-byte CRC scheme
  nRF24_SetAddrWidth(5); // address width is 5 bytes
  nRF24_SetAddr(nRF24_PIPETX, ADDR); // program TX address
  nRF24_SetAddr(nRF24_PIPE0, ADDR); // program pipe#0 RX address, must be same as TX (for ACK packets)
  nRF24_SetTXPower(nRF24_TXPWR_0dBm); // configure TX power
  nRF24_SetAutoRetr(nRF24_ARD_2500us, 10); // configure auto retransmit: 10 retransmissions with pause of 2500s in between
  nRF24_EnableAA(nRF24_PIPE0); // enable Auto-ACK for pipe#0 (for ACK packets)
  nRF24_SetOperationalMode(nRF24_MODE_TX); // switch transceiver to the TX mode
  nRF24_SetPowerMode(nRF24_PWR_UP); // wake-up transceiver (in case if it sleeping)
  HAL_ADC_Start_DMA(&hadc1,a,6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  HAL_Delay (100); // 200ms delay

	  temp[0] = 0;
	  temp[1] = 0;
	  temp[2] = 0;
	  temp[3] = 0;
	  temp[4] = 0;
	  temp[5] = 0;
      count = 0;

	  buf[0] = 's';
	  buf[1] = (a[1] & 0xFF00) >> 8;
      buf[2] = (a[1] & 0x00FF);

	  buf[3] = (a[2] & 0xFF00) >> 8;
      buf[4] = (a[2] & 0x00FF);

	  buf[5] = (a[3] & 0xFF00) >> 8;
      buf[6] = (a[3] & 0x00FF);

	  buf[7] = (a[4] & 0xFF00) >> 8;
      buf[8] = (a[4] & 0x00FF);
      buf[9] = 'e';
      char str[4];
      sprintf(str, "%d ", a[1]);

	  CDC_Transmit_FS(str, strlen(str));
      ssd1306_SetCursor(2, 20);
      ssd1306_WriteString(str,  Font_6x8,White);

      sprintf(str, "%d ", a[2]);
      CDC_Transmit_FS(str,strlen(str));
      ssd1306_SetCursor(30, 20);
      ssd1306_WriteString(str,  Font_6x8,White);

      sprintf(str, "%d ", a[3]);
      CDC_Transmit_FS(str,strlen(str));
      ssd1306_SetCursor(2, 30);
      ssd1306_WriteString(str,  Font_6x8,White);

      sprintf(str, "%d ", a[4]);
      CDC_Transmit_FS(str, strlen(str));
      ssd1306_SetCursor(30, 30);
      ssd1306_WriteString(str,  Font_6x8,White);
      CDC_Transmit_FS("\n", 1);
//	  HAL_UART_Transmit(&huart1,(uint8_t*) buf, strlen(buf),1000);
//      ssd1306_Fill(Black);
      if(ARM_ON){
				  ssd1306_SetCursor(22, 0);
				  ssd1306_WriteString("OFF",  Font_6x8,Black);
    			  ssd1306_SetCursor(22, 0);
    			  ssd1306_WriteString("ON ",  Font_6x8,White);
    		  }
    		  else{
    			  ssd1306_SetCursor(22, 0);
    			  ssd1306_WriteString("ON ",  Font_6x8,Black);
    			  ssd1306_SetCursor(22, 0);
    			  ssd1306_WriteString("OFF",  Font_6x8,White);
    		  }
	  if(TRANSMIT_ON){
	  nRF24_WritePayload(buf, 10); // transfer payload data to transceiver
	  nRF24_CE_H; // assert CE pin (transmission starts)

		  status = nRF24_GetStatus(); // get status of nRF24
		  nRF24_ClearIRQFlags(); // clear any pending IRQ flags
		  uint8_t otx = nRF24_GetRetransmitCounters(); // get some stats, this variable will contain two counters
		  uint8_t otx_plos_cnt = (otx & nRF24_MASK_PLOS_CNT) >> 4; // packets lost counter
		  uint8_t otx_arc_cnt  = (otx & nRF24_MASK_ARC_CNT); // auto retransmissions counter
		  if (otx_arc_cnt > 0) {
		      // ARC_CNT is non zero, has been few retransmissions
		      // ... do something here with this knowlege ...
		      // This counter will be reset when a new transmission starts
		  }
		  if (status & nRF24_FLAG_TX_DS) {
//			  break;
			  sent_pack++;
		  }
		  if (status & nRF24_FLAG_MAX_RT) {
		      // Maximum number of retransmits happened, the lost packets counter must be non zero
		      // This counter counts up to 15 and then stops (protection from overflow as datasheet says)
		      // For example the software can have some variable as counter for lost packets (e.g. uint32_t lost_pckts)...
		      lost_pckts += otx_plos_cnt; // update global counter of lost packets
		      nRF24_ResetPLOS(); // reset PLOS_CNT value
		  }

	  nRF24_CE_L; // de-assert CE pin (nRF24 goes to StandBy-I mode)
	  nRF24_ClearIRQFlags(); // clear any pending IRQ flags

	  ssd1306_SetCursor(30, 10);
	  ssd1306_WriteString("OFF",  Font_6x8,Black);

	  ssd1306_SetCursor(30, 10);
	  ssd1306_WriteString("ON ",  Font_6x8,White);

	  ssd1306_SetCursor(60, 0);
	  ssd1306_WriteString("TX er: ",  Font_6x8,White);

	  ssd1306_SetCursor(60, 10);
	  ssd1306_WriteString("TX OK: ",  Font_6x8,White);

	  ssd1306_SetCursor(103, 0);
	  sprintf(str, "%d ", lost_pckts);
	  ssd1306_WriteString(str,  Font_6x8,White);

	  ssd1306_SetCursor(103, 10);
	  sprintf(str, "%d ", sent_pack);
	  ssd1306_WriteString(str,  Font_6x8,White);
	  }
	  else{
		  ssd1306_SetCursor(30, 10);
		  ssd1306_WriteString("ON ",  Font_6x8,Black);
		  ssd1306_SetCursor(30, 10);
		  ssd1306_WriteString("OFF",  Font_6x8,White);
	  }
//
	  ssd1306_UpdateScreen();
  }

   // then pull CE pin to HIGH, and the nRF24 will start a receive...
//  /* USER CODE END 2 */
//
//  /* Infinite loop */
//  /* USER CODE BEGIN WHILE */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
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
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 6;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
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
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000;
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
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);
  /* NOTE : This function should not be modified. When the callback is needed,
            function HAL_ADC_ConvCpltCallback must be implemented in the user file.
   */
  temp[0] += a[0];
  temp[1] += a[1];
  temp[2] += a[2];
  temp[3] += a[3];
  temp[4] += a[4];
  temp[5] += a[5];
  count++;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
