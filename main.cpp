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
#include "stdint.h" //To guarantee the size of variables.
#include <stdio.h> // Include the standard input/output library for printf function
#include <adc_temp.h> //CAN filters and EEPROM commands
#include "TMC5160.h"
#include "EEPROM.h"

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
ADC_HandleTypeDef hadc3;

CAN_HandleTypeDef hcan;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC3_Init(void);
static void MX_CAN_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

void CAN_Transmit(TMC5160_SPI** motors,
		CAN_HandleTypeDef *hcan,
		CAN_TxHeaderTypeDef *CAN_TxHeader,
		uint32_t *CAN_TxMailbox);
void CAN_Filter(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef* CAN_TxHeader); //Initializes the CANBus filter for the board.
// A method to enable all 6 motors at once
void enableAll(TMC5160_SPI** motors);
// A method to disable all 6 motors at once
void disableAll(TMC5160_SPI** motors);
// A method to initialize all 6 motor at once
void beginAll(TMC5160_SPI** motors,
		const TMC5160::PowerStageParameters &powerParams,
		const TMC5160::MotorParameters &motorParams,
		TMC5160::MotorDirection stepperDirection);


uint8_t getTemp(void); //Returns a value specifying the temperature in Celsius.

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC3_Init();
  MX_CAN_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  EEPROM_SPI eeprom(&hspi1, CSN_EEPROM_Pin, CSN_EEPROM_GPIO_Port);

  CAN_TxHeaderTypeDef CAN_TxHeader; //The transmission header.
  CAN_RxHeaderTypeDef CAN_RxHeader; //The receiver header.
  uint32_t CAN_TxMailbox = 0;
  uint8_t CAN_RxData[CAN_DATA_SIZE]; //The receiving data variable.

  CAN_Filter(&hcan,&CAN_TxHeader); // Initializing the CANbus filter
  HAL_CAN_Start(&hcan); //Start the CANbus

  uint32_t clock_freq = 12000000;
  TMC5160_SPI motor1(&hspi1, CSN_MOTOR_1_Pin, CSN_MOTOR_1_GPIO_Port, clock_freq, CAN_MOTOR_ID_1);
  TMC5160_SPI motor2(&hspi1, CSN_MOTOR_2_Pin, CSN_MOTOR_2_GPIO_Port, clock_freq, CAN_MOTOR_ID_2);
  TMC5160_SPI motor3(&hspi1, CSN_MOTOR_3_Pin, CSN_MOTOR_3_GPIO_Port, clock_freq, CAN_MOTOR_ID_3);
  TMC5160_SPI motor4(&hspi1, CSN_MOTOR_4_Pin, CSN_MOTOR_4_GPIO_Port, clock_freq, CAN_MOTOR_ID_4);
  TMC5160_SPI motor5(&hspi1, CSN_MOTOR_5_Pin, CSN_MOTOR_5_GPIO_Port, clock_freq, CAN_MOTOR_ID_5);
  TMC5160_SPI motor6(&hspi1, CSN_MOTOR_6_Pin, CSN_MOTOR_6_GPIO_Port, clock_freq, CAN_MOTOR_ID_6);
  TMC5160_SPI* motors[6] = {&motor1, &motor2, &motor3, &motor4, &motor5, &motor6};

  // This sets the motor & driver parameters /!\ run the configWizard for your driver and motor for fine tuning !
   TMC5160::PowerStageParameters powerStageParams; // defaults.
   TMC5160::MotorParameters motorParams;

   /* Infinite loop */
   /* USER CODE BEGIN WHILE */
   motorParams.globalScaler = 46;
   motorParams.irun = 23; //To give 2.8A RMS coil current
   motorParams.ihold = 10; // IHold 70% of IRUN or lower (pg 111)
   powerStageParams.bbmTime = 2;

   disableAll(motors);
   enableAll(motors);

   beginAll(motors, powerStageParams,motorParams,TMC5160::NORMAL_MOTOR_DIRECTION);

   enableAll(motors);
   uint8_t updates = 0; //The number of updates to send
   uint8_t update_limit = sizeof(motors); //The value at which data will be sent
   uint8_t i = 0; //An iteration variable

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  //Do nothing until a CAN message comes in.
	  while(HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) == 0){}


	  //Get CAN message.
	  HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &CAN_RxHeader, CAN_RxData);

	  //Adjust motors relative to CAN message
	  motor1.CAN_IN(&CAN_RxHeader,CAN_RxData);
	  motor2.CAN_IN(&CAN_RxHeader,CAN_RxData);
	  motor3.CAN_IN(&CAN_RxHeader,CAN_RxData);
	  motor4.CAN_IN(&CAN_RxHeader,CAN_RxData);
	  motor5.CAN_IN(&CAN_RxHeader,CAN_RxData);
	  motor6.CAN_IN(&CAN_RxHeader,CAN_RxData);

	  for(i = 0; i < 6 ; i++)
	  {
		  if(motors[i]->CAN_SendStatus){updates++;}
	  }

	  //Checking if it is time to send
	  if(updates == update_limit)
	  {
		  CAN_Transmit(motors, &hcan, &CAN_TxHeader, &CAN_TxMailbox);
	  }
	  updates = 0; //Resetting updates to avoid false sending


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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC34;
  PeriphClkInit.Adc34ClockSelection = RCC_ADC34PLLCLK_DIV1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc3.Init.Resolution = ADC_RESOLUTION_10B;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc3, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

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
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
	  __HAL_RCC_GPIOC_CLK_ENABLE();
	  __HAL_RCC_GPIOF_CLK_ENABLE();
	  __HAL_RCC_GPIOA_CLK_ENABLE();
	  __HAL_RCC_GPIOB_CLK_ENABLE();

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOC, CSN_MOTOR_6_Pin|CSN_MOTOR_5_Pin, GPIO_PIN_SET);

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOA, CSN_MOTOR_4_Pin|CSN_MOTOR_3_Pin|CSN_MOTOR_2_Pin|CSN_MOTOR_1_Pin
	                          |CSN_EEPROM_Pin, GPIO_PIN_SET);

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOB, EEPROM_WP_Pin|EEPROM_HOLD_Pin, GPIO_PIN_RESET);

	  /*Configure GPIO pins : CSN_MOTOR_6_Pin CSN_MOTOR_5_Pin */
	  GPIO_InitStruct.Pin = CSN_MOTOR_6_Pin|CSN_MOTOR_5_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	  /*Configure GPIO pins : CSN_MOTOR_4_Pin CSN_MOTOR_3_Pin CSN_MOTOR_2_Pin CSN_MOTOR_1_Pin
	                           CSN_EEPROM_Pin */
	  GPIO_InitStruct.Pin = CSN_MOTOR_4_Pin|CSN_MOTOR_3_Pin|CSN_MOTOR_2_Pin|CSN_MOTOR_1_Pin
	                          |CSN_EEPROM_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /*Configure GPIO pins : EEPROM_WP_Pin EEPROM_HOLD_Pin */
	  GPIO_InitStruct.Pin = EEPROM_WP_Pin|EEPROM_HOLD_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	  /*Configure GPIO pins : CAN_ADD_0_Pin CAN_ADD_1_Pin CAN_ADD_2_Pin CAN_ADD_3_Pin
	                           MOTOR_MODE_1_Pin MOTOR_MODE_2_Pin MOTOR_MODE_3_Pin MOTOR_MODE_4_Pin
	                           MOTOR_MODE_5_Pin MOTOR_MODE_6_Pin */
	  GPIO_InitStruct.Pin = CAN_ADD_0_Pin|CAN_ADD_1_Pin|CAN_ADD_2_Pin|CAN_ADD_3_Pin
	                          |MOTOR_MODE_1_Pin|MOTOR_MODE_2_Pin|MOTOR_MODE_3_Pin|MOTOR_MODE_4_Pin
	                          |MOTOR_MODE_5_Pin|MOTOR_MODE_6_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


uint8_t getTemp(void)
{
    // This function returns a value specifying the board temperature in Celsius.
    uint32_t tempSum = 0;

    HAL_ADC_Start(&hadc3); // Starting the ADC reading
    HAL_ADC_PollForConversion(&hadc3, HAL_MAX_DELAY); // Waiting for the conversion to finish

    // Converting the ADC value using the linear formula and casting it as a uint32_t.
    tempSum += (uint32_t)(ADC_TEMP_SLOPE * HAL_ADC_GetValue(&hadc3) + ADC_TEMP_INTER);

    for (int i = 0; i < 4; ++i) {
        HAL_Delay(5); // Delay to allow for next conversion
        HAL_ADC_Start(&hadc3); // Starting the ADC reading again
        HAL_ADC_PollForConversion(&hadc3, HAL_MAX_DELAY); // Waiting for the conversion to finish
        tempSum += (uint32_t)(ADC_TEMP_SLOPE * HAL_ADC_GetValue(&hadc3) + ADC_TEMP_INTER);
    }

    // Calculate average temperature
    uint16_t tempAvg = tempSum / 5;
    return tempAvg;
}


// A function to enable all motors
void enableAll(TMC5160_SPI** motors) {
    for(int i = 0; i < 6; i++)
        motors[i]->enable();
}

// A function to disable all motors
void disableAll(TMC5160_SPI** motors) {
    for(int i = 0; i < 6; i++)
        motors[i]->disable();
}

// Sends the CAN messages for the motors.
void CAN_Transmit(TMC5160_SPI** motors, CAN_HandleTypeDef *hcan,
		CAN_TxHeaderTypeDef *CAN_TxHeader,
		uint32_t *CAN_TxMailbox)
{
	uint8_t CAN_TxData[CAN_DATA_SIZE];
	uint32_t CANintValue = 0;
	float CANfloatValue = 0;
	for(int i = 0; i < 6; i++)
	{
		if ((motors[i]->CAN_MotorTxData != CAN_NO_DATA) && (motors[i]->CAN_SendStatus))
		{
			CANfloatValue = motors[i]->CAN_MotorTxData;
			CANintValue = reinterpret_cast<uint32_t &>(CANfloatValue);

			CAN_TxData[0] = static_cast<uint8_t>(CANintValue >> 24);
			CAN_TxData[1] = static_cast<uint8_t>(CANintValue >> 16);
			CAN_TxData[2] = static_cast<uint8_t>(CANintValue >> 8);
			CAN_TxData[3] = static_cast<uint8_t>(CANintValue);

			//Adjusting the CAN_TxHeader
			CAN_TxHeader->ExtId &= 0x0000F000; //Eliminate lingering data from other transmissions
			CAN_TxHeader->ExtId |= motors[i]->CAN_MotorTxHeader; //Putting in the motor ID and command ID

			HAL_CAN_AddTxMessage(hcan, CAN_TxHeader, CAN_TxData, CAN_TxMailbox);
			motors[i]->CAN_SendStatus = false;

			}
		}
}
void CAN_Filter(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef* CAN_TxHeader)
{	//This function initializes the CAN filter for the board.
	//The CANID ports are named for their respective address bits, i.e., 0 to the 0th bit.

	//Setting up the TxHeader
	 CAN_TxHeader->IDE = CAN_ID_EXT; //Extended identifier, not the standard length.
	 CAN_TxHeader->RTR = CAN_RTR_DATA; //Specifying data frames, not remote frames.
	 CAN_TxHeader->DLC = CAN_DATA_SIZE;// CAN_SIZE_DATA; //The data size (5 bytes)
	 CAN_TxHeader->ExtId = 0; //Needs to be changed depending on the frame

	  //Initializing a filter for CAN messages.
	 uint32_t filter_ID_low = CAN_ID_LOW;
	 uint32_t filter_mask_low = CAN_MASK_LOW;

	 uint32_t filter_ID_high = CAN_ID_HIGH;
	 uint32_t filter_mask_high = CAN_MASK_HIGH;


	 //Using bitshifting to assign the receiver ID bits in the filter.

	 filter_ID_low |= (uint32_t)(HAL_GPIO_ReadPin(CAN_ADD_0_GPIO_Port, CAN_ADD_0_Pin) << 15);
	 filter_ID_high |= (uint32_t)(HAL_GPIO_ReadPin(CAN_ADD_1_GPIO_Port, CAN_ADD_1_Pin));
	 filter_ID_high |= (uint32_t)(HAL_GPIO_ReadPin(CAN_ADD_2_GPIO_Port, CAN_ADD_2_Pin) << 1);
	 filter_ID_high |= (uint32_t)(HAL_GPIO_ReadPin(CAN_ADD_3_GPIO_Port, CAN_ADD_3_Pin) << 2);

	 //Setting the extended transmission header based on the CAN pins
	 CAN_TxHeader->ExtId |= (uint32_t)(HAL_GPIO_ReadPin(CAN_ADD_0_GPIO_Port, CAN_ADD_0_Pin) << 12);
	 CAN_TxHeader->ExtId |= (uint32_t)(HAL_GPIO_ReadPin(CAN_ADD_1_GPIO_Port, CAN_ADD_1_Pin) << 13);
	 CAN_TxHeader->ExtId |= (uint32_t)(HAL_GPIO_ReadPin(CAN_ADD_2_GPIO_Port, CAN_ADD_2_Pin) << 14);
	 CAN_TxHeader->ExtId |= (uint32_t)(HAL_GPIO_ReadPin(CAN_ADD_3_GPIO_Port, CAN_ADD_3_Pin) << 15);

	 CAN_FilterTypeDef CAN_FILTER_CONFIG; //Declaring the filter structure.
	 CAN_FILTER_CONFIG.FilterFIFOAssignment = CAN_FILTER_FIFO0; //Choosing the FIFO0 set.
	 CAN_FILTER_CONFIG.FilterIdHigh = filter_ID_high;
	 CAN_FILTER_CONFIG.FilterIdLow = filter_ID_low;
	 CAN_FILTER_CONFIG.FilterMaskIdHigh = filter_mask_high;
	 CAN_FILTER_CONFIG.FilterMaskIdLow = filter_mask_low;
	 CAN_FILTER_CONFIG.FilterMode = CAN_FILTERMODE_IDMASK; //Using the mask mode to ignore certain bits.
	 CAN_FILTER_CONFIG.FilterScale = CAN_FILTERSCALE_32BIT; //Using the extended ID so 32bit filters.
	 CAN_FILTER_CONFIG.FilterActivation = ENABLE; //Enabling the filter.
	 HAL_CAN_ConfigFilter(hcan, &CAN_FILTER_CONFIG);

}

// A function to initialize all motors
void beginAll(TMC5160_SPI** motors,
		const TMC5160::PowerStageParameters &powerParams,
		const TMC5160::MotorParameters &motorParams,
		TMC5160::MotorDirection stepperDirection) {

	int mtrType[6] = {
	    HAL_GPIO_ReadPin(MOTOR_MODE_1_GPIO_Port, MOTOR_MODE_1_Pin),
	    HAL_GPIO_ReadPin(MOTOR_MODE_2_GPIO_Port, MOTOR_MODE_2_Pin),
	    HAL_GPIO_ReadPin(MOTOR_MODE_3_GPIO_Port, MOTOR_MODE_3_Pin),
	    HAL_GPIO_ReadPin(MOTOR_MODE_4_GPIO_Port, MOTOR_MODE_4_Pin),
	    HAL_GPIO_ReadPin(MOTOR_MODE_5_GPIO_Port, MOTOR_MODE_5_Pin),
	    HAL_GPIO_ReadPin(MOTOR_MODE_6_GPIO_Port, MOTOR_MODE_6_Pin)
	};

    for(int i = 0; i < 6; i++){
    	if (mtrType[i] == 0)
			motors[i]->begin(powerParams, motorParams, stepperDirection, TMC5160::DC_BRUSHED);
    	else if (mtrType[i] == 1)
    		motors[i]->begin(powerParams, motorParams, stepperDirection, TMC5160::STEPPER);
    }
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
