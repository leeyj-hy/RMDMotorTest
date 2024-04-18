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
//#include "usbd_cdc_if.h"
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
CAN_HandleTypeDef hcan;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
CAN_FilterTypeDef  sFilterConfig;
CAN_TxHeaderTypeDef   TxHeader;
CAN_RxHeaderTypeDef   RxHeader;

uint8_t  M_READ_MULTITURN_ANGLE = 0x92;
uint8_t  M_MOTOR_OFF_COMMAND = 0X80;

uint8_t C_DEFAULT = 0X55;
uint8_t C_MOTOR_LOCK = 0X35;
uint8_t C_MOTOR_UNLOCK = 0X95;

uint32_t last_led_time = 0;

uint32_t last_tx2_time = 0;
uint8_t TxData[8];
uint8_t RxData[8];
uint8_t TxDataTest[8];
uint32_t TxMailbox;

uint32_t defaultTimeout = 100;

uint8_t CANMsgRxFlg = 0;
uint8_t rx_buffer[8];
uint8_t rx_data[8];

int rcv_index = 0;
int rx_index = 0;
uint8_t rcv_data=0;
uint8_t rcv_head[2];
int rcv_count = 0;
int rcv_ready = 0;
int rxISRFlag = 0;

uint8_t opMod = 0;
int32_t goalPosition = 0;
int16_t MaxSpeed = 100;

union{
	uint8_t byteVal[4];
	uint32_t int32Val;
}val4byte;

typedef struct{
	uint32_t motorId;
	int32_t goalPosition;
	int16_t maxSpeed;
  uint32_t lastPosReturnTime;
}RMD_Motor;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

RMD_Motor M1, M2, M3, M4, M5, M6, M7, M8, M9;

int SendCANDataWCommand(uint8_t CANcommand);
int SendCANDataPos(uint16_t CANID,int32_t m_pos, int16_t m_Mspd);
int SendUARTDataCommand(uint8_t command);
int _write(int file, char *ptr, int len)
{
	//HAL_UART_Transmit(&huart1, (uint8_t *)ptr, (uint16_t)len, 100);
	//CDC_Transmit_FS((uint8_t *)ptr, 8);
	return (len);
}



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
{
	if (HAL_CAN_GetRxMessage(CanHandle, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
		{
			/* Reception Error */
			Error_Handler();
		}
	CANMsgRxFlg = 1;
	//printf("StdID: %04lx, IDE: %ld, DLC: %ld\r\n", RxHeader.StdId, RxHeader.IDE, RxHeader.DLC);
	  //printf("Data: %d %d %d %d %d %d %d %d\r\n", RxData[0], RxData[1], RxData[2], RxData[3], RxData[4], RxData[5], RxData[6], RxData[7]);
	  //CDC_Transmit_FS(RxData, 8);

	//HAL_UART_Transmit(&huart2,RxData, sizeof(RxData), HAL_MAX_DELAY);
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_CAN_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, &rcv_data, 1);

  //configure transmission process

  TxHeader.StdId = 0x100;                 // Standard Identifier, 0 ~ 0x7FF
  TxHeader.ExtId = 0x00;                  // Extended Identifier, 0 ~ 0x1FFFFFFF
  TxHeader.RTR = CAN_RTR_DATA;            // ?��?��?��?�� 메세�????????�� ?��?��?�� ???��, DATA or REMOTE
  TxHeader.IDE = CAN_ID_STD;              // ?��?��?��?�� 메세�????????�� ?��별자 ???��, STD or EXT
  TxHeader.DLC = 8;                       // ?��?�� ?��?��?�� 길이, 0 ~ 8 byte
  TxHeader.TransmitGlobalTime = DISABLE;  // ?��?��?�� ?��?�� ?��?��?�� ?�� timestamp counter 값을 capture.

  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;              // 0x00000000 = 모든 ID�??????? 받아?��?��겠다
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;              // CAN2?�� FilterBank?��?�� ?���???????, CAN2�??????? ?��?��?��?���??????? FilterBank�??????? SlaveStartFilterBank보다 ?���??????? ?��?��?��?�� ?��.

  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    Error_Handler();
  }

  TxData[0] = M_READ_MULTITURN_ANGLE;
  TxData[1] = 0;
  TxData[2] = 0;
  TxData[3] = 0;
  TxData[4] = 0;
  TxData[5] = 0;
  TxData[6] = 0;
  TxData[7] = 0;

  TxDataTest[0] = 0x00;
  TxDataTest[1] = 0x01;
  TxDataTest[2] = 0x02;
  TxDataTest[3] = 0x03;
  TxDataTest[4] = 0x04;
  TxDataTest[5] = 0x05;
  TxDataTest[6] = 0x06;
  TxDataTest[7] = 0x07;

  if(HAL_CAN_Start(&hcan)!=HAL_OK)
    {
  	  Error_Handler();
    }


  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
   {
     /* Notification Error */
     Error_Handler();
   }
  opMod = 1; //start loop
/*
  for(int i = 0; i<50; i++)
    {
  	  if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
		  {
			 Error_Handler();
		  }
    }
  */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//	  if(HAL_GetTick() - last_tx2_time > 100){
//		  if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
//			  {
//				 Error_Handler();
//			  }
//		  //HAL_UART_Transmit(&huart2,TxDataTest, sizeof(TxDataTest), HAL_MAX_DELAY);
//		  //SendData(M_READ_MULTITURN_ANGLE);
//		  last_tx2_time = HAL_GetTick();
//	  }

	  if(rxISRFlag){
		  memcpy(&rx_data, &rx_buffer, 8);
		  rxISRFlag = 0;
		  switch(rx_data[2]){	//rx_data is from USART2

			  case 0x34:	//motor mod change to set
				  opMod = 2;
				  break;

			  case 0x94:	//motor mod change to reset
				  opMod = 3;
				  break;

	//		  case 0x92:	//read motor pos
	//			  opMod = 4;
	//			  break;


			  case 0x55:
				  opMod = 1;
				  break;

			  case 0xa4:
				  opMod = 5;
				  memcpy(&val4byte.byteVal, &rx_data[4], 4);
				  goalPosition = val4byte.int32Val;
				  break;

			  default:
				  opMod = 1;
				  break;
		  }
	  }



	  switch(opMod){
	  case 1:	//default mod
		  if(HAL_GetTick() - last_led_time > 500){
			  if(HAL_GPIO_ReadPin(led_GPIO_Port, led_Pin)){
				  HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_RESET);
			  }
			  else{
				  HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_SET);
			  }
			  last_led_time = HAL_GetTick();

		  }

		  if(CANMsgRxFlg){
				  CANMsgRxFlg = 0;

			  }
		  break;
	  case 2:	//motor torque on mod
		  SendCANDataWCommand(0x80);
		  
		  SendUARTDataCommand(0x35);
		  opMod = 5;
		  break;
	  case 3:	//motor torque off mod
		  // TxData[0] = 0xa1;
		  // TxData[1] = 0;
		  // TxData[2] = 0;
		  // TxData[3] = 0;
		  // TxData[4] = 0;
		  // TxData[5] = 0;
		  // TxData[6] = 0;
		  // TxData[7] = 0;

		  // if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
			//   {
			// 	 Error_Handler();
			//   }
		  if(SendCANDataWCommand(0xa1))
      {
        Error_Handler();
      }
		  SendUARTDataCommand(C_MOTOR_UNLOCK);
		  opMod = 4;
		  break;
	  case 4:	//motor position reading mod

      
		  TxData[0] = 0x92;
		  TxData[1] = 0;
		  TxData[2] = 0;
		  TxData[3] = 0;
		  TxData[4] = 0;
		  TxData[5] = 0;
		  TxData[6] = 0;
		  TxData[7] = 0;

			  if(HAL_GetTick() - last_tx2_time > 100){
		  		  if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
		  			  {
		  				 Error_Handler();
		  			  }
		  		  if(CANMsgRxFlg){
						  CANMsgRxFlg = 0;

					  }
		  		  TxData[0] = 0xff;
		  		  TxData[1] = 0xfe;
		  		  TxData[2] = 0x92;
		  		  TxData[3] = 0x00;
		  		  memcpy(&TxData[4], &RxData[1], 4);
		  		  HAL_UART_Transmit(&huart2,TxData, sizeof(TxData), HAL_MAX_DELAY);
		  		  //SendData(M_READ_MULTITURN_ANGLE);
		  		  last_tx2_time = HAL_GetTick();
		  	  }

		  break;
	  case 5:	//motor position writing mod
		  if(HAL_GetTick() - last_led_time > 100){
			  if(HAL_GPIO_ReadPin(led_GPIO_Port, led_Pin)){
				  HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_RESET);
			  }
			  else{
				  HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_SET);
			  }
			  last_led_time = HAL_GetTick();

		  }

			  if(HAL_GetTick() - last_tx2_time > 100){
				  if (SendCANDataPos(7, goalPosition, MaxSpeed) != HAL_OK)
					  {
						 Error_Handler();
					  }
				  if(CANMsgRxFlg){
						  CANMsgRxFlg = 0;

					  }
				  //HAL_UART_Transmit(&huart2,RxData, sizeof(RxData), HAL_MAX_DELAY);
				  //SendData(M_READ_MULTITURN_ANGLE);
				  last_tx2_time = HAL_GetTick();
			  }
		  break;
	  default:
		  opMod = 1;
		  break;
	  }

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hcan.Init.TimeSeg1 = CAN_BS1_6TQ;
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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : user_button_Pin */
  GPIO_InitStruct.Pin = user_button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(user_button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : led_Pin */
  GPIO_InitStruct.Pin = led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(led_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int SendCANDataWCommand(uint8_t CANcommand){

	uint8_t errRet = 1;
	uint8_t data[8];

	data[0] = CANcommand;

	for(int i = 1; i<8; i++)
		data[i] = 0;
	/*
	for(int i = 1; i<10; i++){
		TxHeader.StdId = 0x140 | i;
		if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, data, &TxMailbox) == HAL_OK)
			{
				errRet = 0;
			}
	}
	*/

	TxHeader.StdId = 0x147;
	if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, data, &TxMailbox) == HAL_OK)
		{
			errRet = 0;
		}
	return errRet;
}

int SendCANDataPos(uint16_t CANID, int32_t m_pos, int16_t m_Mspd){
	uint8_t errRet = 1;
	uint8_t data[8];
	TxHeader.StdId = 0x140 & CANID;
	data[0] = 0xa4;
	data[1] = 0;
	data[2] = m_Mspd;
	data[3] = m_Mspd >> 8;
	data[4] = m_pos;
	data[5] = m_pos >> 8;
	data[6] = m_pos >> 16;
	data[7] = m_pos >> 32;

	if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, data, &TxMailbox) == HAL_OK)
		{
			errRet = 0;
		}
	return errRet;
}
int SendUARTDataCommand(uint8_t command){
	uint8_t errRet = 1;
	uint8_t data[8];
	data[0] = 0xff;
	data[1] = 0xfe;
	data[2] = command;
	for(int i = 3; i<8; i++)
		data[i] = 0;

   if(HAL_UART_Transmit(&huart2, data ,sizeof(data), HAL_MAX_DELAY) == HAL_OK){
	   errRet = 0;
   }

   return errRet;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
   if(huart->Instance == USART2)
   {
      if(!rxISRFlag)
      switch(rcv_index){
        case 0:
          if(rcv_data ==0xff && rcv_ready == 0)
          {
            //HAL_GPIO_TogglePin(led_GPIO_Port, led_Pin);
            rcv_index = 1;
            rcv_ready = 0;
            rx_buffer[0] = rcv_data;
          }
          else
            rcv_index = 0;
          break;
        case 1:
			  if(rcv_data ==0xfe && rcv_ready == 0)
			  {
				//HAL_GPIO_TogglePin(led_GPIO_Port, led_Pin);
				rcv_index = 2;
				rcv_ready = 1;
				rx_buffer[1] = rcv_data;
				rx_index = rcv_index;
			  }
			  else
				rcv_index = 0;
        	break;
        case 2:
        	rx_buffer[rx_index] = rcv_data;
        	rx_index++;
          if(rx_index > 7){
        	  rcv_index = 0;
        	  rx_index = 0;
        	  rxISRFlag = 1;
        	  rcv_ready = 0;
          }
          break;
        default:
          rcv_index = 0;
          rx_index = 0;
          rcv_ready = 0;
          break;
      }
      HAL_UART_Receive_IT(&huart2, &rcv_data, 1);
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
