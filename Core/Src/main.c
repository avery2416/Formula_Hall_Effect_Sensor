/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "gpio_setup.h"
#include "tim_setup.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "hallEffectSensor.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
CAN_HandleTypeDef 	CanHandle;
UART_HandleTypeDef 	hUART2;
CAN_RxHeaderTypeDef	RxHeader;		//CAN Bus Transmit Header
CAN_TxHeaderTypeDef TxHeader; 		//CAN Bus Receive Header
uint8_t 			CanRX[8];  		//CAN Bus Receive Buffer
uint8_t				CanTX[8];		//CAN Bus Transmit Buffer
CAN_FilterTypeDef 	FilterConfig; 	//CAN Bus Filter
uint32_t 			CanMailbox; 	//CAN Bus Mail box variable

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void SystemClock_Config(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN_Init(void);
static void CAN_Config_Filter(void);
void CAN_Send_RPM(uint16_t rpm);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Create hallEffectSensor struct
hallEffectSensor hfs;


/* USER CODE END 0 */


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin==hfs.interruptPin) hfs.isReady = 1;
	return;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){
	if(htim==hfs.timeOutTimer) hfs.rpm = 0;
	return;
}

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
	MX_TIM2_Init();
	MX_TIM6_Init();
	/* USER CODE BEGIN 2 */

	// Start hall effect timer
	if(HAL_TIM_Base_Start_IT(&htim2)!= HAL_OK) Error_Handler();

	// Initialize the HallEffectSensor Structure
	if(hallEffectInit(&hfs, 1, 1, 10, &htim2, &ht im6, 1, GPIO_PIN_7)!= HAL_OK) Error_Handler();

	// Start can communication
	// Start CAN communication
	if (HAL_CAN_Start(&hcan) != HAL_OK) Error_Handler();

	// Apply the CAN filter configuration
	CAN_Config_Filter();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
	    // Toggle an LED to show activity
	    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

	    // Check CAN status
	    uint32_t canError = HAL_CAN_GetError(&hcan);
	    if (canError != HAL_CAN_ERROR_NONE) {
	        // Reset CAN if there's an error
	        HAL_CAN_Stop(&hcan);
	        HAL_Delay(100);
	        HAL_CAN_Start(&hcan);
	    }

	    // Perform RPM calculation with hall effect flag
	    hallEffectCalculator(&hfs, &huart2);

	    // Transmit RPM over CAN (only if no errors)
	    if (canError == HAL_CAN_ERROR_NONE) {
	        CAN_Send_RPM(hfs.rpm);
	    }

	    // Delay to not flood CAN transmissions
//	    HAL_Delay(500);
	}
	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */

	/* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */

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
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
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

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;

	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) Error_Handler();
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

	if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

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
	hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
	hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
	hcan.Init.TimeTriggeredMode = DISABLE;
	hcan.Init.AutoBusOff = ENABLE;
	hcan.Init.AutoWakeUp = ENABLE;
	hcan.Init.AutoRetransmission = ENABLE;
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
  * @brief  Configures the CAN.
  * @param  None
  * @retval None
  */
static void CAN_Config_Filter(void)
{
	FilterConfig.FilterBank = 0;
	FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	FilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	FilterConfig.FilterIdHigh = 0x0000;
	FilterConfig.FilterIdLow = 0x0000;
	FilterConfig.FilterMaskIdHigh = 0x0000;
	FilterConfig.FilterMaskIdLow = 0x0000;
	FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	FilterConfig.FilterActivation = ENABLE;
//	FilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan, &FilterConfig) != HAL_OK) Error_Handler();
}

/**
  * @brief  Prepares RPM data for CAN transmission.
  * @param  uint16_t rpm
  * @retval None
  */
void CAN_Send_RPM(uint16_t rpm) {
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t CanTX[8];
    uint32_t CanMailbox;

    TxHeader.StdId = 0x123;         // CAN ID
    TxHeader.ExtId = 0;
    TxHeader.IDE = CAN_ID_STD;      // Standard ID
    TxHeader.RTR = CAN_RTR_DATA;    // Data frame
    TxHeader.DLC = 2;               // 2 bytes for RPM
    TxHeader.TransmitGlobalTime = DISABLE;

    // Pack RPM into 2 bytes
    CanTX[0] = (uint8_t)(rpm >> 8);    // MSB
    CanTX[1] = (uint8_t)(rpm & 0xFF);  // LSB

    // Check if there's a free mailbox before trying to send
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0) {
        // Only try to send if mailbox is available
        HAL_CAN_AddTxMessage(&hcan, &TxHeader, CanTX, &CanMailbox);
        // Note: Not checking return value to avoid Error_Handler
    }
    // Simply return if no mailbox is available, we'll try again in the next cycle
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
