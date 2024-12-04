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
#include "stm32l0xx_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// DFPlayer Mini Command Defines
#define DFPLAYER_CMD_NEXT          0x01  // Play next track
#define DFPLAYER_CMD_PREV          0x02  // Play previous track
#define DFPLAYER_CMD_PLAY_TRACK    0x03  // Play specific track
#define DFPLAYER_CMD_INC_VOL       0x04  // Increase volume
#define DFPLAYER_CMD_DEC_VOL       0x05  // Decrease volume
#define DFPLAYER_CMD_SET_VOL       0x06  // Set volume (0-30)
#define DFPLAYER_CMD_SET_EQ        0x07  // Set EQ mode (0-Normal, 1-Pop, 2-Rock, etc.)
#define DFPLAYER_CMD_PLAYBACK_MODE 0x08  // Set playback mode (single loop, repeat all, etc.)
#define DFPLAYER_CMD_RESET         0x0C  // Reset DFPlayer Mini
#define DFPLAYER_CMD_PLAY          0x0D  // Resume playback
#define DFPLAYER_CMD_PAUSE         0x0E  // Pause playback
#define DFPLAYER_CMD_LOOP_TRACK    0x19  // Loop current track

// Query Commands
#define DFPLAYER_CMD_QUERY_STATUS  0x42  // Query status
#define DFPLAYER_CMD_QUERY_VOL     0x43  // Query volume
#define DFPLAYER_CMD_QUERY_EQ      0x44  // Query EQ mode
#define DFPLAYER_CMD_QUERY_PLAY    0x45  // Query playback status
#define DFPLAYER_CMD_QUERY_FILES   0x48  // Query total number of files

// Advanced Playback
#define DFPLAYER_CMD_PLAY_FOLDER   0x12  // Play track from specific folder
#define DFPLAYER_CMD_LOOP_FOLDER   0x17  // Loop all tracks in a folder
#define DFPLAYER_CMD_RANDOM_PLAY   0x18  // Random playback

// Stop Playback
#define DFPLAYER_CMD_STOP_PLAYBACK 0x16  // Stop playback

#define BLINK_INTERVAL 50 // 250ms interval
#define CHECK_INTERVAL 200 // Interval in milliseconds

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint32_t last_check_time = 0; // Tracks the last time the playback was checked
uint32_t last_blink_time = 0; // Tracks last blink time
uint32_t seed = 1; // Initialize the seed
uint8_t led_state = 0;        // Tracks which LED is ON
uint8_t explosion = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void LED_Init(void);
void Blink_LEDs(void);
void DFPlayer_SendCommand(uint8_t, uint8_t, uint8_t);
void checkPlaybackStatus(void);
uint8_t DFPlayer_CheckPlaybackStatus(void);
uint8_t getRandomNumber(void);
uint8_t status = 0;
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  DFPlayer_SendCommand(DFPLAYER_CMD_RESET , 0, 0); // Reset DFPlayer Mini
  HAL_Delay(1000);
  DFPlayer_SendCommand(DFPLAYER_CMD_SET_VOL, 0, 30);
  HAL_Delay(1000);
  DFPlayer_SendCommand(DFPLAYER_CMD_PLAY_TRACK, 0x00, getRandomNumber());

  explosion = 1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //checkPlaybackStatus();
	  Blink_LEDs();
	  //HAL_GPIO_WritePin(GPIOA, LED_GREEN_Pin, status);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart2.Init.BaudRate = 9600;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_GREEN_Pin|LED_AMBER_Pin|LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_GREEN_Pin LED_AMBER_Pin LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin|LED_AMBER_Pin|LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void DFPlayer_SendCommand(uint8_t command, uint8_t param1, uint8_t param2)
	{
		uint8_t packet[10];
		uint16_t arg = (param1 << 8) | param2;

		packet[0] = 0x7E;             // Start byte
		packet[1] = 0xFF;             // Version
		packet[2] = 0x06;             // Data length
		packet[3] = command;          // Command byte
		packet[4] = 0x00;             // No feedback
		packet[5] = (arg >> 8) & 0xFF; // Parameter high byte
		packet[6] = arg & 0xFF;       // Parameter low byte
		uint16_t checksum = 0 - (packet[1] + packet[2] + packet[3] + packet[4] + packet[5] + packet[6]);
		packet[7] = (checksum >> 8) & 0xFF; // Checksum high byte
		packet[8] = checksum & 0xFF;        // Checksum low byte
		packet[9] = 0xEF;             // End byte

		// Send packet over UART
		HAL_UART_Transmit(&huart2, packet, 10, HAL_MAX_DELAY);
	}

uint8_t DFPlayer_CheckPlaybackStatus(void)
	{
		uint8_t status_packet[10];
		uint8_t status1 = 0x00; // Default to an invalid value
		// Send the status query command
		DFPlayer_SendCommand(0x42, 0x00, 0x00);
		if (HAL_UART_Receive(&huart2, status_packet, 10, 100) == HAL_OK) 		// Wait for the response from the DFPlayer Mini
			{
				if (status_packet[3] == 0x42 && status_packet[0] == 0x7E && status_packet[9] == 0xEF) 			// Ensure the response is valid and matches the query
					{
						status1 = status_packet[6]; // Extract the playback status (0x01 or 0x00)
					}
			}
		return status1; // Return playback status (0x01: Playing, 0x00: Stopped)
	}

void checkPlaybackStatus(void)
	{
		uint32_t current_time = HAL_GetTick();    						 	// Get the current system time in milliseconds
		if ((current_time - last_check_time) >= CHECK_INTERVAL)     		// Check if the interval has elapsed
			{
				last_check_time = current_time; 							// Update the last check time
				status = DFPlayer_CheckPlaybackStatus();        			// Query the playback status
				if (status == 0x00)
					{
						explosion = 0;
					}
			}
	}

uint8_t getRandomNumber()
	{
    	seed = (2*HAL_GetTick() * 1103515245 + 12345) & 0x7FFFFFFF; // LCG formula
    	return (uint8_t)((seed % 3) + 1); // Random number between 1 and 3
	}

void Blink_LEDs(void)
	{
		if (explosion == 1)
			{
				uint32_t current_time = HAL_GetTick(); // Get the current system time in milliseconds
				if ((current_time - last_blink_time) >= BLINK_INTERVAL)
					{
						last_blink_time = current_time; // Update the last toggle time
						if (led_state == 0)
							{
								HAL_GPIO_WritePin(GPIOA, LED_AMBER_Pin, GPIO_PIN_SET);    // Turn ON PA0 (LED 1)
								HAL_GPIO_WritePin(GPIOA, LED_RED_Pin, GPIO_PIN_RESET);  // Turn OFF PA1 (LED 2)
								led_state = 1; // Update state
							} else {
								HAL_GPIO_WritePin(GPIOA, LED_AMBER_Pin, GPIO_PIN_RESET);  // Turn OFF PA0 (LED 1)
								HAL_GPIO_WritePin(GPIOA, LED_RED_Pin, GPIO_PIN_SET);    // Turn ON PA1 (LED 2)
								led_state = 0; // Update state
							}
					}
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
