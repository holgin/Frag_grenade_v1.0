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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

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

#define BLINK_INTERVAL 50 // 50ms interval
#define CHECK_INTERVAL 1000 // Interval in milliseconds

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t last_check_time = 0; // Tracks the last time the playback was checked
uint32_t last_blink_time = 0; // Tracks last blink time
uint32_t seed = 1; // Initialize the seed
uint8_t led_state = 0;        // Tracks which LED is ON
uint8_t explosion = 0;
uint8_t status = 0;
uint8_t dma_rx_buffer[10]; // DMA buffer for UART reception
volatile uint8_t playback_status = 0xFF; // Default playback status
uint32_t last_query_time = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//void Blink_LEDs(void);
void DFPlayer_SendCommand(uint8_t, uint8_t, uint8_t);
//void HandlePlaybackStatus(void);
//void ReadPlaybackStatus_DMA(void);
void Timeout_CheckPlayback(void);
uint8_t getRandomNumber(void);
uint8_t ReadPlaybackStatus(void);

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM21_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim21);
  //HAL_Delay(1000);
  //HAL_UART_Receive_DMA(&huart2, dma_rx_buffer, sizeof(dma_rx_buffer));
  HAL_Delay(1000);
  DFPlayer_SendCommand(DFPLAYER_CMD_RESET , 0, 0); // Reset DFPlayer Mini
  HAL_Delay(1000);
  DFPlayer_SendCommand(DFPLAYER_CMD_PLAY_TRACK, 0x00, getRandomNumber());
  HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      if (HAL_GetTick() - last_query_time >= CHECK_INTERVAL)
		  {
			  last_query_time = HAL_GetTick();
			  HAL_GPIO_WritePin(GPIOA, LED_GREEN_Pin, ReadPlaybackStatus());
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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

		HAL_UART_Transmit(&huart2, packet, 10, HAL_MAX_DELAY);		// Send packet over UART
	}

uint8_t getRandomNumber()
	{
    	seed = (2*HAL_GetTick() * 1103515245 + 12345) & 0x7FFFFFFF; // LCG formula
    	return (uint8_t)((seed % 3) + 1); // Random number between 1 and 3
	}
/*
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
	*/
/*
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
	{
		HAL_GPIO_TogglePin(GPIOA, LED_RED_Pin);
		if (huart->Instance == USART2)  			// Validate the DFPlayer response
			{
				if (dma_rx_buffer[0] == 0x7E && dma_rx_buffer[3] == 0x42 && dma_rx_buffer[9] == 0xEF)
					{
						playback_status = dma_rx_buffer[6]; // Extract playback status
					} else {
						playback_status = 0xFF; // Invalid response
					}
			}
	}
*/
uint8_t ReadPlaybackStatus(void)
	{
		uint8_t status_packet[10];
		uint8_t status = 0x00; 									// Default to an invalid value
		DFPlayer_SendCommand(0x42, 0x00, 0x00);					// Send the status query command
		if (HAL_UART_Receive(&huart2, status_packet, 10, 100) == HAL_OK) 		// Wait for the response from the DFPlayer Mini
			{
				if (status_packet[3] == 0x42 && status_packet[0] == 0x7E && status_packet[9] == 0xEF) 			// Ensure the response is valid and matches the query
					{
						status = status_packet[6]; // Extract the playback status (0x01 or 0x00)
					}
			}
		return status; // Return playback status (0x01: Playing, 0x00: Stopped)
	}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
	{
		static uint8_t led_state = 0;  // Track LED state
//----------------------------------------------------------------------------------
		if (htim->Instance == TIM2)
			{
				if (explosion == 1)
					{
						if (led_state == 0)
							{
								HAL_GPIO_WritePin(GPIOA, LED_AMBER_Pin, GPIO_PIN_SET);   // Turn ON amber LED
								HAL_GPIO_WritePin(GPIOA, LED_RED_Pin, GPIO_PIN_RESET);  // Turn OFF red LED
								led_state = 1;
							} else {
								HAL_GPIO_WritePin(GPIOA, LED_AMBER_Pin, GPIO_PIN_RESET); // Turn OFF amber LED
								HAL_GPIO_WritePin(GPIOA, LED_RED_Pin, GPIO_PIN_SET);     // Turn ON red LED
								led_state = 0;
							}
					} else {
						HAL_GPIO_WritePin(GPIOA, LED_AMBER_Pin, GPIO_PIN_RESET); 						// Turn off blinking LEDs when explosion is inactive
						HAL_GPIO_WritePin(GPIOA, LED_RED_Pin, GPIO_PIN_RESET);
					}
			}
//----------------------------------------------------------------------------------
		static uint8_t second_ticks = 0;
		if (htim->Instance == TIM21)
			{
				second_ticks++;
				if (second_ticks >= 5)
					{
						explosion = 1;
						second_ticks = 0;
					}
			}
	}

void Timeout_CheckPlayback(void)
	{
		static uint32_t last_check_time = 0;      								// Tracks the last check time
		static uint32_t playback_stop_time = 0;  								// Tracks the time when playback was first detected as stopped
		uint32_t current_time = HAL_GetTick();   								// Get the current system time in milliseconds
		if ((current_time - last_check_time) >= 100) 							// Check every 100ms
			{
				last_check_time = current_time; 								// Update last check time
				playback_status = ReadPlaybackStatus();
				if (playback_status == 0)
					{
						if (playback_stop_time == 0)
							{
								playback_stop_time = current_time; // Start tracking stop time
							} else if ((current_time - playback_stop_time) >= 500) {
								explosion = 0;               // Playback has been stopped for 500ms
								playback_stop_time = 0;      // Reset stop time
							}
					} else {
						playback_stop_time = 0;          // Reset if playback is active
					}
			}
	}

void EXTI1_IRQHandler(void)
	{
		HAL_GPIO_EXTI_IRQHandler(Fuze_Pin); // Call HAL to handle the interrupt
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
