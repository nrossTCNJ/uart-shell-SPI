/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <tim_init.h>
#include "i2c.h"
#include "mpu.h"
#include "spi_init.h"
#include "mfrc522.h"
#include "process_command.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define BUFFER_SIZE 64
#define CMD_SIZE 32

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t receive[1];
uint8_t buffer[BUFFER_SIZE];
uint8_t led_state;
uint8_t streaming;
volatile uint8_t head;
volatile uint8_t tail;
extern volatile uint8_t data_ready;
extern volatile uint8_t pulse_ready;
volatile uint8_t pwm_val;
uint8_t pulse_state;
uint8_t mfrc_state;
uint32_t last_detected_tick;
uint8_t correct_uid[4];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
void configure_leds(void);
void handle_uart_cmd(uint8_t *cmd_buff, uint8_t *cmd_index);
void ring_write(uint8_t byte);
uint8_t ring_read(void);
uint8_t ring_empty(void);
void stream_mpu(void);
void pulse_led(void);
void handle_mfrc522(void);
void print_mpu_data();
void check_uid(uint8_t *uid);

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
  uint8_t cmd_buff[CMD_SIZE];
  uint8_t cmd_index = 0;

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
  i2c1_init();
  mpu_init();
  spi1_init();
  mfrc522_init();
  configure_leds();

  /* USER CODE BEGIN 2 */
  head = 0;
  tail = 0;
  streaming = 0;
  mfrc_state = 0;
  last_detected_tick = 0;

  // send greeting to verify connection
  char greeting[16];
  snprintf(greeting, sizeof(greeting), "> Welcome\r\n");
  HAL_UART_Transmit(&huart2, greeting, strlen(greeting), HAL_MAX_DELAY);
  HAL_UART_Receive_IT(&huart2, receive, 1);

  // init timers
  tim3_init();
  tim2_init();
  tim4_init();
  data_ready = 0;
  pulse_ready = 0;

  correct_uid[0] = 0xE7;
  correct_uid[1] = 0x57;
  correct_uid[2] = 0x16;
  correct_uid[3] = 0x06;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	handle_uart_cmd(cmd_buff, &cmd_index); 	// for user inputs
	stream_mpu();							// streams MPU sensor data
	pulse_led(); 							// pulsing command
	handle_mfrc522();						// RF card sensor

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void configure_leds(void)
{
	RCC->AHB1ENR |= (1 << 2);		// GPIOC clock enable
	GPIOC->MODER &= ~(0xF << 4);	// clear PC2, PC3
	GPIOC->MODER |= (0x5 << 4);		// output mode
	GPIOC->OTYPER &= ~(0b11 << 2);	// push-pull
	GPIOC->PUPDR &= ~(0xF << 4);	// no pull-up pull-down
	GPIOC->OSPEEDR &= ~(0xF << 4);	// low speed
}

void handle_uart_cmd(uint8_t *cmd_buff, uint8_t *cmd_index)
{
	if(!ring_empty())
	{
		uint8_t byte = ring_read();

		// New line
		if(byte == '\r' || byte == '\n')
		{
			cmd_buff[*cmd_index] = '\0';
			process_command(cmd_buff);
			*cmd_index = 0;
			memset(cmd_buff, 0, CMD_SIZE);
		}
		// Backspace
		else if(byte == '\b' || byte == 0x7F)
		{
			if(*cmd_index != 0)
			{
				(*cmd_index)--;
				cmd_buff[*cmd_index] = 0;
				uint8_t bs_seq[] = "\b \b";
				HAL_UART_Transmit(&huart2, bs_seq, 3, HAL_MAX_DELAY);
			}
		}
		// Normal character, buffer isn't full
		else if(*cmd_index < CMD_SIZE - 1)
		{
			cmd_buff[*cmd_index] = byte;
			(*cmd_index)++;
			HAL_UART_Transmit(&huart2, &byte, 1, HAL_MAX_DELAY);
		}
	}
}

void ring_write(uint8_t byte)
{
	// checks if buffer is not full
	if (head != (tail - 1 + BUFFER_SIZE) % BUFFER_SIZE)
	{
		// writes to head of ring buffer
		buffer[head] = byte;
		head = (head + 1) % BUFFER_SIZE; // wrap-around
	}
}

uint8_t ring_read(void)
{
	// checks if empty buffer
	if (head == tail)
	{
		return 0;
	}
	// reads current byte (tail) and advances
	else
	{
		uint8_t byte = buffer[tail];
		tail = (tail + 1) % BUFFER_SIZE; // wrap-around
		return byte;
	}
}

uint8_t ring_empty(void)
{
	// returns true if ring is empty
	if (head == tail)
		return 1;
	else
		return 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	// checks for USART 2
	if(huart->Instance == USART2)
	{
		ring_write(receive[0]);
		HAL_UART_Receive_IT(huart, receive, 1);
	}
}

void stream_mpu(void)
{
	if(streaming && data_ready)
	{
		print_mpu_data();
		data_ready = 0;
	}
}

void pulse_led(void)
{
	if(pulse_state != 0 && pulse_ready)
	{
		// pulse based on state
		if(pulse_state == 1)
			pwm_val++;
		else if(pulse_state == 2)
			pwm_val--;

		// change state
		if(pwm_val == 0)
			pulse_state = 1;
		else if(pwm_val == 128)	 // only goes half to be quicker
			pulse_state = 2;

		TIM3->CCR1 = pwm_val;
		pulse_ready = 0;
	}
}

void handle_mfrc522(void)
{
	uint8_t uid[4];
	char msg[16];
	if(mfrc_state == 0 && mfrc522_read_uid(uid) == SPI_OK)
	{
		mfrc_state = 1;
		last_detected_tick = HAL_GetTick();
		check_uid(uid);		// authentication
	}
	else if(mfrc_state == 1 && HAL_GetTick() - last_detected_tick > 2000)
	{
		mfrc_state = 0;
	}
}

void print_mpu_data()
{
	int16_t data[7];
	char msg[64];
	uint8_t err = mpu_read(data);
	if(err != I2C_OK)
	{
		snprintf(msg, sizeof(msg), "> Error\r\n");
		HAL_UART_Transmit(&huart2, msg, strlen(msg), HAL_MAX_DELAY);
	}
	else
	{
		MPU_Data mpu_data;
		mpu_convert(data, &mpu_data);

		snprintf(msg, sizeof(msg), "> Accel X: %.2f\r\n", mpu_data.accel_x);
		HAL_UART_Transmit(&huart2, msg, strlen(msg), HAL_MAX_DELAY);

		snprintf(msg, sizeof(msg), "> Accel Y: %.2f\r\n", mpu_data.accel_y);
		HAL_UART_Transmit(&huart2, msg, strlen(msg), HAL_MAX_DELAY);

		snprintf(msg, sizeof(msg), "> Accel Z: %.2f\r\n", mpu_data.accel_z);
		HAL_UART_Transmit(&huart2, msg, strlen(msg), HAL_MAX_DELAY);

		snprintf(msg, sizeof(msg), "> Temp (C): %.2f\r\n", mpu_data.temp_c);
		HAL_UART_Transmit(&huart2, msg, strlen(msg), HAL_MAX_DELAY);

		snprintf(msg, sizeof(msg), "> Temp (F): %.2f\r\n", mpu_data.temp_f);
		HAL_UART_Transmit(&huart2, msg, strlen(msg), HAL_MAX_DELAY);

		snprintf(msg, sizeof(msg), "> Gyro X: %.2f\r\n", mpu_data.gyro_x);
		HAL_UART_Transmit(&huart2, msg, strlen(msg), HAL_MAX_DELAY);

		snprintf(msg, sizeof(msg), "> Gyro Y: %.2f\r\n", mpu_data.gyro_y);
		HAL_UART_Transmit(&huart2, msg, strlen(msg), HAL_MAX_DELAY);

		snprintf(msg, sizeof(msg), "> Gyro Z: %.2f\r\n", mpu_data.gyro_z);
		HAL_UART_Transmit(&huart2, msg, strlen(msg), HAL_MAX_DELAY);
	}
}

void check_uid(uint8_t *uid)
{
	char msg[64];
	if(memcmp(uid, correct_uid, 4) == 0)
	{
		GPIOC->ODR &= ~(1 << 2);	// turn red led off
		GPIOC->ODR |= (1 << 3);		// turn green led on
		snprintf(msg, sizeof(msg), "> Authentication Successful\r\n");
		HAL_UART_Transmit(&huart2, msg, strlen(msg), HAL_MAX_DELAY);

	}
	else
	{
		GPIOC->ODR &= ~(1 << 3);	// turn green led off
		GPIOC->ODR |= (1 << 2);		// turn red led on
		snprintf(msg, sizeof(msg), "> Authentication Denied\r\n");
		HAL_UART_Transmit(&huart2, msg, strlen(msg), HAL_MAX_DELAY);
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
