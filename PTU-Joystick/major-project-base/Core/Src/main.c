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

#include "ptu_definitions.h"
#include "ptu_i2c.h"
#include "joystick_map.h"
#include "initialisation.h"
#include "timer.h"
#include "serial.h"
#include "stm32f303xc.h"
#include "math.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

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
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define USART_ICR_RXNECF_Pos        5
#define USART_ICR_RXNECF_Msk        (0x01U << USART_ICR_RXNECF_Pos)
#define BUFFER 256

int lidar_values[6] = {0, 0, 0, 0, 0, 0};
int lidar_global = 0;
int timer_count = 0;
int hit_waypoint = 0;

int WAYPOINT_DISTANCE = 300;

int TARGET_ONE_LOWER_X = 500;
int TARGET_ONE_UPPER_X = 800;
int TARGET_ONE_LOWER_Y = 1600;
int TARGET_ONE_UPPER_Y = 1800;

int TARGET_TWO_LOWER_X = 1500;
int TARGET_TWO_UPPER_X = 1700;
int TARGET_TWO_LOWER_Y = 1550;
int TARGET_TWO_UPPER_Y = 1750;

int TARGET_THREE_LOWER_X = 0;
int TARGET_THREE_UPPER_X = 250;
int TARGET_THREE_LOWER_Y = 2900;
int TARGET_THREE_UPPER_Y = 3100;

int TARGET_FOUR_LOWER_X = 1550;
int TARGET_FOUR_UPPER_X = 1750;
int TARGET_FOUR_LOWER_Y = 950;
int TARGET_FOUR_UPPER_Y = 1250;

int TARGET_FIVE_LOWER_X = 2300;
int TARGET_FIVE_UPPER_X = 2500;
int TARGET_FIVE_LOWER_Y = 1650;
int TARGET_FIVE_UPPER_Y = 1850;

int TARGET_SIX_LOWER_X = 1000;
int TARGET_SIX_UPPER_X = 1200;
int TARGET_SIX_LOWER_Y = 1300;
int TARGET_SIX_UPPER_Y = 1500;

int SERVO_CLOCK_PERIOD = 20000;
int SERVO_MAX_UPPER = 3000;
int SERVO_MAX_LOWER = 2000;
int SERVO_MAX_MIDDLE = 2055;

int target_count = 0;
int x_position = 1500;
int y_position = 1500;

void TIM3_IRQHandler(void)
{
	// UIF is from an overflow type event
	if ((TIM3->SR & TIM_SR_UIF) != 0){
		TIM3->SR &= ~TIM_SR_UIF;
	}

	// CC1IF is from a successful output compare
	if ((TIM3->SR & TIM_SR_CC1IF) != 0){
		TIM3->SR &= ~TIM_SR_CC1IF;
	}
	timer_count++;

	int index = timer_count % 6;

	lidar_values[index] = lidar_global;
}

void sendString(const char *str)
{
    while (*str)
    {
        // Wait until transmit data register is empty
        while (!(USART1->ISR & USART_ISR_TXE_Msk));
        // Send a single character
        USART1->TDR = *str++;
    }
}

typedef union {
	uint8_t all_leds;
	struct {
		uint8_t led_pair_1 : 2;
		uint8_t led_pair_2 : 2;
		uint8_t led_set_of_4 : 4;
	} led_groups;
} LedRegister;

uint16_t last_capture = 0;
uint16_t diff = 0;

uint16_t rise_time = 0;
uint16_t last_period = 0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	uint8_t buffer[32];
	if (htim->Instance == TIM1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		uint16_t IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value

		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == 1)
			rise_time = IC_Val1;
		else
			last_period = IC_Val1 - rise_time;

		diff = IC_Val1 - last_capture;
		last_capture = IC_Val1;
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

	uint8_t string_to_send[256];

	enable_clocks();
	initialise_board();
	enableUSART1();

	LedRegister *led_register = ((uint8_t*)&(GPIOE->ODR)) + 1;

	SerialInitialise(BAUD_115200, &USART1_PORT, 0x00);

	HAL_StatusTypeDef return_value = 0x00;

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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USB_PCD_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);

	TIM2->ARR = SERVO_CLOCK_PERIOD; // 20000 = 20ms, which is the desired clock period for servos
	TIM2->CR1 |= TIM_CR1_ARPE; // this makes the timing not change until the next pulse is finished

	initialise_ptu_i2c(&hi2c1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	// Setup timer with interrupts for LIDAR moving average
	__disable_irq();
	initialise_timer();
	__enable_irq();

	// Reset lidar board
	uint8_t reset_value = 0x00;
	return_value = HAL_I2C_Mem_Write(&hi2c1, LIDAR_WR, 0x00, 1, &reset_value, 1, 10);

	// Delay for initialisation of the lidar
	HAL_Delay(100);

	sprintf(string_to_send, "\r\n\n\n\n\nYou are a student enrolled at USYD in the best engineering stream. Six core subjects, six LIDAR waypoints, six numbers to complete the pattern.\r\n\n");
	SerialOutputString(string_to_send, &USART1_PORT);

	while (1)
	{
		// Read Joystick Data
		HAL_ADC_Start(&hadc1); 						// start the adc
		HAL_ADC_PollForConversion(&hadc1, 100); 	// poll for conversion
		int adc_val_x = HAL_ADC_GetValue(&hadc1);   // get the adc value
		HAL_ADC_Stop(&hadc1); 						// stop adc

		HAL_ADC_Start(&hadc2); 						// start the adc
		HAL_ADC_PollForConversion(&hadc2, 100); 	// poll for conversion
		int adc_val_y = HAL_ADC_GetValue(&hadc2);   // get the adc value
		HAL_ADC_Stop(&hadc2); 						// stop adc

		HAL_Delay(20); 								// wait for 20ms

        // Remove noise with no input
		if (adc_val_x <= SERVO_MAX_UPPER && adc_val_x >= SERVO_MAX_LOWER) {
			adc_val_x = SERVO_MAX_MIDDLE;
		}

		if (adc_val_y <= SERVO_MAX_UPPER && adc_val_y >= SERVO_MAX_LOWER) {
			adc_val_y = SERVO_MAX_MIDDLE;
		}

		// Update positions of the servos from the joystick
		x_position = updateValues(adc_val_x, x_position);
		y_position = updateValues(adc_val_y, y_position);

		TIM2->CCR1 = y_position;
		TIM2->CCR2 = x_position;

		uint8_t lidar_value = 0x03;
		return_value = HAL_I2C_Mem_Write(&hi2c1, LIDAR_WR, 0x00, 1, &lidar_value, 1, 100);

		lidar_value = 0xff;

		uint8_t lidar_MSBa = 0x00;
		uint8_t lidar_LSBa = 0x00;

		volatile uint16_t lidar_distance = 0xff;

		uint16_t timeout;

		while ((lidar_value & 0x01) != 0x00) {
			return_value = HAL_I2C_Mem_Read(&hi2c1, LIDAR_RD, 0x01, 1, &lidar_value, 1, 100);
			return_value = HAL_I2C_Mem_Read(&hi2c1, LIDAR_RD, 0x0f, 1, &lidar_MSBa, 1, 100);
			return_value = HAL_I2C_Mem_Read(&hi2c1, LIDAR_RD, 0x10, 1, &lidar_LSBa, 1, 100);

			lidar_distance = ((lidar_MSBa << 8) | lidar_LSBa);
			timeout += 1;
			if (timeout > 0xff)
				break;
		}

		if (last_period > 4000)
			last_period = 5000;
		if (lidar_distance > 4000)
			lidar_distance = 5500;

		lidar_global = lidar_distance * 10;

		// Determine LIDAR distance with moving average
		int lidar_average = (lidar_values[0] + lidar_values[1] + lidar_values[2] + lidar_values[3] + lidar_values[4] + lidar_values[5])/6;

		if (lidar_global > WAYPOINT_DISTANCE) {
			led_register->led_groups.led_pair_1 = 0b00;
			led_register->led_groups.led_pair_2 = 0b00;
			led_register->led_groups.led_set_of_4 = 0b1000;
		}

		if (lidar_global <= WAYPOINT_DISTANCE && lidar_global >= 0) {
			hit_waypoint = 1;
			led_register->led_groups.led_pair_1 = 0b11;
			led_register->led_groups.led_pair_2 = 0b11;
			led_register->led_groups.led_set_of_4 = 0b0111;
		}

		// Way Point targeting and game sequence!

		if (x_position > TARGET_ONE_LOWER_X && x_position < TARGET_ONE_UPPER_X
				&& y_position > TARGET_ONE_LOWER_Y && y_position < TARGET_ONE_UPPER_Y
				&& target_count == 0 ){
			if (hit_waypoint == 1) {
				sprintf(string_to_send, "TARGET #1 ACQUIRED! Pattern #1: 1109\r\n");
				target_count = 1;
				SerialOutputString(string_to_send, &USART1_PORT);
			}
		}

		else if (x_position > TARGET_TWO_LOWER_X && x_position < TARGET_TWO_UPPER_X
				&& y_position > TARGET_TWO_LOWER_Y && y_position < TARGET_TWO_UPPER_Y
				&& target_count == 1 ){
			if (hit_waypoint == 1) {
				sprintf(string_to_send, "TARGET #2 ACQUIRED! Pattern #2: 0109\r\n");
				target_count = 2;
				SerialOutputString(string_to_send, &USART1_PORT);
			}
		}

		else if (x_position > TARGET_THREE_LOWER_X && x_position < TARGET_THREE_UPPER_X
				&& y_position > TARGET_THREE_LOWER_Y && y_position < TARGET_THREE_UPPER_Y
				&& target_count == 2 ){
			if (hit_waypoint == 1) {
				sprintf(string_to_send, "TARGET #3 ACQUIRED! Pattern #3: 6009\r\n");
				target_count = 3;
				SerialOutputString(string_to_send, &USART1_PORT);
			}
		}

		else if (x_position > TARGET_FOUR_LOWER_X && x_position < TARGET_FOUR_UPPER_X
				&& y_position > TARGET_FOUR_LOWER_Y && y_position < TARGET_FOUR_UPPER_Y
				&& target_count == 3 ){
			if (hit_waypoint == 1) {
				sprintf(string_to_send, "TARGET #4 ACQUIRED! Pattern #4: 8006\r\n");
				target_count = 4;
				SerialOutputString(string_to_send, &USART1_PORT);
			}
		}

		else if (x_position > TARGET_FIVE_LOWER_X && x_position < TARGET_FIVE_UPPER_X
				&& y_position > TARGET_FIVE_LOWER_Y && y_position < TARGET_FIVE_UPPER_Y
				&& target_count == 4 ){
			if (hit_waypoint == 1) {
				sprintf(string_to_send, "TARGET #5 ACQUIRED! Pattern #5: ????\r\n");
				target_count = 5;
				SerialOutputString(string_to_send, &USART1_PORT);
			}
		}

		else if (x_position > TARGET_SIX_LOWER_X && x_position < TARGET_SIX_UPPER_X
				&& y_position > TARGET_SIX_LOWER_Y && y_position < TARGET_SIX_UPPER_Y
				&& target_count == 5 ){
			if (hit_waypoint == 1) {
				sprintf(string_to_send, "TARGET #6 ACQUIRED! Pattern #6: 6006\r\n\n");
				SerialOutputString(string_to_send, &USART1_PORT);
				sprintf(string_to_send, "The pattern to unlock Stewart's magic orange box is:\r\n\n");
				SerialOutputString(string_to_send, &USART1_PORT);
				sprintf(string_to_send, "1109, 0109, 6009, 8009, _______, 9009\r\n\n");
				SerialOutputString(string_to_send, &USART1_PORT);
				sprintf(string_to_send, "What is the missing number?\r\n\n");
				SerialOutputString(string_to_send, &USART1_PORT);
				target_count = 6;
			}
		}

//		else {
//			// Do nothing
//		}

		else {
			sprintf(string_to_send, "Timer: %hu, Lidar I2C: %hu,  x-pos: %hu, y-pos: %hu, average: %hu, counter: %hu\r\n", timer_count, lidar_global, x_position, y_position, lidar_average, target_count);
//			sprintf(string_to_send, "%hu, %hu, %hu, %hu, %hu, %hu, %hu\r\n", lidar_values[0], lidar_values[1], lidar_values[2], lidar_values[3], lidar_values[4], lidar_values[5], lidar_average);
			SerialOutputString(string_to_send, &USART1_PORT);
			// Do nothing
		}

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_TIM1|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 47;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 47;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 2000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DRDY_Pin MEMS_INT3_Pin MEMS_INT4_Pin MEMS_INT1_Pin
                           MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT1_Pin
                          |MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_I2C_SPI_Pin LD4_Pin LD3_Pin LD5_Pin
                           LD7_Pin LD9_Pin LD10_Pin LD8_Pin
                           LD6_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
