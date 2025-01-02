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
#include "adc.h"
#include "dac.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
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
#define IN_MAX 4000
#define IN_RELAXED 3900
#define MODE_LIM 2
#define NOT_PRESSED 1000
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Function for mapping input to different steps
uint16_t Map(uint16_t x, int step) {
	return step - (uint16_t)(x * step / IN_MAX);
	// return x * step / (IN_MAX - in_min) + out_min;
}

// Reads the current voltage level from the controller
uint16_t ADC_Read(int step) {
	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
 	uint16_t raw = HAL_ADC_GetValue(&hadc2);
 	if (raw > IN_RELAXED) {
 		return NOT_PRESSED;
 	}
 	char msg[10];
 	sprintf(msg, "%hu\r\n", raw);
 	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
 	HAL_DAC_SetValue(&hdac1, DAC1_CHANNEL_2, DAC_ALIGN_12B_R, IN_MAX - raw);
 	return 1; //Map(raw, step);
}

void DisplayMode(char *mode) {
	ssd1306_Fill(Black);
	ssd1306_SetCursor(5, 20);
	ssd1306_WriteString(mode, Font_7x10, White);
	ssd1306_UpdateScreen();
}

void DisplayStep(int step) {
	ssd1306_SetCursor(8, 20);
	ssd1306_WriteData(step, sizeof(int));
	ssd1306_UpdateScreen();
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  char msg[10];
  uint16_t position;
  uint16_t previous;

  // Array of the correct voltages for 12-TET
  int tet12[] = {
		  801, 920, 1039, 1157, 1275, 1393, 1511, 1630, 1748, 1867, 1985, 2104, // 1st octave
		  2228, 2347, 2466, 2584, 2703, 2822, 2940, 3058, 3177, 3295, 3414, 3532 // 2nd octave
  };

  // Array of the correct voltages for a Major scale
  int major[] = {
		  0, 204, 410, 514, 719, 925, 1028, // 1st Octave
		  1239, 1444, 1651, 1753, 1959, 2166, 2371, // 2nd Octave
		  2479, 2685, 2891, 2994, 3200, 3405, 3611, // 3rd Octave
		  3719 // Last C
  };

  int minor[] = {
  		  0, 204, 307, 514, 719, 822, 1028, // 1st Octave
		  1239, 1444, 1548, 1753, 1857, 1959, 2166, // 2nd Octave
  		  2479, 2685, 2788, 2994, 3200, 3302, 3508, // 3rd Octave
  		  3719 // Last C
  };

  int* modes[] = {tet12, major, minor};
  int mode = 0;
  int steps = 24;
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
  MX_ADC2_Init();
  MX_DAC1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_DAC_Start(&hdac1, DAC1_CHANNEL_2);
  //ssd1306_Init();
  //DisplayMode("12-TET");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (HAL_GPIO_ReadPin(MODE_GPIO_Port, MODE_Pin)) {
		  mode++;
		  if (mode > MODE_LIM) {
			  mode = 0;
			  steps = 24;
			  //DisplayMode("12-TET");
		  }
		  else {
			  steps = 22;
			  //mode == 1 ? DisplayMode("MAJOR") : DisplayMode("MINOR");
		  }
	  }
	  position = ADC_Read(steps);

	  if (position == NOT_PRESSED) {
		  position = previous;
	  }
	  else if (steps == position) {
		  position--;
	  }

	  //HAL_DAC_SetValue(&hdac1, DAC1_CHANNEL_2, DAC_ALIGN_12B_R, modes[mode][position]);
	  sprintf(msg, "%hu\r\n", position);
	  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

	  if (position != previous) {
		  previous = position;
		  //DisplayStep(position);
	  }

	  HAL_Delay(10);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
