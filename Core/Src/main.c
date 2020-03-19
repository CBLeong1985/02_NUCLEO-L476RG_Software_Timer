/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "common.h"
#include "software_timer.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define APP 4
#if APP == 1
	#define LED_BLINKING_APP		// Software timer array for LED blinking
#elif APP == 2
	#define BUZZER_SOUND_APP		// Software timer for press button and buzzer sound
#elif APP == 3
	#define LCD_BACKLIGHT_APP		// Software timer for LCD backlight ON
#elif APP == 4
	#define UART_TIMEOUT_APP		// Software timer for UART receive timeout
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#ifdef LED_BLINKING_APP
uint8_t ledSoftwareTimerId[NUM_OF_SOFTWARE_TIMER / 2];
bool ledSoftwareTimerToggle[NUM_OF_SOFTWARE_TIMER / 2];
#endif

#ifdef BUZZER_SOUND_APP
bool isButtonPressed;
uint8_t buzzerTimerId;
bool isBuzzerOn;
#endif

#ifdef LCD_BACKLIGHT_APP
bool isButtonPressed;
uint8_t lcdBackLightTimerId;
bool isBackLightOn;
#endif

#ifdef UART_TIMEOUT_APP
bool isButtonPressed;
uint8_t uartReceiveTimeoutTimerId;
bool isUartReceiveTimeout;
bool isUartAcceptReceive;
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef LED_BLINKING_APP
/*******************************************************************************
 * @fn      LedTimerCallback
 * @brief   LED timer Callback
 * @paramz  softwareTimerId
 * @return  None
 ******************************************************************************/
void LedTimerCallback(uint8_t softwareTimerId)
{
	ledSoftwareTimerToggle[softwareTimerId] = !ledSoftwareTimerToggle[softwareTimerId];
}
#endif

#ifdef BUZZER_SOUND_APP
/*******************************************************************************
 * @fn      BuzzerTimerStartCallback
 * @brief   Buzzer timer start callback
 * @paramz  softwareTimerId
 * @return  None
 ******************************************************************************/
void BuzzerTimerStartCallback(uint8_t softwareTimerId)
{
	// BuzzerOn();
	isBuzzerOn = true;
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
}

/*******************************************************************************
 * @fn      BuzzerTimerCallback
 * @brief   Buzzer timer callback
 * @paramz  softwareTimerId
 * @return  None
 ******************************************************************************/
void BuzzerTimerCallback(uint8_t softwareTimerId)
{
	// BuzzerOff();
	isBuzzerOn = false;
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}
#endif

#ifdef LCD_BACKLIGHT_APP
/*******************************************************************************
 * @fn      BackLightTimerStartCallback
 * @brief   Back light timer start callback
 * @paramz  softwareTimerId
 * @return  None
 ******************************************************************************/
void BackLightTimerStartCallback(uint8_t softwareTimerId)
{
	// BackLightOn();
	isBackLightOn = true;
}

/*******************************************************************************
 * @fn      BackLightTimerCallback
 * @brief   Back light timer callback
 * @paramz  softwareTimerId
 * @return  None
 ******************************************************************************/
void BackLightTimerCallback(uint8_t softwareTimerId)
{
	// BackLightOff();
	isBackLightOn = false;
}
#endif

#ifdef UART_TIMEOUT_APP
/*******************************************************************************
 * @fn      UartReceiveTimeoutTimerStartCallback
 * @brief   Uart receive timeout timer start callback
 * @paramz  softwareTimerId
 * @return  None
 ******************************************************************************/
void UartReceiveTimeoutTimerStartCallback(uint8_t softwareTimerId)
{
	isUartReceiveTimeout = false;
	isUartAcceptReceive = true;
}

/*******************************************************************************
 * @fn      UartReceiveTimeoutTimerCallback
 * @brief   Uart receive timeout timer callback
 * @paramz  softwareTimerId
 * @return  None
 ******************************************************************************/
void UartReceiveTimeoutTimerCallback(uint8_t softwareTimerId)
{
	isUartReceiveTimeout =true;
	isUartAcceptReceive = false;
}

/*******************************************************************************
 * @fn      UartReceiveTimeoutTimerStopCallback
 * @brief   Uart receive timeout timer stop callback
 * @paramz  softwareTimerId
 * @return  None
 ******************************************************************************/
void UartReceiveTimeoutTimerStopCallback(uint8_t softwareTimerId)
{
	isUartReceiveTimeout = false;
	isUartAcceptReceive = false;
}
#endif
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
#ifdef LED_BLINKING_APP
  uint8_t i = 0;
#endif

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
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  // Enable software timer
  sSoftwareTimer.Enable();

#ifdef LED_BLINKING_APP
  for(i = 0; i < NUM_OF_SOFTWARE_TIMER / 2; i++)
  {
	  ledSoftwareTimerId[i] = sSoftwareTimer.Initialize(NULL, LedTimerCallback, NULL, TIMER_PERIODIC_TYPE);
  }
  sSoftwareTimer.Start(ledSoftwareTimerId[0], 100);
  sSoftwareTimer.Start(ledSoftwareTimerId[1], 300);
  sSoftwareTimer.Start(ledSoftwareTimerId[2], 500);
  sSoftwareTimer.Start(ledSoftwareTimerId[3], 700);
  for(;;)
  {
  }
#endif

#ifdef BUZZER_SOUND_APP
  // Normal code
  // BuzzerOn();
  // HAL_Delay(50);
  // BuzzerOff();
  buzzerTimerId = sSoftwareTimer.Initialize(BuzzerTimerStartCallback, BuzzerTimerCallback, NULL, TIMER_ONCE_TYPE);
  // For "SWV Data Trace Timeline Graph" can show data at startup
  isButtonPressed = true;
  isButtonPressed = false;
  isBuzzerOn = true;
  isBuzzerOn = false;
  for(;;)
  {
	  if(HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == GPIO_PIN_RESET && !isButtonPressed)
	  {
		  isButtonPressed = true;
		  sSoftwareTimer.Start(buzzerTimerId, 50);
	  }
	  else if(HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == GPIO_PIN_SET && isButtonPressed)
	  {
		  isButtonPressed = false;
	  }
  }
#endif
#ifdef LCD_BACKLIGHT_APP
  lcdBackLightTimerId = sSoftwareTimer.Initialize(BackLightTimerStartCallback, BackLightTimerCallback, NULL, TIMER_ONCE_TYPE);
  // For "SWV Data Trace Timeline Graph" can show data at startup
  isButtonPressed = true;
  isButtonPressed = false;
  isBackLightOn = true;
  isBackLightOn = false;
  for(;;)
  {
	  if(HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == GPIO_PIN_RESET && !isButtonPressed)
	  {
		  isButtonPressed = true;
		  sSoftwareTimer.Start(lcdBackLightTimerId, 2000);
	  }
	  else if(HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == GPIO_PIN_SET && isButtonPressed)
	  {
		  isButtonPressed = false;
	  }
  }
#endif

#ifdef UART_TIMEOUT_APP
  isButtonPressed = true;
  isButtonPressed = false;
  isUartReceiveTimeout = true;
  isUartAcceptReceive = true;
  isUartReceiveTimeout = false;
  isUartAcceptReceive = false;
  uartReceiveTimeoutTimerId = sSoftwareTimer.Initialize(UartReceiveTimeoutTimerStartCallback, UartReceiveTimeoutTimerCallback, UartReceiveTimeoutTimerStopCallback, TIMER_ONCE_TYPE);
  for(;;)
  {
	  // Check button press
	  if(HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == GPIO_PIN_RESET && !isButtonPressed)
	  {
		  isButtonPressed = true;
		  if(!isUartAcceptReceive)
		  {
			  sSoftwareTimer.Start(uartReceiveTimeoutTimerId, 5000);
		  }
		  else
		  {
			  sSoftwareTimer.Stop(uartReceiveTimeoutTimerId);
		  }
	  }
	  else if(HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == GPIO_PIN_SET)
	  {
		  isButtonPressed = false;
	  }
  }
#endif
/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
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
