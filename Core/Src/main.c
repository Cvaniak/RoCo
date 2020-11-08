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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "i2s.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "usbd_cdc_if.h" // Plik bedacy interfejsem uzytkownika do kontrolera USB
#include "../Oth/stm32f411e_discovery.h"
#include "../Oth/stm32f411e_discovery_accelerometer.h"
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

/* USER CODE BEGIN PV */
#define L_MIN 280
#define L_MIN_O 280
#define L_MAX 640
#define L_MAX_O 640
#define L_DIR 640

#define R_MIN 280
#define R_MIN_O 280
#define R_MAX 640
#define R_MAX_O 640
#define R_DIR 640

#define OFFS 0
#define DIFF 360

int timeStart = 0;
int lAt = 10;
int rAt = 100;

void leftArm(int n){
	if(n>DIFF)
		n=DIFF;
	if(n<0)
		n=0;
//	htim1.Instance->CCR1 = 640 - n; // lewy  - 640 tył 280 przód
	htim1.Instance->CCR1 = L_MAX - OFFS - n; // lewy  - 640 tył 280 przód
}

void rightArm(int n){
	if(n>DIFF)
		n=DIFF;
	if(n<0)
		n=0;
//	htim1.Instance->CCR4 = 280 +  n; // prawy - 280 tył 640 przód
	htim1.Instance->CCR4 = R_MIN + OFFS +  n; // prawy - 280 tył 640 przód
}

void leg(int n){
	if(n>DIFF)
		n=DIFF;
	htim3.Instance->CCR2 = 280 +  n; // prawy - 280 tył 640 przód
}

void timeF(int t){
	leftArm( DIFF*(t-timeStart)/lAt);
	rightArm(DIFF*(t-timeStart)/rAt);
}

void stopRobot(){
	while(1){
		BSP_LED_Toggle(LED3);
		BSP_LED_Toggle(LED4);
		BSP_LED_Toggle(LED5);
		BSP_LED_Toggle(LED6);
	}
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

uint8_t DataToSend[1000];
uint8_t Dlugosc = 0;
int nIndex = 0;
int lastIndex = 0;
int16_t pDataXYZ[3];
int16_t pDataX = 0;
int16_t pDataY = 0;
int16_t pDataZ = 0;
double i = 1;
double stepPerTime;
float Roll ;
float Pitch;


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
  MX_I2C1_Init();
  MX_I2S2_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  BSP_ACCELERO_Init();

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // PE9
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); // PE14
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // PB5
  //    PC4 R
  //    PC5
  //    PB0 G0
  //    PB1
  //    PB2 W0
  //    PE7
  //    PE8 B0

  //    PB5
  timeStart = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  	if(HAL_GPIO_ReadPin(BBLUE_GPIO_Port, BBLUE_Pin)){
			if(i)
				i = 0;
			else
				i = 1;
			timeStart = HAL_GetTick();
//			Dlugosc = sprintf(DataToSend, "Y \r\n");
//			CDC_Transmit_FS(DataToSend, Dlugosc);
//			lastIndex = nIndex;
	  	}
	  timeF(HAL_GetTick());

//	  nIndex++;
//	  HAL_Delay(400);
//	  leftArm( 110+250*((int)i%2));
//	  rightArm(110+250*(((int)i)%2));
//	  leftArm( DIFF*((int)i%2));
//	  rightArm(DIFF*((int)i%2));
//	  HAL_Delay(50);
//	  leg(55 + 135 + 135*i); // -145 -> 125  270
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 200;
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 5;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
