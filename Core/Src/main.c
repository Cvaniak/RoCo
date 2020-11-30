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


#define B_MIN 300
#define B_MAX 590
#define B_DIFF 290

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
	htim3.Instance->CCR1 = L_MAX - OFFS - n; // lewy  - 640 tył 280 przód
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
	if(n>B_DIFF)
		n=B_DIFF;
	if(n<0)
		n=0;
	htim1.Instance->CCR1 = B_MAX - n; // prawy - 280 tył 640 przód
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

void buttons_handler();
void legs();
void jump();
void jumpL();
void jumpR();
void jumpRR();
void jumpF();
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
double AA1 = 100;
double AA2 = 330;
double AA3 = 330;
double AA4 = 10;
double AA5 = 50;
double AA6 = 200;
double W1 = 0;
double W2 = 0;
double B1 = 0;
double B2 = 0;
double G1 = 0;
double G2 = 0;
double R0 = 0;

double LF = DIFF;
double RF = DIFF;
double BB = 0;

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
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // PB4
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

	  buttons_handler();

//	  rightArm(RF);
//	  leftArm(RF);
//	  leg(RF);
	  legs();
	  HAL_Delay(1);
	  RF = DIFF;
	  LF = DIFF;
	  BB = 0;
//	  timeF(HAL_GetTick());

//	  leftArm( DIFF*((int)i%2));
//	  rightArm(DIFF*((int)i%2));
//	  leg(55 + 135 + 135*i); // -145 -> 125  270
  }


  //			Dlugosc = sprintf(DataToSend, "Y \r\n");
  //			CDC_Transmit_FS(DataToSend, Dlugosc);
  //			lastIndex = nIndex;
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

void buttons_handler(){
  	if(HAL_GPIO_ReadPin(BBLUE_GPIO_Port, BBLUE_Pin)) i = 1;
  	else i = 0;
//
//  	if(HAL_GPIO_ReadPin(ButtonB1_GPIO_Port, ButtonB1_Pin)) BB += 1; // B1 = 1;
//  	else B1 = 0;
//
//  	if(HAL_GPIO_ReadPin(ButtonB2_GPIO_Port, ButtonB2_Pin)) BB -= 1; // B2 = 1;
//  	else B2 = 0;
//
//  	if(HAL_GPIO_ReadPin(ButtonG1_GPIO_Port, ButtonG1_Pin)) LF += 1; // G1 = 1;
//  	else G1 = 0;
//
//  	if(HAL_GPIO_ReadPin(ButtonG2_GPIO_Port, ButtonG2_Pin)) RF += 1; // G2 = 1;
//  	else G2 = 0;
//
//  	if(HAL_GPIO_ReadPin(ButtonW1_GPIO_Port, ButtonW1_Pin)) LF -= 1; // W1 = 1;
//  	else W1 = 0;
//
//  	if(HAL_GPIO_ReadPin(ButtonW2_GPIO_Port, ButtonW2_Pin)) RF -= 1; //  W2 = 1;
//  	else W2 = 0;


  	if(!HAL_GPIO_ReadPin(ButtonB1_GPIO_Port, ButtonB1_Pin)) jumpF(); // B1 = 1;
  	else B1 = 0;

  	if(!HAL_GPIO_ReadPin(ButtonB2_GPIO_Port, ButtonB2_Pin)) jumpF(); // B2 = 1;
  	else B2 = 0;

  	if(!HAL_GPIO_ReadPin(ButtonG1_GPIO_Port, ButtonG1_Pin)) jumpR(); // G1 = 1;
  	else G1 = 0;

  	if(!HAL_GPIO_ReadPin(ButtonG2_GPIO_Port, ButtonG2_Pin)) jumpL(); // G2 = 1;
  	else G2 = 0;

  	if(!HAL_GPIO_ReadPin(ButtonW1_GPIO_Port, ButtonW1_Pin)) jumpRR(); // W1 = 1;
  	else W1 = 0;

  	if(!HAL_GPIO_ReadPin(ButtonW2_GPIO_Port, ButtonW2_Pin)) jumpLL(); //  W2 = 1;
  	else W2 = 0;

  	if(!HAL_GPIO_ReadPin(ButtonR_GPIO_Port, ButtonR_Pin)) jump();
  	else R0 = 0;

  	if(RF > DIFF) RF = DIFF;
  	else if(RF < 0) RF = 0;
  	if(LF > DIFF) LF = DIFF;
  	else if(LF < 0) LF = 0;

  	if(BB > B_DIFF) BB = B_DIFF;
  	else if(BB < 0) BB = 0;

//  	{
//		timeStart = HAL_GetTick();}
}

void legs(){
	leftArm(LF);
	rightArm(RF);
	leg(BB);
}



void jump(){
//	HAL_Delay(AA2);
	// AA1 100, -100, -100, 10 50 26 | RF 270 LF 283 BB 16
	//          -100   100        0  |    194    309    120
	//                            0  |    187    0      165

	//          -100   200  50 50 0  |    148    22     76

	leftArm(LF-AA2);
	rightArm(RF-AA3);
	HAL_Delay(AA4);
	leg(B_DIFF);
	HAL_Delay(AA1);
//	HAL_Delay(40);
	leftArm(LF);
	rightArm(RF);
	HAL_Delay(AA5);
	leg(BB-AA6);
	HAL_Delay(700);
//	leftArm(LF);
//	rightArm(RF);
//	HAL_Delay(100);
}


void jumpR(){
	//	HAL_Delay(AA2);
		// AA1 100, -100   200  50 50 0  |    148    22     76
		AA1 = 100;
		AA2 = -100;
		AA3 = 200;
		AA4 = 50;
		AA5 = 50;
		AA6 = 0;
		RF = 148;
		LF = 22;
		BB = 70;

		leftArm(LF-AA2);
		rightArm(RF-AA3);
		HAL_Delay(AA4);
		leg(B_DIFF);
		HAL_Delay(AA1);
	//	HAL_Delay(40);
//		leftArm(LF);
		rightArm(RF);
		HAL_Delay(AA5);
		leg(BB-AA6);
		HAL_Delay(700);
		leftArm(LF);
	}
void jumpRR(){
	RF = 148;
	LF = 40;
	BB = 70;
	leftArm(LF+50);
	rightArm(RF-50);
	leg(BB+15);
	HAL_Delay(50);
	leg(BB);
	HAL_Delay(30);
	leftArm(LF+300);
	HAL_Delay(120);
}

void jumpLL(){
	RF = 40;
	LF = 148;
	BB = 70;
	rightArm(RF+50);
	leftArm(LF-50);
	leg(BB+15);
	HAL_Delay(50);
	leg(BB);
	HAL_Delay(30);
	rightArm(RF+300);
	HAL_Delay(120);
}

void jumpL(){
	{
		//	HAL_Delay(AA2);
			// AA1 100, -100   200  50 50 0  |    148    22     76

		AA1 = 100;
		AA2 = 200;
		AA3 = -100;
		AA4 = 50;
		AA5 = 50;
		AA6 = 0;
		RF = 22;
		LF = 148;
		BB = 76;
			leftArm(LF-AA2);
			rightArm(RF-AA3);
			HAL_Delay(AA4);
			leg(B_DIFF);
			HAL_Delay(AA1);
		//	HAL_Delay(40);
			leftArm(LF);
//			rightArm(RF);
			HAL_Delay(AA5);
			leg(BB-AA6);
			HAL_Delay(700);
			rightArm(RF);
		}
}
void jumpF(){
	//	HAL_Delay(AA2);
		// AA1 100, -100, -100, 10 50 26 | RF 270 LF 283 BB 16

	AA1 = 100;
	AA2 = -100;
	AA3 = -100;
	AA4 = 10;
	AA5 = 50;
	AA6 = 26;
	RF = 270;
	LF = 283;
	BB = 16;
		leftArm(LF-AA2);
		rightArm(RF-AA3);
		HAL_Delay(AA4);
		leg(B_DIFF);
		HAL_Delay(AA1);
	//	HAL_Delay(40);
		leftArm(LF);
		rightArm(RF);
		HAL_Delay(AA5);
		leg(BB-AA6);
		HAL_Delay(700);
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
