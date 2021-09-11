/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t indata[1];
uint8_t SBUS_in[25];
int num=0;
uint16_t ch[16];
void SBUS_intoCH(void){
ch[0]  = ((SBUS_in[1]|SBUS_in[2]<<8)                      & 0x07FF);
ch[1]  = ((SBUS_in[2]>>3 |SBUS_in[3]<<5)                 & 0x07FF);
ch[2]  = ((SBUS_in[3]>>6 |SBUS_in[4]<<2 |SBUS_in[5]<<10)  & 0x07FF);
ch[3]  = ((SBUS_in[5]>>1 |SBUS_in[6]<<7)                 & 0x07FF);
ch[4]  = ((SBUS_in[6]>>4 |SBUS_in[7]<<4)                 & 0x07FF);
ch[5]  = ((SBUS_in[7]>>7 |SBUS_in[8]<<1 |SBUS_in[9]<<9)   & 0x07FF);
ch[6]  = ((SBUS_in[9]>>2 |SBUS_in[10]<<6)                & 0x07FF);
ch[7]  = ((SBUS_in[10]>>5|SBUS_in[11]<<3)                & 0x07FF);
ch[8]  = ((SBUS_in[12]   |SBUS_in[13]<<8)                & 0x07FF);
ch[9]  = ((SBUS_in[13]>>3|SBUS_in[14]<<5)                & 0x07FF);
ch[10] = ((SBUS_in[14]>>6|SBUS_in[15]<<2|SBUS_in[16]<<10) & 0x07FF);
ch[11] = ((SBUS_in[16]>>1|SBUS_in[17]<<7)                & 0x07FF);
ch[12] = ((SBUS_in[17]>>4|SBUS_in[18]<<4)                & 0x07FF);
ch[13] = ((SBUS_in[18]>>7|SBUS_in[19]<<1|SBUS_in[20]<<9)  & 0x07FF);
ch[14] = ((SBUS_in[20]>>2|SBUS_in[21]<<6)                & 0x07FF);
ch[15] = ((SBUS_in[21]>>5|SBUS_in[22]<<3)                & 0x07FF);
}
int val1;
int val2;
int motor;
int dir;
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
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart2,indata, 1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		motor=ch[2]-800;//300-1800
		dir=ch[3]-1024;//300-1800
		if(motor<0)motor=0;
		val1=(motor*1.3+dir*0.7)/2;
		val2=(motor*1.3-dir*0.7)/2;
		if(val1<0)val1=0;
		if(val2<0)val2=0;
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,val1);
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,val2);
		HAL_Delay(1);
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
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
}

/* USER CODE BEGIN 4 */
 
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(num!=0||indata[0]==0x0f){
		if(indata[0]==0x0f)num=0;
		SBUS_in[num++]=indata[0];
		if(num==25){num=0;SBUS_intoCH();}
	}
	HAL_UART_Receive_IT(&huart2,indata, 1);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
