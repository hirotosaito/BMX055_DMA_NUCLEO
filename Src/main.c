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
#include <stdio.h>
#include <string.h>
#include "dma.h"
#include "gpio.h"
#include "i2c.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define I2C_BUF_SIZE 100
#define true 1
#define false 0
#define Addr_Accl 0x19
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char k[I2C_BUF_SIZE];
uint8_t isxready = false;
uint8_t isyready = false;
uint8_t iszready = false;

float xAccl = 0.00;
float yAccl = 0.00;
float zAccl = 0.00;
float f = 3.14;

float xGyro;
float yGyro;
float zGyro;

int isI2CRunning;

typedef struct {
  /* data */
  uint8_t xLSB;
  uint8_t xMSB;
  uint8_t yLSB;
  uint8_t yMSB;
  uint8_t zLSB;
  uint8_t zMSB;  

} bmx;
bmx Accl_temp = {0,0,0,0,0,0};
bmx Accl = {0,0,0,0,0,0};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Show_float(char*, float);
void BMX_Init(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  BMX_Init();
  // first time to read sensor value
  HAL_I2C_Mem_Read_DMA(&hi2c1, Addr_Accl << 1, 0x00, 1,(&Accl_temp), 6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t i = 0;
  while (1) {
    /* USER CODE END WHILE */
    sprintf(k, "kaisuu : %d\n", i++);
    HAL_UART_Transmit(&huart2, (uint8_t *)(&k[0]), strlen(&k[0]), 100);

    //move the value from temp to buffer
    if(isxready == true){
      Accl.xLSB = Accl_temp.xLSB;
      Accl.xMSB = Accl_temp.xMSB;     
      
      xAccl = ((Accl.xMSB*256) + (Accl.xLSB & 0xF0))/16;
      if(xAccl>2047) xAccl -= 4096;
      xAccl = xAccl*0.0098;

      isxready = false;
      Show_float("xAccl = ",xAccl);
    }

    if(isyready == true){
      Accl.yLSB = Accl_temp.yLSB;
      Accl.yMSB = Accl_temp.yMSB;
      
      yAccl = ((Accl.yMSB*256) + (Accl.yLSB & 0xF0))/16;
      if(yAccl>2047) yAccl -= 4096; 
      yAccl = yAccl*0.0098;

      
      isyready = false;
      Show_float("yAccl = ",yAccl);

    }

    if(iszready == true){
      Accl.zLSB = Accl_temp.zLSB;
      Accl.zMSB = Accl_temp.zMSB;

      zAccl = ((Accl.zMSB*256) + (Accl.zLSB & 0xF0))/16;
      if(zAccl>2047) zAccl -= 4096; 
      zAccl = zAccl*0.0098;
      
      iszready = false;
      Show_float("zAccl = ",zAccl);

    }

//     HAL_I2C_Mem_Read_DMA(&hi2c1, Addr_Accl << 1, 0x02, 1, &Accl_temp, 6);
/*
    if (isxready == true) {
      sprintf(k, "xready\n");
      HAL_UART_Transmit(&huart2, (uint8_t *)(&k[0]), strlen(&k[0]), 100);
    }
*/  

    HAL_Delay(1000);

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}



/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection =
      RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
   */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/** BMX Init*/
void BMX_Init(void){
// init x axis 

//select PMU band width resisiter and set to 7.81 Hz
uint16_t Mem_PMU_BW = 0x10;
uint8_t p_BW = 0x08;
HAL_I2C_Mem_Write(&hi2c1,Addr_Accl<<1,Mem_PMU_BW,1,&p_BW,1,100);
HAL_Delay(100);

//select PMU range resisiter and set to -2
uint16_t Mem_PMU_Range = 0x0F;
uint8_t p_Range = 0x03;
HAL_I2C_Mem_Write(&hi2c1,Addr_Accl<<1,Mem_PMU_Range,1,&p_Range,1,100);
HAL_Delay(100);

//select PMU LPW resisiter and Normal mode
uint16_t Mem_PMU_LPW = 0x11;
uint8_t p_LPW = 0x00;
HAL_I2C_Mem_Write(&hi2c1,Addr_Accl<<1,Mem_PMU_LPW,1,&p_LPW,1,100);
HAL_Delay(100);

}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c1) {
  if((Accl_temp.xLSB && 0b00000001 == 0b00000001)) isxready = true;
  if((Accl_temp.yLSB && 0b00000001 == 0b00000001)) isyready = true;
  if((Accl_temp.zLSB && 0b00000001 == 0b00000001)) iszready = true;
  HAL_I2C_Mem_Read_DMA(hi2c1, Addr_Accl << 1, 0x02, 1, &Accl_temp, 6); //関数定義でhi2cのポインタを引数として渡しているため、HAL_I2C_Mem_Read_DMAの引数は&hi2c1ではなくhi2c1
}

void Show_float(char *str, float value){
  char temp[20];
  char *tempsign = (value < 0) ? "-":"";
  double tempval = (value<0) ? -1*value:value;
  int tempInt1 = (int)tempval;
  float tempfrac = tempval - tempInt1;
  int tempInt2 = trunc(tempfrac*100000);  

  sprintf(temp,"%s%s%d.%04d\n",str,tempsign,tempInt1,tempInt2);
  HAL_UART_Transmit (&huart2,(uint8_t*)(&temp[0]),strlen(&temp[0]),100);
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(char *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line)
   */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
