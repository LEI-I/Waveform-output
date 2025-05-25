/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "retarget.h"
#include "Dino.h"
#include "stm32f1xx_it.h"
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
extern uint8_t modeSelectFlag;
uint8_t message[20];
extern uint16_t dcDuty, sawtoothDuty;
extern OutputType outputType;
extern uint8_t sinFrequency, sawtoothFrequency;
FlagStatus dinoFlag = RESET;
extern FlagStatus dinoStart;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(50);
  OLED_Init();
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      if(modeSelectFlag == SET) {
          OLED_NewFrame();
          OLED_PrintString(0, 0, "选择输出类型:", &selectOutputTypeFont, OLED_COLOR_NORMAL);
          switch(outputType) {
              case OUTPUT_TYPE_DC:
                  OLED_DrawImage(40, 21, &DCImgFont45x45, OLED_COLOR_NORMAL); break;
              case OUTPUT_TYPE_SIN:
                  OLED_DrawImage(40, 21, &sinImg, OLED_COLOR_NORMAL); break;
              case OUTPUT_TYPE_SAWTOOTH:
                  OLED_DrawImage(40, 21, &sawtoothImg45x45, OLED_COLOR_NORMAL); break;
              case OUTPUT_TYPE_DINO:
                  OLED_DrawImage(40, 21, &dinoImg45x45, OLED_COLOR_NORMAL); break;
              default:
                  OLED_PrintString(0, 13, "Error!", &font16x16, OLED_COLOR_NORMAL); break;
          }
          OLED_ShowFrame();
      }
      else {
          switch (outputType) {
              case OUTPUT_TYPE_LOGO:
                  OLED_NewFrame();
                  OLED_DrawImage(35, 15, &logoImg50x50, OLED_COLOR_NORMAL);
                  OLED_ShowFrame();
                  HAL_Delay(100);
                  break;
              case OUTPUT_TYPE_DC:
                  sprintf((char *)message, "设定值:%d%%", dcDuty/10);
                  OLED_NewFrame();
                  OLED_PrintString(0, 0, "当前输出-DC", &messageDisplayFont12x12, OLED_COLOR_NORMAL);
                  OLED_PrintString(0, 17, (char *)message, &DCoutputValueFont12x12, OLED_COLOR_NORMAL);
                  OLED_DrawRectangle(13, 40, 101, 12, OLED_COLOR_NORMAL);
                  OLED_DrawFilledRectangle(13, 41, dcDuty/10, 11, OLED_COLOR_NORMAL);
                  OLED_ShowFrame();
                  break;
              case OUTPUT_TYPE_SIN:
                  sprintf((char *)message, "设定值:%dHz", sinFrequency);
                  OLED_NewFrame();
                  OLED_PrintString(0, 0, "当前输出-Sin", &messageDisplayFont12x12, OLED_COLOR_NORMAL);
                  OLED_PrintString(0, 17, (char *)message, &DCoutputValueFont12x12, OLED_COLOR_NORMAL);
                  OLED_DrawRectangle(13, 40, 101, 12, OLED_COLOR_NORMAL);
                  OLED_DrawFilledRectangle(13, 41, sinFrequency, 11, OLED_COLOR_NORMAL);
                  OLED_ShowFrame();
                  break;
              case OUTPUT_TYPE_SAWTOOTH:
                  sprintf((char *)message, "设定值:%dHz", sawtoothFrequency);
                  OLED_NewFrame();
                  OLED_PrintString(0, 0, "当前输出-Sawtooth", &messageDisplayFont12x12, OLED_COLOR_NORMAL);
                  OLED_PrintString(0, 17, (char *)message, &DCoutputValueFont12x12, OLED_COLOR_NORMAL);
                  OLED_DrawRectangle(13, 40, 101, 12, OLED_COLOR_NORMAL);
                  OLED_DrawFilledRectangle(13, 41, sawtoothFrequency, 11, OLED_COLOR_NORMAL);
                  OLED_ShowFrame();
                  break;
              case OUTPUT_TYPE_DINO:
                  if(dinoFlag == RESET) {
                      introMessage();
                      dinoFlag = SET;
                  }
                  if(dinoStart == SET) {
                      showLine();
                      play();
                      HAL_Delay(100);
                  }
                  break;
              default:
                  OLED_NewFrame();
                  OLED_PrintString(0, 0, "Error!", &font12x12, OLED_COLOR_NORMAL);
                  OLED_ShowFrame();
                  break;
          }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
