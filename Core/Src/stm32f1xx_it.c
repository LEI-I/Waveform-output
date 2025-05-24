/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "arm_math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SEQUENCE_LENGTH 4
#define KEY1_VALUE 1
#define KEY2_VALUE 2
#define OUTPUT_TYPE_COUNT 3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
FlagStatus modeSelectFlag = SET;
FlagStatus dinoStrat = RESET;
extern FlagStatus dinoFlag;
OutputType outputType = OUTPUT_TYPE_DC;
uint16_t dcDuty=0, sawtoothDuty=0;
uint8_t sinFrequency = 1, sawtoothFrequency = 8;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void CheckModeSelectSequence(void)
{
    static const uint8_t modeSequence[SEQUENCE_LENGTH] = {KEY1_VALUE, KEY2_VALUE, KEY1_VALUE, KEY2_VALUE};
    static uint8_t inputSequence[SEQUENCE_LENGTH] = {0};
    static uint8_t sequenceIndex = 0;
    uint8_t key = 0;
    
    if(HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET) {
        key = KEY1_VALUE;
    } else if(HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET) {
        key = KEY2_VALUE;
    } else {
        return; // 没有按键按下,保证没有队列不会空移
    }

    // 滑动窗口：前移，加入新按键
    for(uint8_t i = 0; i < SEQUENCE_LENGTH - 1; i++) {
        inputSequence[i] = inputSequence[i + 1];
    }
    inputSequence[SEQUENCE_LENGTH - 1] = key;

    // 判断是否匹配
    uint8_t match = 1;
    for(uint8_t i = 0; i < SEQUENCE_LENGTH; i++) {
        if(inputSequence[i] != modeSequence[i]) {
            match = 0;
            break;
        }
    }
    if(match) {
        modeSelectFlag = 1;
        // 匹配后可清空输入序列
        for(uint8_t i = 0; i < SEQUENCE_LENGTH; i++) {
            inputSequence[i] = 0;
        }
    }
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_i2c1_rx;
extern DMA_HandleTypeDef hdma_i2c1_tx;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel6 global interrupt.
  */
void DMA1_Channel6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel6_IRQn 0 */

  /* USER CODE END DMA1_Channel6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2c1_tx);
  /* USER CODE BEGIN DMA1_Channel6_IRQn 1 */

  /* USER CODE END DMA1_Channel6_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel7 global interrupt.
  */
void DMA1_Channel7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel7_IRQn 0 */

  /* USER CODE END DMA1_Channel7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2c1_rx);
  /* USER CODE BEGIN DMA1_Channel7_IRQn 1 */

  /* USER CODE END DMA1_Channel7_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt.
  */
void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */

  /* USER CODE END TIM1_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */

  /* USER CODE END TIM1_UP_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles I2C1 event interrupt.
  */
void I2C1_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_EV_IRQn 0 */

  /* USER CODE END I2C1_EV_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&hi2c1);
  /* USER CODE BEGIN I2C1_EV_IRQn 1 */

  /* USER CODE END I2C1_EV_IRQn 1 */
}

/**
  * @brief This function handles I2C1 error interrupt.
  */
void I2C1_ER_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_ER_IRQn 0 */

  /* USER CODE END I2C1_ER_IRQn 0 */
  HAL_I2C_ER_IRQHandler(&hi2c1);
  /* USER CODE BEGIN I2C1_ER_IRQn 1 */

  /* USER CODE END I2C1_ER_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim == (&htim2)) {
        if(modeSelectFlag == RESET) {
            CheckModeSelectSequence();
            if(HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET) {
                switch (outputType) {
                    case OUTPUT_TYPE_DC:
                        if(dcDuty > 10) dcDuty -= 10;
                        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, dcDuty);
                        break;
                    case OUTPUT_TYPE_SIN:
                        if(sinFrequency > 1) sinFrequency--;
                        break;
                    case OUTPUT_TYPE_SAWTOOTH:
                        if(sawtoothFrequency > 8) sawtoothFrequency -= 8;
                        break;
                    case OUTPUT_TYPE_DINO:
                        dinoStrat = SET;
                    default:
                        break;
                }
            }
            if(HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET) {
                switch (outputType) {
                    case OUTPUT_TYPE_DC:
                        if(dcDuty < 1000) dcDuty += 10;
                        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, dcDuty);
                        break;
                    case OUTPUT_TYPE_SIN:
                        if(sinFrequency < 100) sinFrequency++;
                        break;
                    case OUTPUT_TYPE_SAWTOOTH:
                        if(sawtoothFrequency < 96) sawtoothFrequency += 8;
                        break;
                    default:
                        break;
                }
            }
        }
        else {
            if(HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET) {
                outputType++;
                outputType %= OUTPUT_TYPE_COUNT;
            }
            if(HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET) {
                modeSelectFlag = RESET;
            }
        }
    }
    if(htim == (&htim1)) {
        if(outputType == OUTPUT_TYPE_SIN) {
            static int sinPWM = 0;
            htim1.Instance->CCR1 = (int)((arm_sin_f32(sinPWM*2.0*3.1415926/(8000.0 / sinFrequency)) + 1.0) * 499);
            sinPWM++;
            if(sinPWM >= htim1.Instance->ARR)
                sinPWM = 0;
        }
        else {
            if(outputType == OUTPUT_TYPE_SAWTOOTH) {
                sawtoothDuty += (1 * sawtoothFrequency / 8);
                htim1.Instance->CCR1 = sawtoothDuty;
                if(sawtoothDuty > 1000)
                    sawtoothDuty = 0;
            }
        }
    }
}


#if 0
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim == (&htim2)) {
        if(HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET && dcDuty < 100) {
            dcDuty++;
            if(state == 0) {
                state = 1;
            }
            if(state == 2) {
                state = 3;
            } else {
                if(state == 3) {
                    state = 0;
                }
            }
        }
        if(HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET && dcDuty > 0) {
            dcDuty--;
            if(state == 1) {
                state = 2;
            } else {
                if(state == 2) {
                    state = 0;
                }
            }
            if(state == 3) {
                state = 4;
            }
        }
                __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, dcDuty);
    }
}
#endif
/* USER CODE END 1 */
