/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_hal_timebase_TIM.c 
  * @brief   HAL time base based on the hardware TIM.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include <stm32g4xx_hal.h>
#include <stm32g4xx_hal_tim.h>
 
#ifdef USE_FREERTOS
#include <cmsis_os.h>
#endif

#ifndef STM32G4_PRECISIONSYSTICK_FREQUENCY
#define STM32G4_PRECISIONSYSTICK_FREQUENCY 100000 // 100 kHz
#endif

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static TIM_HandleTypeDef        htim15;
static __IO uint32_t uwTickHighRes;
static uint32_t frequency;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  This function configures the TIM15 as a time base source. 
  *         The time source is configured  to have 1ms time base with a dedicated 
  *         Tick interrupt priority. 
  * @note   This function is called  automatically at the beginning of program after
  *         reset by HAL_Init() or at any time when clock is configured, by HAL_RCC_ClockConfig(). 
  * @param  TickPriority: Tick interrupt priority.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
  RCC_ClkInitTypeDef    clkconfig;
  uint32_t              uwTimclock = 0;
  uint32_t              uwPrescalerValue = 0;
  uint32_t              pFLatency;
  
  /*Configure the TIM15 IRQ priority */
  HAL_NVIC_SetPriority(TIM1_BRK_TIM15_IRQn, TickPriority ,0); 
  
  /* Enable the TIM15 global Interrupt */
  HAL_NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn); 
  
  /* Enable TIM15 clock */
  __HAL_RCC_TIM15_CLK_ENABLE();
  
  /* Get clock configuration */
  HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);
  
  /* Compute TIM15 clock */
  uwTimclock = HAL_RCC_GetPCLK2Freq();
   
  frequency = STM32G4_PRECISIONSYSTICK_FREQUENCY;

  /* Compute the prescaler value to have TIM15 counter clock equal to the frequency */
  uwPrescalerValue = (uint32_t) ((uwTimclock / frequency) - 1);

  // Recompute the exact timer frequency using the set prescaler
  frequency = uwTimclock / (uwPrescalerValue + 1);
  
  /* Initialize TIM15 */
  htim15.Instance = TIM15;
  
  /* Initialize TIMx peripheral as follow:
  + Period = [(TIM15CLK/1000) - 1]. to have a (1/1000) s time base.
  + Prescaler = (uwTimclock/1000000 - 1) to have a 1MHz counter clock.
  + ClockDivision = 0
  + Counter direction = Up
  */
  // Modified by Thomas: Configure timer to overflow and thus firing an interrupt @ 10 Hz rate
  htim15.Init.Period = (frequency / 10) - 1;
  htim15.Init.Prescaler = uwPrescalerValue;
  htim15.Init.ClockDivision = 0;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  if(HAL_TIM_Base_Init(&htim15) == HAL_OK)
  {
    /* Start the TIM time Base generation in interrupt mode */
    return HAL_TIM_Base_Start_IT(&htim15);
  }
  
  /* Return function status */
  return HAL_ERROR;
}

uint32_t HAL_GetTickTimerValue(void)
{
	return (uint32_t)__HAL_TIM_GET_COUNTER(&htim15);
}

/**
  * @brief  Suspend Tick increment.
  * @note   Disable the tick increment by disabling TIM15 update interrupt.
  * @param  None
  * @retval None
  */
void HAL_SuspendTick(void)
{
  /* Disable TIM15 update Interrupt */
  __HAL_TIM_DISABLE_IT(&htim15, TIM_IT_UPDATE);                                                  
}

/**
  * @brief  Resume Tick increment.
  * @note   Enable the tick increment by Enabling TIM15 update interrupt.
  * @param  None
  * @retval None
  */
void HAL_ResumeTick(void)
{
  /* Enable TIM15 Update interrupt */
  __HAL_TIM_ENABLE_IT(&htim15, TIM_IT_UPDATE);
}


/**
  * @brief This function is called to increment  a global variable "uwTick"
  *        used as application time base.
  * @note In the default implementation, this variable is incremented each 1ms
  *       in Systick ISR.
 * @note This function is declared as __weak to be overwritten in case of other
  *      implementations in user file.
  * @retval None
  */
void HAL_IncTick(void)
{
    // Called when timer overflows at 10 Hz
	uwTickHighRes += htim15.Init.Period + 1;
}

uint32_t HAL_GetHighResTick(void)
{
	return (uwTickHighRes + HAL_GetTickTimerValue());
}

uint32_t HAL_tic()
{
	return HAL_GetHighResTick();
}

float HAL_toc(uint32_t timerPrev)
{
	uint32_t timerDelta;
	uint32_t timerNow = HAL_GetHighResTick();
	if (timerNow > timerPrev)
		timerDelta = timerNow - timerPrev;
	else
		timerDelta = ((uint32_t)0xFFFFFFFF - timerPrev) + timerNow;

	float microsTime = (float)timerDelta / frequency;
	return microsTime;
}

#ifdef USE_FREERTOS
void HAL_Delay(uint32_t Delay)
{
	osDelay(Delay);
}
#else
void HAL_Delay(uint32_t Delay)
{
  uint32_t tickstart = HAL_GetTick();
  uint32_t wait = Delay;

  /*
  // Add a freq to guarantee minimum wait
  if (wait < HAL_MAX_DELAY)
  {
    wait += (uint32_t)(uwTickFreq);
  }
  */

  while ((HAL_GetTick() - tickstart) < wait)
  {
  }
}
#endif

void HAL_DelayHighRes(uint32_t Delay)
{
  uint32_t tickstart = HAL_GetHighResTick();
  uint32_t wait = Delay;

  /*
  // Add a freq to guarantee minimum wait
  if (wait < HAL_MAX_DELAY)
  {
    wait += (uint32_t)(uwTickFreq);
  }
  */

  while ((HAL_GetHighResTick() - tickstart) < wait)
  {
  }
}

/**
  * @brief Provides a tick value in millisecond.
  * @note This function is declared as __weak to be overwritten in case of other
  *       implementations in user file.
  * @retval tick value
  */
uint32_t HAL_GetTick(void)
{
  return (HAL_GetHighResTick() / 100); // divide by 100 since we have configured HAL timer (HAL_InitTick) to run at 100 kHz
}

float HAL_GetTime(void)
{
	return (float)HAL_GetHighResTick() / frequency;
}

/**
  * @brief This function handles TIM1 break interrupt and TIM15 global interrupt.
  */
void TIM1_BRK_TIM15_IRQHandler(void)
{
	/* TIM Update event */
	if(__HAL_TIM_GET_FLAG(&htim15, TIM_FLAG_UPDATE) != RESET)
	{
		if(__HAL_TIM_GET_IT_SOURCE(&htim15, TIM_IT_UPDATE) !=RESET)
		{
			__HAL_TIM_CLEAR_IT(&htim15, TIM_IT_UPDATE);
			HAL_IncTick();
		}
	}
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
