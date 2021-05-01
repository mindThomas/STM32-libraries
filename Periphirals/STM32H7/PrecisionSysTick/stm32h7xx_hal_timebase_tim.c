/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_tim.h"

#ifndef HAL_SYSTICK_FREQUENCY
#define HAL_SYSTICK_FREQUENCY 10000 // 10 kHz
#endif

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim16;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

static __IO uint32_t uwTickHighRes;

/**
 * @brief  This function configures the TIM16 as a time base source.
 *         The time source is configured  to have 1ms time base with a dedicated
 *         Tick interrupt priority.
 * @note   This function is called  automatically at the beginning of program after
 *         reset by HAL_Init() or at any time when clock is configured, by HAL_RCC_ClockConfig().
 * @param  TickPriority: Tick interrupt priority.
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
    RCC_ClkInitTypeDef clkconfig;
    uint32_t           uwPrescalerValue = 0;
    uint32_t           pFLatency;

    /*Configure the TIM16 IRQ priority */
    HAL_NVIC_SetPriority(TIM16_IRQn, TickPriority, 0);

    /* Enable the TIM16 global Interrupt */
    HAL_NVIC_EnableIRQ(TIM16_IRQn);

    /* Enable TIM16 clock */
    __HAL_RCC_TIM16_CLK_ENABLE();

    /* Get clock configuration */
    HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);

    /* Compute TIM16 clock */
    // Configure timer prescaler based on desired frequency
    //   fCNT = (ARR+1) * fPERIOD
    //   PSC = (fTIM / fCNT) - 1
    uint32_t TimerClock = 2 * HAL_RCC_GetPCLK2Freq();

    /* Compute the prescaler value to have TIM16 counter clock equal to 10 kHz */
    uwPrescalerValue = (uint32_t)((TimerClock / HAL_SYSTICK_FREQUENCY) - 1);

    /* Initialize TIM16 */
    htim16.Instance = TIM16;

    /* Initialize TIMx peripheral as follow:
    + Period = [(TIM16CLK/1000) - 1]. to have a (1/1000) s time base.
    + Prescaler = (uwTimclock/1000000 - 1) to have a 1MHz counter clock.
    + ClockDivision = 0
    + Counter direction = Up
    */
    // Modified by Thomas: Configure timer to overflow every second, thus firing an interrupt @ 1 Hz rate
    htim16.Init.Period        = (HAL_SYSTICK_FREQUENCY / 1) - 1;
    htim16.Init.Prescaler     = uwPrescalerValue;
    htim16.Init.ClockDivision = 0;
    htim16.Init.CounterMode   = TIM_COUNTERMODE_UP;
    if (HAL_TIM_Base_Init(&htim16) == HAL_OK) {
        /* Start the TIM time Base generation in interrupt mode */
        return HAL_TIM_Base_Start_IT(&htim16);
    }

    /* Return function status */
    return HAL_ERROR;
}

uint32_t HAL_GetTickTimerValue(void)
{
    return (uint32_t)__HAL_TIM_GET_COUNTER(&htim16);
}

/**
 * @brief  Suspend Tick increment.
 * @note   Disable the tick increment by disabling TIM16 update interrupt.
 * @param  None
 * @retval None
 */
void HAL_SuspendTick(void)
{
    /* Disable TIM16 update Interrupt */
    __HAL_TIM_DISABLE_IT(&htim16, TIM_IT_UPDATE);
}

/**
 * @brief  Resume Tick increment.
 * @note   Enable the tick increment by Enabling TIM16 update interrupt.
 * @param  None
 * @retval None
 */
void HAL_ResumeTick(void)
{
    /* Enable TIM16 Update interrupt */
    __HAL_TIM_ENABLE_IT(&htim16, TIM_IT_UPDATE);
}

/**
 * @brief This function handles TIM16 global interrupt.
 */
void TIM16_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim16);
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM16 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM16) {
        HAL_IncTick();
    }
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
    uwTickHighRes += HAL_SYSTICK_FREQUENCY; // timer update rate configured 1 Hz but with timer count frequency running at 10 kHz
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

    float microsTime = (float)timerDelta / HAL_SYSTICK_FREQUENCY;
    return microsTime;
}

void HAL_Delay(uint32_t Delay)
{
    uint32_t tickstart = HAL_GetTick();
    uint32_t wait      = Delay;

    /*
    // Add a freq to guarantee minimum wait
    if (wait < HAL_MAX_DELAY)
    {
      wait += (uint32_t)(uwTickFreq);
    }
    */

    while ((HAL_GetTick() - tickstart) < wait) {
    }
}

void HAL_DelayHighRes(uint32_t Delay)
{
    uint32_t tickstart = HAL_GetHighResTick();
    uint32_t wait      = Delay;

    /*
    // Add a freq to guarantee minimum wait
    if (wait < HAL_MAX_DELAY)
    {
      wait += (uint32_t)(uwTickFreq);
    }
    */

    while ((HAL_GetHighResTick() - tickstart) < wait) {
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
    return (uwTickHighRes /
            10); // divide by 10 since we have configured HAL timer to run at 10 kHz in stm32h7xx_hal_timebase_tim.c
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
