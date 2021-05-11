#ifndef __STM32G4xx_HAL_TIMEBASE_TIM_H
#define __STM32G4xx_HAL_TIMEBASE_TIM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stm32g4xx_hal.h>

uint32_t HAL_GetTickTimerValue(void);
uint32_t HAL_GetHighResTick(void);
void HAL_DelayHighRes(uint32_t Delay);
float HAL_GetTime(void);

uint32_t HAL_tic();
float HAL_toc(uint32_t timerPrev);

float HAL_Tick2Time(uint32_t ticks);

#ifdef __cplusplus
}
#endif

#endif /* __STM32G4xx_HAL_TIMEBASE_TIM_H */
