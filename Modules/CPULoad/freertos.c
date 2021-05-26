/* Includes ------------------------------------------------------------------*/
#ifdef USE_PRECISION_SYSTICK
#include <PrecisionSysTick/PrecisionSysTick.h>
#endif
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);
float convertRunTimeCounterValue(unsigned long ticks);

/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
void configureTimerForRunTimeStats(void)
{
#if( configINCLUDE_FREERTOS_TASK_C_ADDITIONS_H == 1 && configUSE_TRACE_FACILITY==1)
    /* only needed for openOCD or Segger FreeRTOS thread awareness. It needs the symbol uxTopUsedPriority present after linking */
    {
        extern const int uxTopUsedPriority;
        __attribute__((__unused__)) volatile uint8_t dummy_value_for_openocd;
        dummy_value_for_openocd = uxTopUsedPriority;
    }
    /* reference FreeRTOSDebugConfig, otherwise it might get removed by the linker or optimizations */
    {
        extern const uint8_t FreeRTOSDebugConfig[];
        if (FreeRTOSDebugConfig[0]==0) { /* just use it, so the linker cannot remove FreeRTOSDebugConfig[] */
            for(;;); /* FreeRTOSDebugConfig[0] should always be non-zero, so this should never happen! */
        }
    }
#endif
#if configHEAP_SCHEME_IDENTIFICATION
    extern const uint8_t freeRTOSMemoryScheme; /* constant for NXP Kernel Awareness to indicate heap scheme */
  if (freeRTOSMemoryScheme>100) { /* reference/use variable so it does not get optimized by the linker */
    for(;;);
  }
#endif
}

inline unsigned long getRunTimeCounterValue(void)
{
#ifdef USE_PRECISION_SYSTICK
	return HAL_GetHighResTick();
#else
	return HAL_GetTick();
#endif
}

float convertRunTimeCounterValue(unsigned long ticks)
{
#ifdef USE_PRECISION_SYSTICK
    return HAL_Tick2Time(ticks);
#else
    return (float)ticks / HAL_GetTickFreq();
#endif
}

/* Arduino-similar function to get milli seconds count based on FreeRTOS ticks, hence only in FreeRTOS tick resolution */
uint32_t millis()
{
	 return (1000 * xTaskGetTickCount()) / configTICK_RATE_HZ;
}