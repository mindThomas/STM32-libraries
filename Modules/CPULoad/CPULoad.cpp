/* Copyright (C) 2021- Thomas Jespersen, TKJ Electronics. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the MIT License
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the MIT License for further details.
 *
 * Contact information
 * ------------------------------------------
 * Thomas Jespersen, TKJ Electronics
 * Web      :  http://www.tkjelectronics.dk
 * e-mail   :  thomasj@tkjelectronics.dk
 * ------------------------------------------
 */

#include "CPULoad.hpp"

#include <Debug/Debug.h>

#include "task.h"

CPULoad::CPULoad(LSPC& lspc, uint32_t cpuLoadTaskPriority)
    : lspc_(lspc)
    , cpuLoadTaskHandle_(0)
{
    xTaskCreate(CPULoad::CPULoadThread, (char*)"CPU Load", CPULOAD_THREAD_STACK, (void*)this, cpuLoadTaskPriority,
                &cpuLoadTaskHandle_);
}

CPULoad::~CPULoad()
{
    if (cpuLoadTaskHandle_)
        vTaskDelete(cpuLoadTaskHandle_); // stop task
}

void CPULoad::CPULoadThread(void * pvParameters)
{
    CPULoad* obj  = (CPULoad*)pvParameters;
    LSPC&    lspc = obj->lspc_;

#if( configUSE_TRACE_FACILITY != 1 )
#error configUSE_TRACE_FACILITY must also be set to 1 in FreeRTOSConfig.h to use vTaskGetRunTimeStats().
#endif

    /* Send CPU load every second */
    char * pcWriteBuffer = (char *)pvPortMalloc(400); // Approx 40 bytes pr. task
    while (1)
    {
        obj->TaskRunTimeStats(pcWriteBuffer);
        obj->MemoryStats(pcWriteBuffer);
        osDelay(1000);
    }
}

#if 0
void CPULoad::CPULoadThread2(void * pvParameters)
{
    CPULoad* obj  = (CPULoad*)pvParameters;
    LSPC&    lspc = obj->_lspc;

#if( configUSE_TRACE_FACILITY != 1 )
#error configUSE_TRACE_FACILITY must also be set to 1 in FreeRTOSConfig.h to use vTaskGetRunTimeStats().
#endif

    /* Send CPU load every second */
    char * pcWriteBuffer = (char *)pvPortMalloc(800); // Approx 40 bytes pr. task
    while (1)
    {
        TaskRunTimeStats(pcWriteBuffer);
        char * endPtr = &pcWriteBuffer[strlen(pcWriteBuffer)];
        *endPtr++ = '\n'; *endPtr++ = '\n'; *endPtr++ = 0;

        // Split into multiple packages and send
        uint16_t txIdx = 0;
        uint16_t remainingLength = strlen(pcWriteBuffer);
        uint16_t txLength;

        while (remainingLength > 0) {
            txLength = remainingLength;
            if (txLength > LSPC_MAXIMUM_PACKAGE_LENGTH) {
                txLength = LSPC_MAXIMUM_PACKAGE_LENGTH-1;
                while (pcWriteBuffer[txIdx+txLength] != '\n' && txLength > 0) txLength--; // find and include line-break (if possible)
                if (txLength == 0) txLength = LSPC_MAXIMUM_PACKAGE_LENGTH;
                else txLength++;
            }
            lspc.TransmitAsync(lspc::MessageTypesToPC::CPUload, (uint8_t *)&pcWriteBuffer[txIdx], txLength);

            txIdx += txLength;
            remainingLength -= txLength;
        }

        osDelay(1000);
    }
}
#endif

char * CPULoad::WriteTaskNameToBuffer(char * pcBuffer, const char * pcTaskName)
{
    size_t x;

    /* Start by copying the entire string. */
    strcpy( pcBuffer, pcTaskName );

    /* Pad the end of the string with spaces to ensure columns line up when
    printed out. */
    for( x = strlen( pcBuffer ); x < ( size_t ) ( configMAX_TASK_NAME_LEN - 1 ); x++ )
    {
        pcBuffer[ x ] = ' ';
    }

    /* Terminate. */
    pcBuffer[ x ] = 0x00;

    /* Return the new end of string. */
    return &( pcBuffer[ x ] );
}

// Copied from FreeRTOS/tasks.c since tskTCB is kept opaque
// Make sure to keep this updated
typedef struct tskTaskControlBlock 			/* The old naming convention is used to prevent breaking kernel aware debuggers. */
{
    volatile StackType_t	*pxTopOfStack;	/*< Points to the location of the last item placed on the tasks stack.  THIS MUST BE THE FIRST MEMBER OF THE TCB STRUCT. */

#if ( portUSING_MPU_WRAPPERS == 1 )
    xMPU_SETTINGS	xMPUSettings;		/*< The MPU settings are defined as part of the port layer.  THIS MUST BE THE SECOND MEMBER OF THE TCB STRUCT. */
#endif

    ListItem_t			xStateListItem;	/*< The list that the state list item of a task is reference from denotes the state of that task (Ready, Blocked, Suspended ). */
    ListItem_t			xEventListItem;		/*< Used to reference a task from an event list. */
    UBaseType_t			uxPriority;			/*< The priority of the task.  0 is the lowest priority. */
    StackType_t			*pxStack;			/*< Points to the start of the stack. */
    char				pcTaskName[ configMAX_TASK_NAME_LEN ];/*< Descriptive name given to the task when created.  Facilitates debugging only. */ /*lint !e971 Unqualified char types are allowed for strings and single characters only. */

#if ( ( portSTACK_GROWTH > 0 ) || ( configRECORD_STACK_HIGH_ADDRESS == 1 ) )
    StackType_t		*pxEndOfStack;		/*< Points to the highest valid address for the stack. */
#endif

#if ( portCRITICAL_NESTING_IN_TCB == 1 )
    UBaseType_t		uxCriticalNesting;	/*< Holds the critical section nesting depth for ports that do not maintain their own count in the port layer. */
#endif

#if ( configUSE_TRACE_FACILITY == 1 )
    UBaseType_t		uxTCBNumber;		/*< Stores a number that increments each time a TCB is created.  It allows debuggers to determine when a task has been deleted and then recreated. */
    UBaseType_t		uxTaskNumber;		/*< Stores a number specifically for use by third party trace code. */
#endif

#if ( configUSE_MUTEXES == 1 )
    UBaseType_t		uxBasePriority;		/*< The priority last assigned to the task - used by the priority inheritance mechanism. */
    UBaseType_t		uxMutexesHeld;
#endif

#if ( configUSE_APPLICATION_TASK_TAG == 1 )
    TaskHookFunction_t pxTaskTag;
#endif

#if( configNUM_THREAD_LOCAL_STORAGE_POINTERS > 0 )
    void			*pvThreadLocalStoragePointers[ configNUM_THREAD_LOCAL_STORAGE_POINTERS ];
#endif

#if( configGENERATE_RUN_TIME_STATS == 1 )
    uint32_t		ulRunTimeCounter;	/*< Stores the amount of time the task has spent in the Running state. */
#endif

#if ( configUSE_NEWLIB_REENTRANT == 1 )
    /* Allocate a Newlib reent structure that is specific to this task.
    Note Newlib support has been included by popular demand, but is not
    used by the FreeRTOS maintainers themselves.  FreeRTOS is not
    responsible for resulting newlib operation.  User must be familiar with
    newlib and must provide system-wide implementations of the necessary
    stubs. Be warned that (at the time of writing) the current newlib design
    implements a system-wide malloc() that must be provided with locks. */
    struct	_reent xNewLib_reent;
#endif

#if( configUSE_TASK_NOTIFICATIONS == 1 )
    volatile uint32_t ulNotifiedValue;
    volatile uint8_t ucNotifyState;
#endif

    /* See the comments in FreeRTOS.h with the definition of
    tskSTATIC_AND_DYNAMIC_ALLOCATION_POSSIBLE. */
#if( tskSTATIC_AND_DYNAMIC_ALLOCATION_POSSIBLE != 0 ) /*lint !e731 !e9029 Macro has been consolidated for readability reasons. */
    uint8_t	ucStaticallyAllocated; 		/*< Set to pdTRUE if the task is a statically allocated to ensure no attempt is made to free the memory. */
#endif

#if( INCLUDE_xTaskAbortDelay == 1 )
    uint8_t ucDelayAborted;
#endif

#if( configUSE_POSIX_ERRNO == 1 )
    int iTaskErrno;
#endif

} tskTCB;

void CPULoad::TaskRunTimeStats(char * pcWriteBuffer)
{
    // Based on vTaskGetRunTimeStats
    //static TaskStatus_t *pxTaskStatusArrayPrev = 0;
    //static UBaseType_t uxArraySizePrev = 0;
    //static uint32_t ulTotalTimeD100Prev = 0;
    static const auto TICK_FREQUENCY = 1.f/portCONVERT_RUN_TIME_COUNTER_VALUE(1);

    uint32_t ulTotalTime, ulTotalTimeD100, ulStatsAsPercentage, ulOverallStatsAsPercentage;
    TaskStatus_t *pxTaskStatusArray;
    UBaseType_t uxArraySize;

    char * bufferStart = pcWriteBuffer;

    *pcWriteBuffer = 0x00; // ensure that we start clean

    uxArraySize = uxTaskGetNumberOfTasks();
    pxTaskStatusArray = (TaskStatus_t*)pvPortMalloc( uxArraySize * sizeof( TaskStatus_t ) );

    unsigned long numTasksPrinted = 0;

    if( pxTaskStatusArray != NULL )
    {
        uxArraySize = uxTaskGetSystemState( pxTaskStatusArray, uxArraySize, &ulTotalTime );

        ulTotalTimeD100 = ulTotalTime / 100UL; // For percentage calculations.

        if( ulTotalTimeD100 > 0 )
        {
            strcpy( pcWriteBuffer, "Prio\tTask name" );
            pcWriteBuffer += strlen( pcWriteBuffer );
            for( UBaseType_t x = 0; x < ( size_t ) ( configMAX_TASK_NAME_LEN - 10 ); x++ )
            {
                *pcWriteBuffer++ = ' ';
            }
#ifdef CPULOAD_USE_VERBOSE
            strcpy( pcWriteBuffer, "\tTotal     \tUtil\tOverall\tState    \tStack alloc\tStack left\r\n" );
#else
            strcpy( pcWriteBuffer, "\tUtil\tOverall\tState    \tStack alloc\tStack left\r\n" );
#endif
            pcWriteBuffer += strlen( pcWriteBuffer );
            lspc_.TransmitAsync(lspc::MessageTypesToPC::CPUload, (uint8_t *)bufferStart, pcWriteBuffer - bufferStart);
            pcWriteBuffer = bufferStart;

            const auto deltaRuntime = (ulTotalTime - prevTotalRunTime) / 100UL;

            while (numTasksPrinted < uxArraySize)
            {
                UBaseType_t taskIdx = 0;
                UBaseType_t priority = configLIBRARY_LOWEST_INTERRUPT_PRIORITY + 1;
                for( UBaseType_t x = 0; x < uxArraySize; x++ ) {
                    if (pxTaskStatusArray[x].pxStackBase != 0 && pxTaskStatusArray[x].uxCurrentPriority <= priority) {
                        taskIdx = x;
                        priority = pxTaskStatusArray[x].uxCurrentPriority;
                    }
                }

                if (prevTaskRunTime.find(pxTaskStatusArray[taskIdx].xHandle) == prevTaskRunTime.end()) {
                    prevTaskRunTime[pxTaskStatusArray[taskIdx].xHandle] = pxTaskStatusArray[taskIdx].ulRunTimeCounter;
                }

                ulStatsAsPercentage = (pxTaskStatusArray[taskIdx].ulRunTimeCounter - prevTaskRunTime[pxTaskStatusArray[taskIdx].xHandle]) / deltaRuntime;

                if (ulStatsAsPercentage > 100)
                    ulStatsAsPercentage = 0;

                ulOverallStatsAsPercentage = pxTaskStatusArray[taskIdx].ulRunTimeCounter / ulTotalTimeD100;

                if (ulOverallStatsAsPercentage > 100)
                    ulOverallStatsAsPercentage = 0;

                sprintf( pcWriteBuffer, "%u\t", ( unsigned int ) pxTaskStatusArray[taskIdx].uxCurrentPriority );
                pcWriteBuffer += strlen( pcWriteBuffer );

                pcWriteBuffer = WriteTaskNameToBuffer( pcWriteBuffer, pxTaskStatusArray[taskIdx].pcTaskName );

#ifdef CPULOAD_USE_VERBOSE
                sprintf(pcWriteBuffer, "\t%-10lu", pxTaskStatusArray[taskIdx].ulRunTimeCounter);
                pcWriteBuffer += strlen( pcWriteBuffer );
#endif

                if( ulStatsAsPercentage > 0UL )
                {
                    //sprintf( pcWriteBuffer, "\t%u\t\t%u%%\t", ( unsigned int ) pxTaskStatusArray[taskIdx].ulRunTimeCounter, ( unsigned int ) ulStatsAsPercentage );
                    sprintf( pcWriteBuffer, "\t%u%%", ( unsigned int ) ulStatsAsPercentage );
                }
                else
                {
                    //sprintf( pcWriteBuffer, "\t%u\t\t<1%%\t", ( unsigned int ) pxTaskStatusArray[taskIdx].ulRunTimeCounter );
                    strcpy( pcWriteBuffer, "\t<1%" );
                }
                pcWriteBuffer += strlen( pcWriteBuffer );

                if( ulOverallStatsAsPercentage > 0UL )
                {
                    //sprintf( pcWriteBuffer, "\t%u\t\t%u%%\t", ( unsigned int ) pxTaskStatusArray[taskIdx].ulRunTimeCounter, ( unsigned int ) ulOverallStatsAsPercentage );
                    sprintf( pcWriteBuffer, "\t%u%%\t", ( unsigned int ) ulOverallStatsAsPercentage );
                }
                else
                {
                    //sprintf( pcWriteBuffer, "\t%u\t\t<1%%\t", ( unsigned int ) pxTaskStatusArray[taskIdx].ulRunTimeCounter );
                    strcpy( pcWriteBuffer, "\t<1%\t" );
                }
                pcWriteBuffer += strlen( pcWriteBuffer );

                switch (pxTaskStatusArray[taskIdx].eCurrentState) {
                    case eRunning:
                        strcpy(pcWriteBuffer, "[RUNNING]");
                        break;
                    case eReady:
                        strcpy(pcWriteBuffer, "[READY]  ");
                        break;
                    case eBlocked:
                        strcpy(pcWriteBuffer, "[BLOCKED]");
                        break;
                    case eSuspended:
                        strcpy(pcWriteBuffer, "[SUSPENDED]");
                        break;
                    case eDeleted:
                        strcpy(pcWriteBuffer, "[DELETED]");
                        break;
                    case eInvalid:
                        strcpy(pcWriteBuffer, "[INVALID]");
                        break;
                    default:
                        strcpy(pcWriteBuffer, "[UNKNOWN]");
                        break;
                }
                pcWriteBuffer += strlen( pcWriteBuffer );

                // Words = 4 bytes
                sprintf( pcWriteBuffer, "\t%u words", (unsigned int)(pxTaskStatusArray[taskIdx].xHandle->pxEndOfStack - pxTaskStatusArray[taskIdx].xHandle->pxStack + 2));
                pcWriteBuffer += strlen( pcWriteBuffer );

                sprintf( pcWriteBuffer, "\t%u words\t", pxTaskStatusArray[taskIdx].usStackHighWaterMark);
                pcWriteBuffer += strlen( pcWriteBuffer );

                *pcWriteBuffer++ = '\r';
                *pcWriteBuffer++ = '\n';
                *pcWriteBuffer = 0;

                lspc_.TransmitAsync(lspc::MessageTypesToPC::CPUload, (uint8_t *)bufferStart, pcWriteBuffer - bufferStart);
                pcWriteBuffer = bufferStart;

                // Mark task as printed
                pxTaskStatusArray[taskIdx].pxStackBase = 0;
                prevTaskRunTime[pxTaskStatusArray[taskIdx].xHandle] = pxTaskStatusArray[taskIdx].ulRunTimeCounter;
                numTasksPrinted++;
            }
        }

#ifdef CPULOAD_USE_VERBOSE
        sprintf(pcWriteBuffer, "\r\nTotal Ticks @ %.2f Hz: %lu", TICK_FREQUENCY, ulTotalTime);
        pcWriteBuffer += strlen( pcWriteBuffer );
        lspc_.TransmitAsync(lspc::MessageTypesToPC::CPUload, (uint8_t *)bufferStart, pcWriteBuffer - bufferStart);
        pcWriteBuffer = bufferStart;
#endif

        const auto totalRunTime = portCONVERT_RUN_TIME_COUNTER_VALUE(ulTotalTime);

        sprintf(pcWriteBuffer, "\r\nTotal Time: %.5f s\r\n\r\n", totalRunTime);
        pcWriteBuffer += strlen( pcWriteBuffer );
        lspc_.TransmitAsync(lspc::MessageTypesToPC::CPUload, (uint8_t *)bufferStart, pcWriteBuffer - bufferStart);
        pcWriteBuffer = bufferStart;

//        // Add extra spacing at the end of the CPU Load package
//        *pcWriteBuffer++ = '\r';
//        *pcWriteBuffer++ = '\n';
//        *pcWriteBuffer = 0;
//        lspc.TransmitAsync(lspc::MessageTypesToPC::CPUload, (uint8_t *)bufferStart, pcWriteBuffer - bufferStart);
//        pcWriteBuffer = bufferStart;

        if (pxTaskStatusArray)
            vPortFree( pxTaskStatusArray );
        //pxTaskStatusArrayPrev = pxTaskStatusArray;
        //uxArraySizePrev = uxArraySize;
        //ulTotalTimeD100Prev = ulTotalTimeD100;
        prevTotalRunTime = ulTotalTime;
    }
}

// Forward declaration
extern "C" {
size_t xPortGetTotalHeapSize(void);
}

void CPULoad::MemoryStats(char * pcWriteBuffer)
{
    char * bufferStart = pcWriteBuffer;
    *pcWriteBuffer = 0x00; // ensure that we start clean

    strcpy( pcWriteBuffer, "Memory\tTotal\tFree\tUtil\r\n" );
    pcWriteBuffer += strlen( pcWriteBuffer );
    lspc_.TransmitAsync(lspc::MessageTypesToPC::CPUload, (uint8_t *)bufferStart, pcWriteBuffer - bufferStart);
    pcWriteBuffer = bufferStart;

    const auto availableHeap = xPortGetFreeHeapSize();
    const auto totalHeap = xPortGetTotalHeapSize();
    const uint32_t heapUtil = ((totalHeap - availableHeap) * 100) / totalHeap;

    sprintf( pcWriteBuffer, "Bytes\t%zu\t%zu\t%lu%%", totalHeap, availableHeap, heapUtil);
    pcWriteBuffer += strlen( pcWriteBuffer );

    // Add extra spacing at the end of the CPU Load package
    *pcWriteBuffer++ = '\r';
    *pcWriteBuffer++ = '\n';
    *pcWriteBuffer++ = '\r';
    *pcWriteBuffer++ = '\n';
    *pcWriteBuffer = 0;
    lspc_.TransmitAsync(lspc::MessageTypesToPC::CPUload, (uint8_t *)bufferStart, pcWriteBuffer - bufferStart);
    pcWriteBuffer = bufferStart;
}