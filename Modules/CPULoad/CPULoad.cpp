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

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "FreeRTOS_Definitions.h"

#include <stddef.h>

#ifndef FREERTOS_USE_NEWLIB
#error "CPULoad needs to use FreeRTOS::Heap::Newlib, otherwise the printf support does not work."
#endif

#if( configUSE_TRACE_FACILITY != 1 )
#error "configUSE_TRACE_FACILITY must also be set to 1 in FreeRTOSConfig.h to use vTaskGetRunTimeStats().""
#endif

#include <mallocTracker.h>
#include <mallocTracker.hpp>
#include <numeric>

CPULoad::CPULoad(LSPC& lspc, uint32_t cpuLoadTaskPriority)
    : lspc_(lspc)
    , cpuLoadTaskHandle_(0)
    , sleepMs_{1000}
    , printingEnabled_{true}
    , printingBuffer_{(char *)pvPortMalloc(200)}
{
    printingSemaphore_ = xSemaphoreCreateBinary();
    if (printingSemaphore_ == NULL) {
        ERROR("Could not create CPULoad print semaphore");
        return;
    }
    vQueueAddToRegistry(printingSemaphore_, "CPU Load Print");

    if (!printingBuffer_) {
        ERROR("Could not allocate printing buffer for CPU Load");
    }

    xTaskCreate(CPULoad::CPULoadThread, (char*)"CPU Load", CPULOAD_THREAD_STACK, (void*)this, cpuLoadTaskPriority,
                &cpuLoadTaskHandle_);
}

CPULoad::~CPULoad()
{
    if (cpuLoadTaskHandle_)
        vTaskDelete(cpuLoadTaskHandle_); // stop task
}

void CPULoad::SetRefreshRate(uint32_t sleepMs)
{
    sleepMs_ = sleepMs;
}

void CPULoad::EnablePrinting(bool enable)
{
    printingEnabled_ = enable;
}

void CPULoad::CPULoadThread(void * pvParameters)
{
    CPULoad* obj  = (CPULoad*)pvParameters;
    LSPC&    lspc = obj->lspc_;

    /* Send CPU load every second */
    while (1)
    {
        const auto printNow = xSemaphoreTake(obj->printingSemaphore_, obj->sleepMs_*configTICK_RATE_HZ/1000);

        if (!obj->printingBuffer_) {
            obj->printingBuffer_ = (char *)pvPortMalloc(200);
        }

        if (printNow == pdTRUE || obj->printingEnabled_) {
            obj->PrintTaskStats();
            obj->PrintQueueStats();
            obj->PrintMemoryStats();
        } else {
            obj->ComputeTaskStats();
        }

        if (obj->printingEnabled_ == false && obj->printingBuffer_)
        {
            vPortFree(obj->printingBuffer_);
            obj->printingBuffer_ = 0;
        }
    }
}

void CPULoad::Print()
{
    xSemaphoreGive(printingSemaphore_); // print now
}

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

void CPULoad::PrintTaskStats()
{
    // Based on vTaskGetRunTimeStats
    //static TaskStatus_t *pxTaskStatusArrayPrev = 0;
    //static UBaseType_t uxArraySizePrev = 0;
    //static uint32_t ulTotalTimeD100Prev = 0;
    static const auto TICK_FREQUENCY = 1.f/portCONVERT_RUN_TIME_COUNTER_VALUE(1);

    uint32_t ulTotalTime, ulTotalTimeD100, ulStatsAsPercentage, ulOverallStatsAsPercentage;
    TaskStatus_t *pxTaskStatusArray;
    UBaseType_t uxArraySize;

    char * printingBuffer = printingBuffer_;
    *printingBuffer = 0x00; // ensure that we start clean

    uxArraySize = uxTaskGetNumberOfTasks();
    pxTaskStatusArray = (TaskStatus_t*)pvPortMalloc( uxArraySize * sizeof( TaskStatus_t ) );

    unsigned long numTasksPrinted = 0;
    unsigned long tasksTotalMemoryUsageBytes = 0;

    if (pxTaskStatusArray != NULL)
    {
        uxArraySize = uxTaskGetSystemState( pxTaskStatusArray, uxArraySize, &ulTotalTime );

        ulTotalTimeD100 = ulTotalTime / 100UL; // For percentage calculations.

        if( ulTotalTimeD100 > 0 )
        {
            strcpy(printingBuffer, "Prio\tTask name");
            printingBuffer += strlen(printingBuffer);
            for (UBaseType_t x = 0; x < (size_t) (configMAX_TASK_NAME_LEN - 10); x++) {
                *printingBuffer++ = ' ';
            }
#ifdef CPULOAD_USE_VERBOSE
            strcpy(printingBuffer, "\tTotal     \tUtil\tOverall\tState    \tMem usage\tStack alloc\tStack left\r\n");
#else
            strcpy( printingBuffer, "\tUtil\tOverall\tState    \tMem usage\tStack alloc\tStack left\r\n" );
#endif
            printingBuffer += strlen(printingBuffer);
            lspc_.TransmitAsync(lspc::MessageTypesToPC::CPUload, (uint8_t *) printingBuffer_,
                                printingBuffer - printingBuffer_);
            printingBuffer = printingBuffer_;

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

                sprintf( printingBuffer, "%u\t", ( unsigned int ) pxTaskStatusArray[taskIdx].uxCurrentPriority );
                printingBuffer += strlen( printingBuffer );

                printingBuffer = WriteTaskNameToBuffer( printingBuffer, pxTaskStatusArray[taskIdx].pcTaskName );

#ifdef CPULOAD_USE_VERBOSE
                sprintf(printingBuffer, "\t%-10lu", pxTaskStatusArray[taskIdx].ulRunTimeCounter);
                printingBuffer += strlen( printingBuffer );
#endif

                if( ulStatsAsPercentage > 0UL )
                {
                    //sprintf( printingBuffer, "\t%u\t\t%u%%\t", ( unsigned int ) pxTaskStatusArray[taskIdx].ulRunTimeCounter, ( unsigned int ) ulStatsAsPercentage );
                    sprintf( printingBuffer, "\t%u%%", ( unsigned int ) ulStatsAsPercentage );
                }
                else
                {
                    //sprintf( printingBuffer, "\t%u\t\t<1%%\t", ( unsigned int ) pxTaskStatusArray[taskIdx].ulRunTimeCounter );
                    strcpy( printingBuffer, "\t<1%" );
                }
                printingBuffer += strlen( printingBuffer );

                if( ulOverallStatsAsPercentage > 0UL )
                {
                    //sprintf( printingBuffer, "\t%u\t\t%u%%\t", ( unsigned int ) pxTaskStatusArray[taskIdx].ulRunTimeCounter, ( unsigned int ) ulOverallStatsAsPercentage );
                    sprintf( printingBuffer, "\t%u%%\t", ( unsigned int ) ulOverallStatsAsPercentage );
                }
                else
                {
                    //sprintf( printingBuffer, "\t%u\t\t<1%%\t", ( unsigned int ) pxTaskStatusArray[taskIdx].ulRunTimeCounter );
                    strcpy( printingBuffer, "\t<1%\t" );
                }
                printingBuffer += strlen( printingBuffer );

                switch (pxTaskStatusArray[taskIdx].eCurrentState) {
                    case eRunning:
                        strcpy(printingBuffer, "[RUNNING]");
                        break;
                    case eReady:
                        strcpy(printingBuffer, "[READY]  ");
                        break;
                    case eBlocked:
                        strcpy(printingBuffer, "[BLOCKED]");
                        break;
                    case eSuspended:
                        strcpy(printingBuffer, "[SUSPENDED]");
                        break;
                    case eDeleted:
                        strcpy(printingBuffer, "[DELETED]");
                        break;
                    case eInvalid:
                        strcpy(printingBuffer, "[INVALID]");
                        break;
                    default:
                        strcpy(printingBuffer, "[UNKNOWN]");
                        break;
                }
                printingBuffer += strlen( printingBuffer );

                // Words = 4 bytes
                const unsigned int taskStackSizeWords = (unsigned int)(pxTaskStatusArray[taskIdx].xHandle->pxEndOfStack - pxTaskStatusArray[taskIdx].xHandle->pxStack + 2);
                tasksTotalMemoryUsageBytes += 4*taskStackSizeWords + sizeof(tskTCB);
                sprintf( printingBuffer, "\t%u bytes", 4*taskStackSizeWords + sizeof(tskTCB));
                printingBuffer += strlen( printingBuffer );

                // Words = 4 bytes
                sprintf( printingBuffer, "\t%u words", taskStackSizeWords);
                printingBuffer += strlen( printingBuffer );

                sprintf( printingBuffer, "\t%u words\t", pxTaskStatusArray[taskIdx].usStackHighWaterMark);
                printingBuffer += strlen( printingBuffer );

                *printingBuffer++ = '\r';
                *printingBuffer++ = '\n';
                *printingBuffer = 0;

                lspc_.TransmitAsync(lspc::MessageTypesToPC::CPUload, (uint8_t *)printingBuffer_, printingBuffer - printingBuffer_);
                printingBuffer = printingBuffer_;

                // Mark task as printed
                pxTaskStatusArray[taskIdx].pxStackBase = 0;
                prevTaskRunTime[pxTaskStatusArray[taskIdx].xHandle] = pxTaskStatusArray[taskIdx].ulRunTimeCounter;
                numTasksPrinted++;
            }
        }

        strcpy(printingBuffer, "Total memory usage:");
        printingBuffer += strlen(printingBuffer);
        for (UBaseType_t x = 0; x < (size_t)configMAX_TASK_NAME_LEN - 10; x++) {
            *printingBuffer++ = ' ';
        }
        sprintf(printingBuffer, "\t\t\t\t\t%lu bytes", tasksTotalMemoryUsageBytes);
        printingBuffer += strlen(printingBuffer);
        lspc_.TransmitAsync(lspc::MessageTypesToPC::CPUload, (uint8_t *) printingBuffer_, printingBuffer - printingBuffer_);
        printingBuffer = printingBuffer_;

#ifdef CPULOAD_USE_VERBOSE
        sprintf(printingBuffer, "\r\nTotal Ticks @ %.2f Hz: %lu", TICK_FREQUENCY, ulTotalTime);
        printingBuffer += strlen(printingBuffer);
        lspc_.TransmitAsync(lspc::MessageTypesToPC::CPUload, (uint8_t *) printingBuffer_, printingBuffer - printingBuffer_);
        printingBuffer = printingBuffer_;
#endif

        const auto totalRunTime = portCONVERT_RUN_TIME_COUNTER_VALUE(ulTotalTime);

        sprintf(printingBuffer, "\r\nTotal Time: %.5f s\r\n\r\n", totalRunTime);
        printingBuffer += strlen(printingBuffer);
        lspc_.TransmitAsync(lspc::MessageTypesToPC::CPUload, (uint8_t *) printingBuffer_, printingBuffer - printingBuffer_);
        printingBuffer = printingBuffer_;

        prevTotalRunTime = ulTotalTime;
    }

    if (pxTaskStatusArray) {
        vPortFree(pxTaskStatusArray);
    }
}


void CPULoad::ComputeTaskStats()
{
    // Based on vTaskGetRunTimeStats
    uint32_t ulTotalTime;
    TaskStatus_t *pxTaskStatusArray;
    UBaseType_t uxArraySize;

    uxArraySize = uxTaskGetNumberOfTasks();
    pxTaskStatusArray = (TaskStatus_t*)pvPortMalloc( uxArraySize * sizeof( TaskStatus_t ) );

    unsigned long numTasksPrinted = 0;

    if( pxTaskStatusArray != NULL )
    {
        uxArraySize = uxTaskGetSystemState( pxTaskStatusArray, uxArraySize, &ulTotalTime );

        if( ulTotalTime > 0 )
        {
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

                // Mark task as printed
                pxTaskStatusArray[taskIdx].pxStackBase = 0;
                prevTaskRunTime[pxTaskStatusArray[taskIdx].xHandle] = pxTaskStatusArray[taskIdx].ulRunTimeCounter;
                numTasksPrinted++;
            }
        }

        vPortFree(pxTaskStatusArray);
        prevTotalRunTime = ulTotalTime;
    }
}


// Forward declaration
#ifdef FREERTOS_USE_NEWLIB
extern "C" {
    size_t xPortGetTotalHeapSize(void);
}
#endif

void CPULoad::PrintMemoryStats()
{
    char * printingBuffer = printingBuffer_;
    *printingBuffer = 0x00; // ensure that we start clean

    strcpy( printingBuffer, "Memory\tTotal\tUsed\tFree\tUtil\r\n" );
    printingBuffer += strlen( printingBuffer );
    lspc_.TransmitAsync(lspc::MessageTypesToPC::CPUload, (uint8_t *)printingBuffer_, printingBuffer - printingBuffer_);
    printingBuffer = printingBuffer_;

    const uint32_t availableHeap = xPortGetFreeHeapSize();
#ifdef FREERTOS_USE_NEWLIB
    const uint32_t totalHeap = xPortGetTotalHeapSize();
#else
    const uint32_t totalHeap = configTOTAL_HEAP_SIZE;
#endif
    const uint32_t heapUsage = (totalHeap - availableHeap);
    const uint32_t heapUtil = (heapUsage * 100) / totalHeap;

    sprintf( printingBuffer, "Bytes\t%lu\t%lu\t%lu\t%lu%%\r\n", totalHeap, heapUsage, availableHeap, heapUtil);
    printingBuffer += strlen( printingBuffer );

    // Get memory allocation map
    const std::array<allocatedMemory_t, NUM_SLOTS>& allocatedMemory = getAllocatedMemorySlots();
    const auto addMemory = [](size_t a, allocatedMemory_t b) {
        return a + b.size;
    };
    const volatile auto totalMemoryUsage = std::accumulate(allocatedMemory.begin(), allocatedMemory.end(),
                    size_t{0},
                    addMemory);

#ifdef CPULOAD_USE_VERBOSE
    strcpy( printingBuffer, "Size\tDescription\r\n" );
    printingBuffer += strlen( printingBuffer );
    lspc_.TransmitAsync(lspc::MessageTypesToPC::CPUload, (uint8_t *)printingBuffer_, printingBuffer - printingBuffer_);
    printingBuffer = printingBuffer_;

    for (size_t i = 0; i < allocatedMemory.size(); ++i) {
        if (allocatedMemory[i].ptr) {
            if (allocatedMemory[i].assignCaller) {
                sprintf(printingBuffer, "%lu\t%s:%lu\r\n", allocatedMemory[i].size, allocatedMemory[i].assignCaller, allocatedMemory[i].line);
            } else {
                sprintf(printingBuffer, "%lu\r\n", allocatedMemory[i].size);
            }
            printingBuffer += strlen(printingBuffer);
            while (!lspc_.TransmitAsync(lspc::MessageTypesToPC::CPUload, (uint8_t *) printingBuffer_,
                                printingBuffer - printingBuffer_)) {
                osDelay(100); // send some of the messages
            }
            printingBuffer = printingBuffer_;
        }
    }
#endif

    // Add extra spacing at the end of the CPU Load package
    *printingBuffer++ = '\r';
    *printingBuffer++ = '\n';
    *printingBuffer++ = '\r';
    *printingBuffer++ = '\n';
    *printingBuffer = 0;
    lspc_.TransmitAsync(lspc::MessageTypesToPC::CPUload, (uint8_t *)printingBuffer_, printingBuffer - printingBuffer_);
}

void CPULoad::PrintQueueStats()
{
    // Iterate through: xQueueRegistry
    // Make a printout similar to: https://mcuoneclipse.files.wordpress.com/2017/03/queues-with-names.png
    // which comes from: https://mcuoneclipse.com/2017/03/18/better-freertos-debugging-in-eclipse/
    char * printingBuffer = printingBuffer_;
    *printingBuffer = 0x00; // ensure that we start clean

    unsigned long queuesTotalMemoryUsageBytes = 0;

    // Determine maximum queue name length
    unsigned int maximumQueueNameLength = 0;
    for (unsigned int i = 0; i < configQUEUE_REGISTRY_SIZE; ++i) {
        if (xQueueRegistry[i].xHandle != NULL) {
            maximumQueueNameLength = std::max(maximumQueueNameLength, strlen(xQueueRegistry[i].pcQueueName));
        }
    }
    maximumQueueNameLength++; // add 1 space as spacing

    strcpy( printingBuffer, "Index\tQueue name" );
    printingBuffer += strlen( printingBuffer );
    for( UBaseType_t x = 0; x < ( size_t ) ( maximumQueueNameLength - 10 ); x++ )
    {
        *printingBuffer++ = ' ';
    }
#ifdef CPULOAD_USE_VERBOSE
    strcpy( printingBuffer, "Address   " );
    printingBuffer += strlen( printingBuffer );
#endif
    strcpy( printingBuffer, "Length\tSize\tTotal\t# TX Wait" );
    printingBuffer += strlen( printingBuffer );
    for( UBaseType_t x = 0; x < ( size_t ) ( maximumQueueNameLength + 4 - 9 ); x++ )
    {
        *printingBuffer++ = ' ';
    }
    strcpy( printingBuffer, "# RX Wait" );
    printingBuffer += strlen( printingBuffer );
    for( UBaseType_t x = 0; x < ( size_t ) ( maximumQueueNameLength + 4 - 9 ); x++ )
    {
        *printingBuffer++ = ' ';
    }
    strcpy( printingBuffer, "Queue Type\r\n" );
    printingBuffer += strlen( printingBuffer );
    lspc_.TransmitAsync(lspc::MessageTypesToPC::CPUload, (uint8_t *)printingBuffer_, printingBuffer - printingBuffer_);
    printingBuffer = printingBuffer_;

    for (unsigned int i = 0; i < configQUEUE_REGISTRY_SIZE; ++i) {
        if (xQueueRegistry[i].xHandle != NULL) {
            sprintf( printingBuffer, "%u\t", i);
            printingBuffer += strlen( printingBuffer );

            strcpy( printingBuffer, xQueueRegistry[i].pcQueueName );
            printingBuffer += strlen( printingBuffer );
            for( char * x = printingBuffer; x < printingBuffer_ + maximumQueueNameLength; ++x )
            {
                *printingBuffer++ = ' ';
            }

#ifdef CPULOAD_USE_VERBOSE
            sprintf( printingBuffer, "\t0x%08x", (uint32_t)xQueueRegistry[i].xHandle);
            printingBuffer += strlen( printingBuffer );
#endif

            sprintf( printingBuffer, "\t%lu/%lu", xQueueRegistry[i].xHandle->uxMessagesWaiting, xQueueRegistry[i].xHandle->uxLength);
            printingBuffer += strlen( printingBuffer );

            sprintf( printingBuffer, "\t%lu", xQueueRegistry[i].xHandle->uxItemSize);
            printingBuffer += strlen( printingBuffer );

            queuesTotalMemoryUsageBytes +=  xQueueRegistry[i].xHandle->uxLength*xQueueRegistry[i].xHandle->uxItemSize + sizeof(xQUEUE);
            sprintf( printingBuffer, "\t%lu\t", xQueueRegistry[i].xHandle->uxLength*xQueueRegistry[i].xHandle->uxItemSize + sizeof(xQUEUE));
            printingBuffer += strlen( printingBuffer );

            // Print tasks waiting for queue/semaphore (if any)
            sprintf( printingBuffer, "%lu", xQueueRegistry[i].xHandle->xTasksWaitingToSend.uxNumberOfItems);
            if (xQueueRegistry[i].xHandle->xTasksWaitingToSend.uxNumberOfItems > 0) {
                if (xQueueRegistry[i].xHandle->xTasksWaitingToSend.xListEnd.pxNext->pvOwner != nullptr) {
                    tskTaskControlBlock* task = (tskTaskControlBlock*)xQueueRegistry[i].xHandle->xTasksWaitingToSend.xListEnd.pxNext->pvOwner;
                    strcpy(printingBuffer + strlen(printingBuffer), " [");
                    strcpy(printingBuffer + strlen(printingBuffer), task->pcTaskName);
                    strcpy(printingBuffer + strlen(printingBuffer), "]");
                }
            }
            size_t totalLen = strlen(printingBuffer);
            printingBuffer += strlen( printingBuffer );
            for( UBaseType_t x = 0; x < ( size_t ) ( maximumQueueNameLength + 4 - totalLen ); x++ )
            {
                *printingBuffer++ = ' ';
            }

            sprintf( printingBuffer, "%lu", xQueueRegistry[i].xHandle->xTasksWaitingToReceive.uxNumberOfItems);
            if (xQueueRegistry[i].xHandle->xTasksWaitingToReceive.uxNumberOfItems > 0) {
                if (xQueueRegistry[i].xHandle->xTasksWaitingToReceive.xListEnd.pxNext->pvOwner != nullptr) {
                    tskTaskControlBlock* task = (tskTaskControlBlock*)xQueueRegistry[i].xHandle->xTasksWaitingToReceive.xListEnd.pxNext->pvOwner;
                    strcpy(printingBuffer + strlen(printingBuffer), " [");
                    strcpy(printingBuffer + strlen(printingBuffer), task->pcTaskName);
                    strcpy(printingBuffer + strlen(printingBuffer), "]");
                }
            }
            totalLen = strlen(printingBuffer);
            printingBuffer += strlen( printingBuffer );
            for( UBaseType_t x = 0; x < ( size_t ) ( maximumQueueNameLength + 4 - totalLen ); x++ )
            {
                *printingBuffer++ = ' ';
            }

            // Print Queue type
            switch (xQueueRegistry[i].xHandle->ucQueueType) {
                default:
                    if (xQueueRegistry[i].xHandle->uxItemSize == 0) {
                        if (xQueueRegistry[i].xHandle->pcHead == NULL) {
                            strcpy(printingBuffer, "Mutex");
                        } else {
                            strcpy(printingBuffer, "Binary Semaphore");
                        }
                    } else {
                        strcpy(printingBuffer, "Queue");
                    }
                    break;
                case 1:
                    strcpy(printingBuffer, "Mutex");
                    break;
                case 2:
                    strcpy(printingBuffer, "Counting Semaphore");
                    break;
                case 3:
                    strcpy(printingBuffer, "Binary Semaphore");
                    break;
                case 4:
                    strcpy(printingBuffer, "Recursive Mutex");
                    break;
            }
            printingBuffer += strlen( printingBuffer );

            *printingBuffer++ = '\r';
            *printingBuffer++ = '\n';
            *printingBuffer = 0;
            lspc_.TransmitAsync(lspc::MessageTypesToPC::CPUload, (uint8_t *)printingBuffer_, printingBuffer - printingBuffer_);
            printingBuffer = printingBuffer_;
        }
    }

    strcpy(printingBuffer, "Total memory usage:");
    printingBuffer += strlen(printingBuffer);
    for (UBaseType_t x = 0; x < (size_t)configMAX_TASK_NAME_LEN - 10; x++) {
        *printingBuffer++ = ' ';
    }
    sprintf(printingBuffer, "\t\t\t\t%lu bytes", queuesTotalMemoryUsageBytes);
    printingBuffer += strlen(printingBuffer);
    *printingBuffer++ = '\r';
    *printingBuffer++ = '\n';

    // Add extra spacing at the end of the Queue list
    *printingBuffer++ = '\r';
    *printingBuffer++ = '\n';
    *printingBuffer = 0;
    lspc_.TransmitAsync(lspc::MessageTypesToPC::CPUload, (uint8_t *)printingBuffer_, printingBuffer - printingBuffer_);
}