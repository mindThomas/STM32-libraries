#ifndef FREERTOS_DEFINITIONS_H
#define FREERTOS_DEFINITIONS_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#if (tskKERNEL_VERSION_MAJOR != 10 || tskKERNEL_VERSION_MINOR != 2 || tskKERNEL_VERSION_BUILD != 0)
#error "FreeRTOS version changed. Please update FreeRTOS_Definitions.h"
#endif

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

// Copied from FreeRTOS/queue.c since xQueueRegistryItem is kept opaque
// Make sure to keep this updated
typedef struct QueuePointers
{
    int8_t *pcTail;					/*< Points to the byte at the end of the queue storage area.  Once more byte is allocated than necessary to store the queue items, this is used as a marker. */
    int8_t *pcReadFrom;				/*< Points to the last place that a queued item was read from when the structure is used as a queue. */
} QueuePointers_t;

typedef struct SemaphoreData
{
    tskTaskControlBlock *xMutexHolder;		 /*< The handle of the task that holds the mutex. */
    UBaseType_t uxRecursiveCallCount;/*< Maintains a count of the number of times a recursive mutex has been recursively 'taken' when the structure is used as a mutex. */
} SemaphoreData_t;

typedef struct QueueDefinition 		/* The old naming convention is used to prevent breaking kernel aware debuggers. */
{
    int8_t *pcHead;					/*< Points to the beginning of the queue storage area. */
    int8_t *pcWriteTo;				/*< Points to the free next place in the storage area. */

    union
    {
        QueuePointers_t xQueue;		/*< Data required exclusively when this structure is used as a queue. */
        SemaphoreData_t xSemaphore; /*< Data required exclusively when this structure is used as a semaphore. */
    } u;

    List_t xTasksWaitingToSend;		/*< List of tasks that are blocked waiting to post onto this queue.  Stored in priority order. */
    List_t xTasksWaitingToReceive;	/*< List of tasks that are blocked waiting to read from this queue.  Stored in priority order. */

    volatile UBaseType_t uxMessagesWaiting;/*< The number of items currently in the queue. */
    UBaseType_t uxLength;			/*< The length of the queue defined as the number of items it will hold, not the number of bytes. */
    UBaseType_t uxItemSize;			/*< The size of each items that the queue will hold. */

    volatile int8_t cRxLock;		/*< Stores the number of items received from the queue (removed from the queue) while the queue was locked.  Set to queueUNLOCKED when the queue is not locked. */
    volatile int8_t cTxLock;		/*< Stores the number of items transmitted to the queue (added to the queue) while the queue was locked.  Set to queueUNLOCKED when the queue is not locked. */

#if( ( configSUPPORT_STATIC_ALLOCATION == 1 ) && ( configSUPPORT_DYNAMIC_ALLOCATION == 1 ) )
    uint8_t ucStaticallyAllocated;	/*< Set to pdTRUE if the memory used by the queue was statically allocated to ensure no attempt is made to free the memory. */
#endif

#if ( configUSE_QUEUE_SETS == 1 )
    struct QueueDefinition *pxQueueSetContainer;
#endif

#if ( configUSE_TRACE_FACILITY == 1 )
    UBaseType_t uxQueueNumber;
    uint8_t ucQueueType;
#endif

} xQUEUE;

typedef struct QUEUE_REGISTRY_ITEM
{
    const char *pcQueueName; /*lint !e971 Unqualified char types are allowed for strings and single characters only. */
    QueueDefinition *xHandle;
} xQueueRegistryItem;

extern xQueueRegistryItem xQueueRegistry[ configQUEUE_REGISTRY_SIZE ];

#endif // FREERTOS_DEFINITIONS_H
