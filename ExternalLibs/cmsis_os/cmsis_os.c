/* ----------------------------------------------------------------------
 * $Date:        5. February 2013
 * $Revision:    V1.02
 *
 * Project:      CMSIS-RTOS API
 * Title:        cmsis_os.c
 *
 * Version 0.02
 *    Initial Proposal Phase
 * Version 0.03
 *    osKernelStart added, optional feature: main started as thread
 *    osSemaphores have standard behavior
 *    osTimerCreate does not start the timer, added osTimerStart
 *    osThreadPass is renamed to osThreadYield
 * Version 1.01
 *    Support for C++ interface
 *     - const attribute removed from the osXxxxDef_t typedef's
 *     - const attribute added to the osXxxxDef macros
 *    Added: osTimerDelete, osMutexDelete, osSemaphoreDelete
 *    Added: osKernelInitialize
 * Version 1.02
 *    Control functions for short timeouts in microsecond resolution:
 *    Added: osKernelSysTick, osKernelSysTickFrequency, osKernelSysTickMicroSec
 *    Removed: osSignalGet
 *
 *
 *----------------------------------------------------------------------------
 *
 * Portions Copyright © 2016 STMicroelectronics International N.V. All rights reserved.
 * Portions Copyright (c) 2013 ARM LIMITED
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  - Neither the name of ARM  nor the names of its contributors may be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *---------------------------------------------------------------------------*/

 /**
  ******************************************************************************
  * @file    cmsis_os.c
  * @author  MCD Application Team
  * @date    13-July-2017
  * @brief   CMSIS-RTOS API implementation for FreeRTOS V9.0.0
  ******************************************************************************
  * @attention
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

#include <string.h>
#include "cmsis_os.h"

  /*-----------------------------------------------------------*/
#pragma region MyRegion
#define THIS_REGION_ON 
#ifdef THIS_REGION_ON
//from tasks.c
/* Values that can be assigned to the ucNotifyState member of the TCB. */
#define taskNOT_WAITING_NOTIFICATION	( ( uint8_t ) 0 )
#define taskWAITING_NOTIFICATION		( ( uint8_t ) 1 )
#define taskNOTIFICATION_RECEIVED		( ( uint8_t ) 2 )

/*
 * The value used to fill the stack of a task when the task is created.  This
 * is used purely for checking the high water mark for tasks.
 */
#define tskSTACK_FILL_BYTE	( 0xa5U )

 /*
  * Task control block.  A task control block (TCB) is allocated for each task,
  * and stores task state information, including a pointer to the task's context
  * (the task's run time environment, including register values)
  */
typedef struct tskTaskControlBlock
{
	volatile StackType_t* pxTopOfStack;	/*< Points to the location of the last item placed on the tasks stack.  THIS MUST BE THE FIRST MEMBER OF THE TCB STRUCT. */

#if ( portUSING_MPU_WRAPPERS == 1 )
	xMPU_SETTINGS	xMPUSettings;		/*< The MPU settings are defined as part of the port layer.  THIS MUST BE THE SECOND MEMBER OF THE TCB STRUCT. */
#endif

	ListItem_t			xStateListItem;	/*< The list that the state list item of a task is reference from denotes the state of that task (Ready, Blocked, Suspended ). */
	ListItem_t			xEventListItem;		/*< Used to reference a task from an event list. */
	UBaseType_t			uxPriority;			/*< The priority of the task.  0 is the lowest priority. */
	StackType_t* pxStack;			/*< Points to the start of the stack. */
	char				pcTaskName[configMAX_TASK_NAME_LEN];/*< Descriptive name given to the task when created.  Facilitates debugging only. */ /*lint !e971 Unqualified char types are allowed for strings and single characters only. */

#if ( ( portSTACK_GROWTH > 0 ) || ( configRECORD_STACK_HIGH_ADDRESS == 1 ) )
	StackType_t* pxEndOfStack;		/*< Points to the highest valid address for the stack. */
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
	void* pvThreadLocalStoragePointers[configNUM_THREAD_LOCAL_STORAGE_POINTERS];
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

	/* See the comments above the definition of
	tskSTATIC_AND_DYNAMIC_ALLOCATION_POSSIBLE. */
#if( tskSTATIC_AND_DYNAMIC_ALLOCATION_POSSIBLE != 0 ) /*lint !e731 Macro has been consolidated for readability reasons. */
	uint8_t	ucStaticallyAllocated; 		/*< Set to pdTRUE if the task is a statically allocated to ensure no attempt is made to free the memory. */
#endif

#if( INCLUDE_xTaskAbortDelay == 1 )
	uint8_t ucDelayAborted;
#endif

} tskTCB;

/* The old tskTCB name is maintained above then typedefed to the new TCB_t name
below to enable the use of older kernel aware debuggers. */
typedef tskTCB TCB_t;

 /*lint -save -e956 A manual analysis and inspection has been used to determine
 which static variables must be declared volatile. */

//- PRIVILEGED_DATA TCB_t* volatile pxCurrentTCB = NULL;

/* Lists for ready and blocked tasks. --------------------*/
PRIVILEGED_DATA static List_t pxReadyTasksLists[configMAX_PRIORITIES];/*< Prioritised ready tasks. */
PRIVILEGED_DATA static List_t xDelayedTaskList1;						/*< Delayed tasks. */
PRIVILEGED_DATA static List_t xDelayedTaskList2;						/*< Delayed tasks (two lists are used - one for delays that have overflowed the current tick count. */
PRIVILEGED_DATA static List_t* volatile pxDelayedTaskList;				/*< Points to the delayed task list currently being used. */
PRIVILEGED_DATA static List_t* volatile pxOverflowDelayedTaskList;		/*< Points to the delayed task list currently being used to hold tasks that have overflowed the current tick count. */
PRIVILEGED_DATA static List_t xPendingReadyList;						/*< Tasks that have been readied while the scheduler was suspended.  They will be moved to the ready list when the scheduler is resumed. */

#if( INCLUDE_vTaskDelete == 1 )

PRIVILEGED_DATA static List_t xTasksWaitingTermination;				/*< Tasks that have been deleted - but their memory not yet freed. */
PRIVILEGED_DATA static volatile UBaseType_t uxDeletedTasksWaitingCleanUp = (UBaseType_t)0U;

#endif

#if ( INCLUDE_vTaskSuspend == 1 )

PRIVILEGED_DATA static List_t xSuspendedTaskList;					/*< Tasks that are currently suspended. */

#endif

/* Other file private variables. --------------------------------*/
PRIVILEGED_DATA static volatile UBaseType_t uxCurrentNumberOfTasks = (UBaseType_t)0U;
PRIVILEGED_DATA static volatile TickType_t xTickCount = (TickType_t)configINITIAL_TICK_COUNT;
PRIVILEGED_DATA static volatile UBaseType_t uxTopReadyPriority = tskIDLE_PRIORITY;
PRIVILEGED_DATA static volatile BaseType_t xSchedulerRunning = pdFALSE;
PRIVILEGED_DATA static volatile UBaseType_t uxPendedTicks = (UBaseType_t)0U;
PRIVILEGED_DATA static volatile BaseType_t xYieldPending = pdFALSE;
PRIVILEGED_DATA static volatile BaseType_t xNumOfOverflows = (BaseType_t)0;
PRIVILEGED_DATA static UBaseType_t uxTaskNumber = (UBaseType_t)0U;
PRIVILEGED_DATA static volatile TickType_t xNextTaskUnblockTime = (TickType_t)0U; /* Initialised to portMAX_DELAY before the scheduler starts. */
PRIVILEGED_DATA static TaskHandle_t xIdleTaskHandle = NULL;			/*< Holds the handle of the idle task.  The idle task is created automatically when the scheduler is started. */


#if( configSUPPORT_STATIC_ALLOCATION == 1 )


static void prvInitialiseNewTask(TaskFunction_t pxTaskCode,
	const char* const pcName,		/*lint !e971 Unqualified char types are allowed for strings and single characters only. */
	const uint32_t ulStackDepth,
	void* const pvParameters,
	UBaseType_t uxPriority,
	TaskHandle_t* const pxCreatedTask,
	TCB_t* pxNewTCB,
	const MemoryRegion_t* const xRegions)
{
	StackType_t* pxTopOfStack;
	UBaseType_t x;

#if( portUSING_MPU_WRAPPERS == 1 )
	/* Should the task be created in privileged mode? */
	BaseType_t xRunPrivileged;
	if ((uxPriority & portPRIVILEGE_BIT) != 0U)
	{
		xRunPrivileged = pdTRUE;
	}
	else
	{
		xRunPrivileged = pdFALSE;
	}
	uxPriority &= ~portPRIVILEGE_BIT;
#endif /* portUSING_MPU_WRAPPERS == 1 */

	/* Avoid dependency on memset() if it is not required. */
#if( tskSET_NEW_STACKS_TO_KNOWN_VALUE == 1 )
	{
		/* Fill the stack with a known value to assist debugging. */
		(void)memset(pxNewTCB->pxStack, (int)tskSTACK_FILL_BYTE, (size_t)ulStackDepth * sizeof(StackType_t));
	}
#endif /* tskSET_NEW_STACKS_TO_KNOWN_VALUE */

	/* Calculate the top of stack address.  This depends on whether the stack
	grows from high memory to low (as per the 80x86) or vice versa.
	portSTACK_GROWTH is used to make the result positive or negative as required
	by the port. */
#if( portSTACK_GROWTH < 0 )
	{
		pxTopOfStack = pxNewTCB->pxStack + (ulStackDepth - (uint32_t)1);
		pxTopOfStack = (StackType_t*)(((portPOINTER_SIZE_TYPE)pxTopOfStack) & (~((portPOINTER_SIZE_TYPE)portBYTE_ALIGNMENT_MASK))); /*lint !e923 MISRA exception.  Avoiding casts between pointers and integers is not practical.  Size differences accounted for using portPOINTER_SIZE_TYPE type. */

		/* Check the alignment of the calculated top of stack is correct. */
		configASSERT((((portPOINTER_SIZE_TYPE)pxTopOfStack & (portPOINTER_SIZE_TYPE)portBYTE_ALIGNMENT_MASK) == 0UL));

#if( configRECORD_STACK_HIGH_ADDRESS == 1 )
		{
			/* Also record the stack's high address, which may assist
			debugging. */
			pxNewTCB->pxEndOfStack = pxTopOfStack;
		}
#endif /* configRECORD_STACK_HIGH_ADDRESS */
	}
#else /* portSTACK_GROWTH */
	{
		pxTopOfStack = pxNewTCB->pxStack;

		/* Check the alignment of the stack buffer is correct. */
		configASSERT((((portPOINTER_SIZE_TYPE)pxNewTCB->pxStack & (portPOINTER_SIZE_TYPE)portBYTE_ALIGNMENT_MASK) == 0UL));

		/* The other extreme of the stack space is required if stack checking is
		performed. */
		pxNewTCB->pxEndOfStack = pxNewTCB->pxStack + (ulStackDepth - (uint32_t)1);
	}
#endif /* portSTACK_GROWTH */

	/* Store the task name in the TCB. */
	for (x = (UBaseType_t)0; x < (UBaseType_t)configMAX_TASK_NAME_LEN; x++)
	{
		pxNewTCB->pcTaskName[x] = pcName[x];

		/* Don't copy all configMAX_TASK_NAME_LEN if the string is shorter than
		configMAX_TASK_NAME_LEN characters just in case the memory after the
		string is not accessible (extremely unlikely). */
		if (pcName[x] == 0x00)
		{
			break;
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}
	}

	/* Ensure the name string is terminated in the case that the string length
	was greater or equal to configMAX_TASK_NAME_LEN. */
	pxNewTCB->pcTaskName[configMAX_TASK_NAME_LEN - 1] = '\0';

	/* This is used as an array index so must ensure it's not too large.  First
	remove the privilege bit if one is present. */
	if (uxPriority >= (UBaseType_t)configMAX_PRIORITIES)
	{
		uxPriority = (UBaseType_t)configMAX_PRIORITIES - (UBaseType_t)1U;
	}
	else
	{
		mtCOVERAGE_TEST_MARKER();
	}

	pxNewTCB->uxPriority = uxPriority;
#if ( configUSE_MUTEXES == 1 )
	{
		pxNewTCB->uxBasePriority = uxPriority;
		pxNewTCB->uxMutexesHeld = 0;
	}
#endif /* configUSE_MUTEXES */

	vListInitialiseItem(&(pxNewTCB->xStateListItem));
	vListInitialiseItem(&(pxNewTCB->xEventListItem));

	/* Set the pxNewTCB as a link back from the ListItem_t.  This is so we can get
	back to	the containing TCB from a generic item in a list. */
	listSET_LIST_ITEM_OWNER(&(pxNewTCB->xStateListItem), pxNewTCB);

	/* Event lists are always in priority order. */
	listSET_LIST_ITEM_VALUE(&(pxNewTCB->xEventListItem), (TickType_t)configMAX_PRIORITIES - (TickType_t)uxPriority); /*lint !e961 MISRA exception as the casts are only redundant for some ports. */
	listSET_LIST_ITEM_OWNER(&(pxNewTCB->xEventListItem), pxNewTCB);

#if ( portCRITICAL_NESTING_IN_TCB == 1 )
	{
		pxNewTCB->uxCriticalNesting = (UBaseType_t)0U;
	}
#endif /* portCRITICAL_NESTING_IN_TCB */

#if ( configUSE_APPLICATION_TASK_TAG == 1 )
	{
		pxNewTCB->pxTaskTag = NULL;
	}
#endif /* configUSE_APPLICATION_TASK_TAG */

#if ( configGENERATE_RUN_TIME_STATS == 1 )
	{
		pxNewTCB->ulRunTimeCounter = 0UL;
	}
#endif /* configGENERATE_RUN_TIME_STATS */

#if ( portUSING_MPU_WRAPPERS == 1 )
	{
		vPortStoreTaskMPUSettings(&(pxNewTCB->xMPUSettings), xRegions, pxNewTCB->pxStack, ulStackDepth);
	}
#else
	{
		/* Avoid compiler warning about unreferenced parameter. */
		(void)xRegions;
	}
#endif

#if( configNUM_THREAD_LOCAL_STORAGE_POINTERS != 0 )
	{
		for (x = 0; x < (UBaseType_t)configNUM_THREAD_LOCAL_STORAGE_POINTERS; x++)
		{
			pxNewTCB->pvThreadLocalStoragePointers[x] = NULL;
		}
	}
#endif

#if ( configUSE_TASK_NOTIFICATIONS == 1 )
	{
		pxNewTCB->ulNotifiedValue = 0;
		pxNewTCB->ucNotifyState = taskNOT_WAITING_NOTIFICATION;
	}
#endif

#if ( configUSE_NEWLIB_REENTRANT == 1 )
	{
		/* Initialise this task's Newlib reent structure. */
		_REENT_INIT_PTR((&(pxNewTCB->xNewLib_reent)));
	}
#endif

#if( INCLUDE_xTaskAbortDelay == 1 )
	{
		pxNewTCB->ucDelayAborted = pdFALSE;
	}
#endif

	/* Initialize the TCB stack to look as if the task was already running,
	but had been interrupted by the scheduler.  The return address is set
	to the start of the task function. Once the stack has been initialised
	the top of stack variable is updated. */
#if( portUSING_MPU_WRAPPERS == 1 )
	{
		pxNewTCB->pxTopOfStack = pxPortInitialiseStack(pxTopOfStack, pxTaskCode, pvParameters, xRunPrivileged);
	}
#else /* portUSING_MPU_WRAPPERS */
	{
		pxNewTCB->pxTopOfStack = pxPortInitialiseStack(pxTopOfStack, pxTaskCode, pvParameters);
	}
#endif /* portUSING_MPU_WRAPPERS */

	if ((void*)pxCreatedTask != NULL)
	{
		/* Pass the handle out in an anonymous way.  The handle can be used to
		change the created task's priority, delete the created task, etc.*/
		*pxCreatedTask = (TaskHandle_t)pxNewTCB;
	}
	else
	{
		mtCOVERAGE_TEST_MARKER();
	}
}
/*-----------------------------------------------------------*/

static void prvAddNewTaskToReadyList(TCB_t* pxNewTCB)
{
	/* Ensure interrupts don't access the task lists while the lists are being
	updated. */
	taskENTER_CRITICAL();
	{
		uxCurrentNumberOfTasks++;
		if (pxCurrentTCB == NULL)
		{
			/* There are no other tasks, or all the other tasks are in
			the suspended state - make this the current task. */
			pxCurrentTCB = pxNewTCB;

			if (uxCurrentNumberOfTasks == (UBaseType_t)1)
			{
				/* This is the first task to be created so do the preliminary
				initialisation required.  We will not recover if this call
				fails, but we will report the failure. */
				prvInitialiseTaskLists();
			}
			else
			{
				mtCOVERAGE_TEST_MARKER();
			}
		}
		else
		{
			/* If the scheduler is not already running, make this task the
			current task if it is the highest priority task to be created
			so far. */
			if (xSchedulerRunning == pdFALSE)
			{
				if (pxCurrentTCB->uxPriority <= pxNewTCB->uxPriority)
				{
					pxCurrentTCB = pxNewTCB;
				}
				else
				{
					mtCOVERAGE_TEST_MARKER();
				}
			}
			else
			{
				mtCOVERAGE_TEST_MARKER();
			}
		}

		uxTaskNumber++;

#if ( configUSE_TRACE_FACILITY == 1 )
		{
			/* Add a counter into the TCB for tracing only. */
			pxNewTCB->uxTCBNumber = uxTaskNumber;
		}
#endif /* configUSE_TRACE_FACILITY */
		traceTASK_CREATE(pxNewTCB);

		prvAddTaskToReadyList(pxNewTCB);

		portSETUP_TCB(pxNewTCB);
	}
	taskEXIT_CRITICAL();

	if (xSchedulerRunning != pdFALSE)
	{
		/* If the created task is of a higher priority than the current task
		then it should run now. */
		if (pxCurrentTCB->uxPriority < pxNewTCB->uxPriority)
		{
			taskYIELD_IF_USING_PREEMPTION();
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}
	}
	else
	{
		mtCOVERAGE_TEST_MARKER();
	}
}
/*-----------------------------------------------------------*/

TaskHandle_t xTaskCreateStatic(TaskFunction_t pxTaskCode,
	const char* const pcName,		/*lint !e971 Unqualified char types are allowed for strings and single characters only. */
	const uint32_t ulStackDepth,
	void* const pvParameters,
	UBaseType_t uxPriority,
	StackType_t* const puxStackBuffer,
	StaticTask_t* const pxTaskBuffer)
{
	TCB_t* pxNewTCB;
	TaskHandle_t xReturn;

	configASSERT(puxStackBuffer != NULL);
	configASSERT(pxTaskBuffer != NULL);

#if( configASSERT_DEFINED == 1 )
	{
		/* Sanity check that the size of the structure used to declare a
		variable of type StaticTask_t equals the size of the real task
		structure. */
		volatile size_t xSize = sizeof(StaticTask_t);
		configASSERT(xSize == sizeof(TCB_t));
	}
#endif /* configASSERT_DEFINED */


	if ((pxTaskBuffer != NULL) && (puxStackBuffer != NULL))
	{
		/* The memory used for the task's TCB and stack are passed into this
		function - use them. */
		pxNewTCB = (TCB_t*)pxTaskBuffer; /*lint !e740 Unusual cast is ok as the structures are designed to have the same alignment, and the size is checked by an assert. */
		pxNewTCB->pxStack = (StackType_t*)puxStackBuffer;

#if( tskSTATIC_AND_DYNAMIC_ALLOCATION_POSSIBLE != 0 ) /*lint !e731 Macro has been consolidated for readability reasons. */
		{
			/* Tasks can be created statically or dynamically, so note this
			task was created statically in case the task is later deleted. */
			pxNewTCB->ucStaticallyAllocated = tskSTATICALLY_ALLOCATED_STACK_AND_TCB;
		}
#endif /* configSUPPORT_DYNAMIC_ALLOCATION */

		prvInitialiseNewTask(pxTaskCode, pcName, ulStackDepth, pvParameters, uxPriority, &xReturn, pxNewTCB, NULL);
		prvAddNewTaskToReadyList(pxNewTCB);
	}
	else
	{
		xReturn = NULL;
	}

	return xReturn;
}

#endif /* SUPPORT_STATIC_ALLOCATION */*/
#endif //THIS_REGION_ON
#pragma endregion from tasks.c
/*-----------------------------------------------------------*/

/* Convert from CMSIS type osPriority to FreeRTOS priority number */
static unsigned portBASE_TYPE makeFreeRtosPriority(osPriority priority)
{
	unsigned portBASE_TYPE fpriority = tskIDLE_PRIORITY;

	if (priority != osPriorityError) {
		fpriority += (priority - osPriorityIdle);
	}

	return fpriority;
}

/*********************** Thread Management *****************************/
/**
* @brief  Create a thread and add it to Active Threads and set it to state READY.
* @param  thread_def    thread definition referenced with \ref osThread.
* @param  argument      pointer that is passed to the thread function as start argument.
* @retval thread ID for reference by other functions or NULL in case of error.
* @note   MUST REMAIN UNCHANGED: \b osThreadCreate shall be consistent in every CMSIS-RTOS.
*/
osThreadId osThreadCreate (const osThreadDef_t *thread_def, void *argument)
{
  TaskHandle_t handle;

#if( configSUPPORT_STATIC_ALLOCATION == 1 ) &&  ( configSUPPORT_DYNAMIC_ALLOCATION == 1 )
  if((thread_def->buffer != NULL) && (thread_def->controlblock != NULL)) {
    handle = xTaskCreateStatic((TaskFunction_t)thread_def->pthread,(const portCHAR *)thread_def->name,
              thread_def->stacksize, argument, makeFreeRtosPriority(thread_def->tpriority),
              thread_def->buffer, thread_def->controlblock);
  }
  else {
  /*  if (xTaskCreate((TaskFunction_t)thread_def->pthread,(const portCHAR *)thread_def->name,
              thread_def->stacksize, argument, makeFreeRtosPriority(thread_def->tpriority),
              &handle) != pdPASS)*/  {
      return NULL;
    }
  }
#elif( configSUPPORT_STATIC_ALLOCATION == 1 )

    handle = xTaskCreateStatic((TaskFunction_t)thread_def->pthread,(const portCHAR *)thread_def->name,
              thread_def->stacksize, argument, makeFreeRtosPriority(thread_def->tpriority),
              thread_def->buffer, thread_def->controlblock);
#else
  if (xTaskCreate((TaskFunction_t)thread_def->pthread,(const portCHAR *)thread_def->name,
                   thread_def->stacksize, argument, makeFreeRtosPriority(thread_def->tpriority),
                   &handle) != pdPASS)  {
    return NULL;
  }
#endif

  return handle;
}