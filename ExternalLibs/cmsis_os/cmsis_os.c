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

#if( configSUPPORT_STATIC_ALLOCATION == 1 )

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