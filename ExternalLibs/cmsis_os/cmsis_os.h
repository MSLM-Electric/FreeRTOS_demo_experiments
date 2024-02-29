/* ----------------------------------------------------------------------
 * $Date:        5. February 2013
 * $Revision:    V1.02
 *
 * Project:      CMSIS-RTOS API
 * Title:        cmsis_os.h header file
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
 * Portions Copyright ? 2016 STMicroelectronics International N.V. All rights reserved.
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
  * @file    cmsis_os.h
  * @author  MCD Application Team
  * @date    13-July-2017
  * @brief   Header of cmsis_os.c
  *          A new set of APIs are added in addition to existing ones, these APIs
  *          are specific to FreeRTOS.
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
//#include "type_def.h"
#include "../type_def.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

/*from FreeRTOS.h*/
#ifndef configSUPPORT_STATIC_ALLOCATION
	/* Defaults to 0 for backward compatibility. */
#define configSUPPORT_STATIC_ALLOCATION 1
#endif

#ifndef configSUPPORT_DYNAMIC_ALLOCATION
	/* Defaults to 1 for backward compatibility. */
#define configSUPPORT_DYNAMIC_ALLOCATION 1
#endif

#define configINITIAL_TICK_COUNT 0
/***FreeRTOS.h****/

/// Entry point of a thread.
/// \note MUST REMAIN UNCHANGED: \b os_pthread shall be consistent in every CMSIS-RTOS.
typedef void (*os_pthread) (void const* argument);

/// Thread ID identifies the thread (pointer to a thread control block).
/// \note CAN BE CHANGED: \b os_thread_cb is implementation specific in every CMSIS-RTOS.
typedef TaskHandle_t osThreadId;

// ==== Enumeration, structures, defines ====

/// Priority used for thread control.
/// \note MUST REMAIN UNCHANGED: \b osPriority shall be consistent in every CMSIS-RTOS.
typedef enum {
	osPriorityIdle = -3,          ///< priority: idle (lowest)
	osPriorityLow = -2,          ///< priority: low
	osPriorityBelowNormal = -1,          ///< priority: below normal
	osPriorityNormal = 0,          ///< priority: normal (default)
	osPriorityAboveNormal = +1,          ///< priority: above normal
	osPriorityHigh = +2,          ///< priority: high
	osPriorityRealtime = +3,          ///< priority: realtime (highest)
	osPriorityError = 0x84        ///< system cannot determine priority or thread has illegal priority
} osPriority;


/*from FreeRTOS.h*/

/*
 * In line with software engineering best practice, FreeRTOS implements a strict
 * data hiding policy, so the real structures used by FreeRTOS to maintain the
 * state of tasks, queues, semaphores, etc. are not accessible to the application
 * code.  However, if the application writer wants to statically allocate such
 * an object then the size of the object needs to be know.  Dummy structures
 * that are guaranteed to have the same size and alignment requirements of the
 * real objects are used for this purpose.  The dummy list and list item
 * structures below are used for inclusion in such a dummy structure.
 */
struct xSTATIC_LIST_ITEM
{
	TickType_t xDummy1;
	void* pvDummy2[4];
};
typedef struct xSTATIC_LIST_ITEM StaticListItem_t;

/*
 * In line with software engineering best practice, especially when supplying a
 * library that is likely to change in future versions, FreeRTOS implements a
 * strict data hiding policy.  This means the Task structure used internally by
 * FreeRTOS is not accessible to application code.  However, if the application
 * writer wants to statically allocate the memory required to create a task then
 * the size of the task object needs to be know.  The StaticTask_t structure
 * below is provided for this purpose.  Its sizes and alignment requirements are
 * guaranteed to match those of the genuine structure, no matter which
 * architecture is being used, and no matter how the values in FreeRTOSConfig.h
 * are set.  Its contents are somewhat obfuscated in the hope users will
 * recognise that it would be unwise to make direct use of the structure members.
 */
typedef struct xSTATIC_TCB
{
	void* pxDummy1;
#if ( portUSING_MPU_WRAPPERS == 1 )
	xMPU_SETTINGS	xDummy2;
#endif
	StaticListItem_t	xDummy3[2];
	UBaseType_t			uxDummy5;
	void* pxDummy6;
	uint8_t				ucDummy7[configMAX_TASK_NAME_LEN];
#if ( ( portSTACK_GROWTH > 0 ) || ( configRECORD_STACK_HIGH_ADDRESS == 1 ) )
	void* pxDummy8;
#endif
#if ( portCRITICAL_NESTING_IN_TCB == 1 )
	UBaseType_t		uxDummy9;
#endif
#if ( configUSE_TRACE_FACILITY == 1 )
	UBaseType_t		uxDummy10[2];
#endif
#if ( configUSE_MUTEXES == 1 )
	UBaseType_t		uxDummy12[2];
#endif
#if ( configUSE_APPLICATION_TASK_TAG == 1 )
	void* pxDummy14;
#endif
#if( configNUM_THREAD_LOCAL_STORAGE_POINTERS > 0 )
	void* pvDummy15[configNUM_THREAD_LOCAL_STORAGE_POINTERS];
#endif
#if ( configGENERATE_RUN_TIME_STATS == 1 )
	uint32_t		ulDummy16;
#endif
#if ( configUSE_NEWLIB_REENTRANT == 1 )
	struct	_reent	xDummy17;
#endif
#if ( configUSE_TASK_NOTIFICATIONS == 1 )
	uint32_t 		ulDummy18;
	uint8_t 		ucDummy19;
#endif
#if( ( ( configSUPPORT_STATIC_ALLOCATION == 1 ) && ( configSUPPORT_DYNAMIC_ALLOCATION == 1 ) ) || ( portUSING_MPU_WRAPPERS == 1 ) )
	uint8_t			uxDummy20;
#endif

#if( INCLUDE_xTaskAbortDelay == 1 )
	uint8_t ucDummy21;
#endif

} StaticTask_t;

#if( configSUPPORT_STATIC_ALLOCATION == 1 )

typedef StaticTask_t               osStaticThreadDef_t;
//typedef StaticTimer_t              osStaticTimerDef_t;
//typedef StaticSemaphore_t          osStaticMutexDef_t;
//typedef StaticSemaphore_t          osStaticSemaphoreDef_t;
//typedef StaticQueue_t              osStaticMessageQDef_t;

#endif
/***FreeRTOS.h****/

/// Thread Definition structure contains startup information of a thread.
/// \note CAN BE CHANGED: \b os_thread_def is implementation specific in every CMSIS-RTOS.
typedef struct os_thread_def {
	char* name;        ///< Thread name 
	os_pthread             pthread;      ///< start address of thread function
	osPriority             tpriority;    ///< initial thread priority
	uint32_t               instances;    ///< maximum number of instances of that thread function
	uint32_t               stacksize;    ///< stack size requirements in bytes; 0 is default stack size
#if( configSUPPORT_STATIC_ALLOCATION == 1 )
	uint32_t* buffer;      ///< stack buffer for static allocation; NULL for dynamic allocation
	osStaticThreadDef_t* controlblock;     ///< control block to hold thread's data for static allocation; NULL for dynamic allocation
#endif
} osThreadDef_t;

//  ==== Thread Management ====

/// Create a Thread Definition with function, priority, and stack requirements.
/// \param         name         name of the thread function.
/// \param         priority     initial priority of the thread function.
/// \param         instances    number of possible thread instances.
/// \param         stacksz      stack size (in bytes) requirements for the thread function.
/// \note CAN BE CHANGED: The parameters to \b osThreadDef shall be consistent but the
///       macro body is implementation specific in every CMSIS-RTOS.
#if defined (osObjectsExternal)  // object is external
#define osThreadDef(name, thread, priority, instances, stacksz)  \
extern const osThreadDef_t os_thread_def_##name
#else                            // define the object

#if( configSUPPORT_STATIC_ALLOCATION == 1 )
#define osThreadDef(name, thread, priority, instances, stacksz)  \
const osThreadDef_t os_thread_def_##name = \
{ #name, (thread), (priority), (instances), (stacksz), NULL, NULL }

#define osThreadStaticDef(name, thread, priority, instances, stacksz, buffer, control)  \
const osThreadDef_t os_thread_def_##name = \
{ #name, (thread), (priority), (instances), (stacksz), (buffer), (control) }
#else //configSUPPORT_STATIC_ALLOCATION == 0

#define osThreadDef(name, thread, priority, instances, stacksz)  \
const osThreadDef_t os_thread_def_##name = \
{ #name, (thread), (priority), (instances), (stacksz)}
#endif
#endif

osThreadId osThreadCreate(const osThreadDef_t* thread_def, void* argument);