cmake_minimum_required(VERSION 3.12)

project(FreeRTOS_demo_experiments)
set(PROJ_NAME FreeRTOS_demo_experiments)

set(ROOT_PATH ${CMAKE_CURRENT_SOURCE_DIR})

message("Start building project...")

set(FREERTOS_SRCS "${ROOT_PATH}/../../../../../FreeRTOS_Source")
set(FREERTOS_INC "${FREERTOS_SRCS}/include")
set(FREERTOS_PORTABLE "${FREERTOS_SRCS}/portable")
set(FREERTOS_SUPPORT_FUNCS "${ROOT_PATH}/../../../../../Supporting_Functions")
set(EXTERNAL_LIBS "${ROOT_PATH}/../../../ExternalLibs")
set(BITLOGGER ${EXTERNAL_LIBS}/BitLogger)
set(SIMPLETIMER ${EXTERNAL_LIBS}/SimpleTimer)
set(CMSIS_OS ${EXTERNAL_LIBS}/cmsis_os)

#include(Configs.cmake)

set(SOURCES
	${FREERTOS_SRCS}/event_groups.c
    ${FREERTOS_SRCS}/list.c
    ${FREERTOS_SRCS}/queue.c
    ${FREERTOS_SRCS}/tasks.c
    ${FREERTOS_SRCS}/timers.c

    ${FREERTOS_PORTABLE}/MemMang/heap_4.c
    ${FREERTOS_PORTABLE}/MSVC-MingW/port.c

    ${FREERTOS_SUPPORT_FUNCS}/supporting_functions.c

    ${BITLOGGER}/BitLogger.c
    ${CMSIS_OS}/cmsis_os.c
    ${SIMPLETIMER}/SimpleTimerWP.c

    ${ROOT_PATH}/main.c
)

set(HEADERS
    ${FREERTOS_INC}/croutine.h
    ${FREERTOS_INC}/deprecated_definitions.h
    ${FREERTOS_INC}/event_groups.h
    ${FREERTOS_INC}/FreeRTOS.h
    ${FREERTOS_INC}/list.h
    ${FREERTOS_INC}/mpu_wrappers.h
    ${FREERTOS_INC}/portable.h
    ${FREERTOS_INC}/projdefs.h
    ${FREERTOS_INC}/queue.h
    ${FREERTOS_INC}/semphr.h
    ${FREERTOS_INC}/StackMacros.h
    ${FREERTOS_INC}/task.h
    ${FREERTOS_INC}/timers.h

    ${FREERTOS_PORTABLE}/MSVC-MingW/portmacro.h

    ${FREERTOS_SUPPORT_FUNCS}/supporting_functions.h

    ${EXTERNAL_LIBS}/type_def.h
    ${BITLOGGER}/BitLogger.h
    ${CMSIS_OS}/cmsis_os.h
    ${SIMPLETIMER}/SimpleTimerWP.h
)

add_executable(FreeRTOS_demo_experiments ${SOURCES} ${HEADERS})

if(${CMAKE_VERSION} VERSION_LESS "2.8.12")
    include_directories(FreeRTOS_demo_experiments PUBLIC ${INCLUDE_DIR})
else()
    target_include_directories(FreeRTOS_demo_experiments PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>/include)
endif()

target_link_options(${PROJ_NAME} PRIVATE "/MAP:${PROJECT_SOURCE_DIR}/outputMasterProject.map")
