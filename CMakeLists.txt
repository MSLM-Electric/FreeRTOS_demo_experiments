cmake_minimum_required(VERSION 3.12)

project(FreeRTOS_demo_experiments)
set(PROJ_NAME FreeRTOS_demo_experiments)

set(ROOT_PATH ${CMAKE_CURRENT_SOURCE_DIR})

set(FREERTOS_SRCS "${ROOT_PATH}/../../FreeRTOS_Source")
set(FREERTOS_INC "${FREERTOS_SRCS}/include")
set(FREERTOS_PORTABLE "${FREERTOS_SRCS}/portable")
set(FREERTOS_SUPPORT_FUNCS "${ROOT_PATH}/../../FreeRTOS_Source")
set(EXTERNAL_LIBS ${ROOT_PATH}/ExternalLibs)
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
)

set(HEADERS
    
    ${FREERTOS_PORTABLE}/MSVC-MingW/portmacro.h
)

add_executable(FreeRTOS_demo_experiments ${SOURCES} ${HEADERS})

if(${CMAKE_VERSION} VERSION_LESS "2.8.12")
    include_directories(FreeRTOS_demo_experiments PUBLIC ${INCLUDE_DIR})
else()
    target_include_directories(FreeRTOS_demo_experiments PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>/include)
endif()

target_link_options(${PROJ_NAME} PRIVATE "/MAP:${PROJECT_SOURCE_DIR}/outputMasterProject.map")
