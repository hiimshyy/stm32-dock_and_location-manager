/*
 * debugger.h
 *
 *  Created on: Sep 5, 2025
 *      Author: tiensy
 */

#ifndef INC_DEBUGGER_H_
#define INC_DEBUGGER_H_

#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "usbd_cdc_if.h"

/* Debug message levels */
typedef enum {
    DEBUG_LEVEL_ERROR = 0,
    DEBUG_LEVEL_WARNING = 1,
    DEBUG_LEVEL_INFO = 2,
    DEBUG_LEVEL_DEBUG = 3
} debug_level_t;

/* Debug configuration */
#define DEBUG_MAX_MESSAGE_SIZE  128     // Reduced from 256 to save memory
#define DEBUG_ENABLED           1
#define DEBUG_DEFAULT_LEVEL     DEBUG_LEVEL_DEBUG

/* Queue-based debug configuration */
#define DEBUG_QUEUE_SIZE        16      // Reduced from 32 to save memory
#define DEBUG_TASK_STACK_SIZE   256     // Reduced from 512 to save memory
#define DEBUG_TASK_PRIORITY     1       // Low priority (higher number = higher priority)

/* Ring buffer configuration */
#define DEBUG_RING_BUFFER_SIZE  1024    // Reduced from 2048 to save memory
#define DEBUG_USE_RING_BUFFER   0       // Disable ring buffer to save memory

/* Debug functions */
void Debug_Init(void);
void Debug_Printf(debug_level_t level, const char* format, ...);
void Debug_Print(debug_level_t level, const char* message);
void Debug_PrintHex(debug_level_t level, const char* prefix, uint8_t* data, uint16_t length);
uint8_t Debug_IsUSBReady(void);

/* Advanced debug functions */
void Debug_SetLevel(debug_level_t level);
debug_level_t Debug_GetLevel(void);
void Debug_PrintTaskInfo(void);
void Debug_PrintSystemInfo(void);

/* Queue-based debug functions */
void Debug_StartTask(void);
void Debug_StopTask(void);
uint32_t Debug_GetQueueUsage(void);
uint32_t Debug_GetDroppedMessages(void);
void Debug_FlushQueue(void);
uint8_t Debug_ProcessMessage(void);  // Call from StartDebugTask

/* Ring buffer functions */
void Debug_EnableRingBuffer(uint8_t enable);
uint32_t Debug_GetRingBufferUsage(void);
void Debug_DumpRingBuffer(void);

/* Convenience macros */
#if DEBUG_ENABLED
    #define DEBUG_ERROR(...)    Debug_Printf(DEBUG_LEVEL_ERROR, __VA_ARGS__)
    #define DEBUG_WARNING(...)  Debug_Printf(DEBUG_LEVEL_WARNING, __VA_ARGS__)
    #define DEBUG_INFO(...)     Debug_Printf(DEBUG_LEVEL_INFO, __VA_ARGS__)
    #define DEBUG_DEBUG(...)    Debug_Printf(DEBUG_LEVEL_DEBUG, __VA_ARGS__)
    #define DEBUG_HEX(prefix, data, len) Debug_PrintHex(DEBUG_LEVEL_DEBUG, prefix, data, len)
    
    /* Task-specific debug macros */
    #define DEBUG_TASK_START()  Debug_Printf(DEBUG_LEVEL_INFO, "Task started\r\n")
    #define DEBUG_TASK_ERROR(...)  Debug_Printf(DEBUG_LEVEL_ERROR, __VA_ARGS__)
    #define DEBUG_ENTER_FUNCTION()  Debug_Printf(DEBUG_LEVEL_DEBUG, "Entered %s()\r\n", __FUNCTION__)
    #define DEBUG_EXIT_FUNCTION()   Debug_Printf(DEBUG_LEVEL_DEBUG, "Exited %s()\r\n", __FUNCTION__)
    
    /* Queue monitoring macros */
    #define DEBUG_QUEUE_STATUS() Debug_Printf(DEBUG_LEVEL_INFO, "Queue: %lu%% used, %lu dropped\r\n", Debug_GetQueueUsage(), Debug_GetDroppedMessages())
    #define DEBUG_RING_STATUS()  Debug_Printf(DEBUG_LEVEL_INFO, "Ring Buffer: %lu%% used\r\n", Debug_GetRingBufferUsage())
#else
    #define DEBUG_ERROR(...)
    #define DEBUG_WARNING(...)
    #define DEBUG_INFO(...)
    #define DEBUG_DEBUG(...)
    #define DEBUG_HEX(prefix, data, len)
    #define DEBUG_TASK_START()
    #define DEBUG_TASK_ERROR(...)
    #define DEBUG_ENTER_FUNCTION()
    #define DEBUG_EXIT_FUNCTION()
    #define DEBUG_QUEUE_STATUS()
    #define DEBUG_RING_STATUS()
#endif

#endif /* INC_DEBUGGER_H_ */
