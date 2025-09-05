/*
 * debugger.c
 *
 *  Created on: Sep 5, 2025
 *      Author: tiensy
 */

#include "debugger.h"
#include "cmsis_os.h"
#include <string.h>

/* Debug message structure for queue */
typedef struct {
    debug_level_t level;
    char task_name[16];
    uint32_t timestamp;
    char message[DEBUG_MAX_MESSAGE_SIZE];
    uint16_t length;
} debug_message_t;

/* Private variables */
static debug_level_t current_debug_level = DEBUG_DEFAULT_LEVEL;
static osMutexId debug_mutex;

/* Enhanced protection variables */
static uint8_t debug_system_ready = 0;

/* Queue-based debug variables */
extern osMessageQId debugMsgQueueHandle;  // From freertos.c
extern osThreadId debugTaskHandle;        // From freertos.c
static uint32_t debug_dropped_messages = 0;
static uint8_t debug_task_running = 0;

/* Static message buffer for queue (since CubeIDE queue only supports uint16_t) */
static debug_message_t debug_msg_buffer[DEBUG_QUEUE_SIZE];
static volatile uint8_t buffer_write_index = 0;
static volatile uint8_t buffer_read_index = 0;
static volatile uint8_t buffer_count = 0;

/* Ring buffer variables */
#if DEBUG_USE_RING_BUFFER
static char debug_ring_buffer[DEBUG_RING_BUFFER_SIZE];
static volatile uint32_t ring_buffer_head = 0;
static volatile uint32_t ring_buffer_tail = 0;
static volatile uint32_t ring_buffer_count = 0;
static uint8_t ring_buffer_enabled = 1;
static osMutexId ring_buffer_mutex = NULL;
#endif

/* Private function prototypes */
static const char* Debug_GetLevelString(debug_level_t level);
static uint32_t Debug_GetTimestamp(void);
static uint8_t Debug_TransmitWithRetry(uint8_t* data, uint16_t length);
static uint8_t Debug_QueueMessage(debug_level_t level, const char* message, uint16_t length);

#if DEBUG_USE_RING_BUFFER
static void Debug_RingBufferWrite(const char* data, uint16_t length);
static uint32_t Debug_RingBufferRead(char* buffer, uint32_t max_length);
#endif

/**
 * @brief Initialize debug system
 */
void Debug_Init(void)
{
    // Create mutex for thread-safe debug output (CMSIS-RTOS V1 style)
    osMutexDef(DebugMutex);
    debug_mutex = osMutexCreate(osMutex(DebugMutex));
    
#if DEBUG_USE_RING_BUFFER
    // Create ring buffer mutex (CMSIS-RTOS V1 style)
    osMutexDef(RingBufferMutex);
    ring_buffer_mutex = osMutexCreate(osMutex(RingBufferMutex));
    
    // Initialize ring buffer
    ring_buffer_head = 0;
    ring_buffer_tail = 0;
    ring_buffer_count = 0;
#endif
    
    // Queue and task are already created by CubeIDE in freertos.c
    debug_task_running = (debugTaskHandle != NULL && debugMsgQueueHandle != NULL);
    
    // Mark system as ready
    debug_system_ready = 1;
    
    // Send initial debug message
    Debug_Printf(DEBUG_LEVEL_INFO, "\r\n=== Queue-Based Debug System Initialized ===\r\n");
    Debug_Printf(DEBUG_LEVEL_INFO, "System Clock: %lu Hz\r\n", HAL_RCC_GetSysClockFreq());
    Debug_Printf(DEBUG_LEVEL_INFO, "Debug Level: %s\r\n", Debug_GetLevelString(current_debug_level));
    Debug_Printf(DEBUG_LEVEL_INFO, "Queue Size: %d messages\r\n", DEBUG_QUEUE_SIZE);
#if DEBUG_USE_RING_BUFFER
    Debug_Printf(DEBUG_LEVEL_INFO, "Ring Buffer: %d bytes\r\n", DEBUG_RING_BUFFER_SIZE);
#endif
    Debug_Printf(DEBUG_LEVEL_INFO, "Debug Task: %s\r\n", debug_task_running ? "Running" : "Stopped");
}

/**
 * @brief Print formatted debug message
 * @param level Debug level
 * @param format Printf-style format string
 * @param ... Variable arguments
 */
void Debug_Printf(debug_level_t level, const char* format, ...)
{
    // Early exit checks
    if (!debug_system_ready || level > current_debug_level) {
        return;
    }
    
    // Format message locally (each task has its own stack)
    char local_buffer[DEBUG_MAX_MESSAGE_SIZE];
    va_list args;
    va_start(args, format);
    
    int msg_len = vsnprintf(local_buffer, DEBUG_MAX_MESSAGE_SIZE, format, args);
    va_end(args);
    
    if (msg_len > 0) {
        // Queue the message (non-blocking for tasks)
        if (!Debug_QueueMessage(level, local_buffer, msg_len)) {
            // Queue full - increment dropped counter
            debug_dropped_messages++;
            
#if DEBUG_USE_RING_BUFFER
            // Fallback to ring buffer if enabled
            if (ring_buffer_enabled) {
                char formatted_msg[DEBUG_MAX_MESSAGE_SIZE];
                int total_len = snprintf(formatted_msg, DEBUG_MAX_MESSAGE_SIZE,
                                       "[%08lu][%s][%s] %s",
                                       Debug_GetTimestamp(),
                                       Debug_GetLevelString(level),
                                       pcTaskGetName(NULL),
                                       local_buffer);
                
                if (total_len > 0) {
                    Debug_RingBufferWrite(formatted_msg, total_len);
                }
            }
#endif
        }
    }
}

/**
 * @brief Print debug message without formatting
 * @param level Debug level
 * @param message Message string
 */
void Debug_Print(debug_level_t level, const char* message)
{
    Debug_Printf(level, "%s", message);
}

/**
 * @brief Print hexadecimal data
 * @param level Debug level
 * @param prefix Prefix string
 * @param data Pointer to data
 * @param length Data length
 */
void Debug_PrintHex(debug_level_t level, const char* prefix, uint8_t* data, uint16_t length)
{
    // Early exit checks
    if (!debug_system_ready || level > current_debug_level || data == NULL) {
        return;
    }
    
    // Send header message
    char header_msg[128];
    snprintf(header_msg, sizeof(header_msg), "%s (%d bytes):\r\n", prefix, length);
    Debug_Printf(level, "%s", header_msg);
    
    // Process hex dump in chunks of 16 bytes
    for (uint16_t i = 0; i < length; i += 16) {
        char line_msg[128];
        int line_len = snprintf(line_msg, sizeof(line_msg), "%04X: ", i);
        
        // Add hex bytes
        uint16_t line_end = (i + 16 < length) ? i + 16 : length;
        for (uint16_t j = i; j < line_end; j++) {
            line_len += snprintf(line_msg + line_len, 
                               sizeof(line_msg) - line_len, 
                               "%02X ", data[j]);
        }
        
        // Pad with spaces if needed
        for (uint16_t j = line_end; j < i + 16; j++) {
            line_len += snprintf(line_msg + line_len, 
                               sizeof(line_msg) - line_len, "   ");
        }
        
        // Add ASCII representation
        line_len += snprintf(line_msg + line_len, 
                           sizeof(line_msg) - line_len, " |");
        
        for (uint16_t j = i; j < line_end; j++) {
            char c = (data[j] >= 32 && data[j] <= 126) ? data[j] : '.';
            line_len += snprintf(line_msg + line_len, 
                               sizeof(line_msg) - line_len, "%c", c);
        }
        
        line_len += snprintf(line_msg + line_len, 
                           sizeof(line_msg) - line_len, "|\r\n");
        
        // Queue this line
        Debug_Printf(level, "%s", line_msg);
    }
}

/**
 * @brief Check if USB CDC is ready for transmission
 * @return 1 if ready, 0 if not ready
 */
uint8_t Debug_IsUSBReady(void)
{
    // Check if USB device is configured and CDC is ready
    extern USBD_HandleTypeDef hUsbDeviceFS;
    
    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) {
        return 0;
    }
    
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
    if (hcdc == NULL || hcdc->TxState != 0) {
        return 0;
    }
    
    return 1;
}

/**
 * @brief Get debug level string
 * @param level Debug level
 * @return Level string
 */
static const char* Debug_GetLevelString(debug_level_t level)
{
    switch (level) {
        case DEBUG_LEVEL_ERROR:   return "ERROR";
        case DEBUG_LEVEL_WARNING: return "WARN ";
        case DEBUG_LEVEL_INFO:    return "INFO ";
        case DEBUG_LEVEL_DEBUG:   return "DEBUG";
        default:                  return "UNKN ";
    }
}

/**
 * @brief Get current timestamp in milliseconds
 * @return Timestamp
 */
static uint32_t Debug_GetTimestamp(void)
{
    return osKernelSysTick(); // CMSIS-RTOS V1 API
}

/**
 * @brief Transmit data via USB CDC with retry mechanism
 * @param data Pointer to data
 * @param length Data length
 * @return 1 if successful, 0 if failed
 */
static uint8_t Debug_TransmitWithRetry(uint8_t* data, uint16_t length)
{
    const uint8_t max_retries = 3;
    const uint32_t retry_delay_ms = 10;
    
    for (uint8_t retry = 0; retry < max_retries; retry++) {
        uint8_t result = CDC_Transmit_FS(data, length);
        
        if (result == USBD_OK) {
            return 1; // Success
        }
        
        if (result == USBD_BUSY && retry < max_retries - 1) {
            // Wait and retry if busy
            osDelay(retry_delay_ms);
            continue;
        }
        
        // Other errors or max retries reached
        break;
    }
    
    return 0; // Failed
}

/**
 * @brief Set debug level
 * @param level New debug level
 */
void Debug_SetLevel(debug_level_t level)
{
    current_debug_level = level;
    DEBUG_INFO("Debug level changed to: %s\r\n", Debug_GetLevelString(level));
}

/**
 * @brief Get current debug level
 * @return Current debug level
 */
debug_level_t Debug_GetLevel(void)
{
    return current_debug_level;
}

/**
 * @brief Print information about all running tasks
 */
void Debug_PrintTaskInfo(void)
{
    if (!debug_system_ready || !Debug_IsUSBReady()) {
        return;
    }
    
    // Simple task count only (to avoid memory allocation and undefined references)
    UBaseType_t task_count = uxTaskGetNumberOfTasks();
    
    DEBUG_INFO("=== TASK INFORMATION ===\r\n");
    DEBUG_INFO("Total Tasks: %lu\r\n", task_count);
    DEBUG_INFO("Note: Detailed task info disabled to save memory\r\n");
    DEBUG_INFO("Available functions: uxTaskGetNumberOfTasks()\r\n");
}

/**
 * @brief Print system information
 */
void Debug_PrintSystemInfo(void)
{
    if (!debug_system_ready || !Debug_IsUSBReady()) {
        return;
    }
    
    DEBUG_INFO("=== SYSTEM INFORMATION ===\r\n");
    DEBUG_INFO("System Clock: %lu Hz\r\n", HAL_RCC_GetSysClockFreq());
    DEBUG_INFO("FreeRTOS Kernel: %s\r\n", tskKERNEL_VERSION_NUMBER);
    DEBUG_INFO("Tick Rate: %lu Hz\r\n", configTICK_RATE_HZ);
    DEBUG_INFO("Total Heap: %lu bytes\r\n", configTOTAL_HEAP_SIZE);
    DEBUG_INFO("Free Heap: %lu bytes\r\n", xPortGetFreeHeapSize());
    DEBUG_INFO("Min Free Heap: %lu bytes\r\n", xPortGetMinimumEverFreeHeapSize());
    DEBUG_INFO("Current Tick: %lu\r\n", osKernelSysTick());
    DEBUG_INFO("CPU Usage: Not implemented\r\n");
    
    // Print task count
    UBaseType_t task_count = uxTaskGetNumberOfTasks();
    DEBUG_INFO("Running Tasks: %lu\r\n", task_count);
}

/**
 * @brief Start debug output task (Task is created by CubeIDE)
 */
void Debug_StartTask(void)
{
    // Task is already created by CubeIDE, just check if running
    debug_task_running = (debugTaskHandle != NULL);
}

/**
 * @brief Stop debug output task
 */
void Debug_StopTask(void)
{
    if (debugTaskHandle != NULL) {
        osThreadTerminate(debugTaskHandle);
        debug_task_running = 0;
    }
}

/**
 * @brief Get debug queue usage percentage
 * @return Queue usage (0-100%)
 */
uint32_t Debug_GetQueueUsage(void)
{
    if (debugMsgQueueHandle == NULL) {
        return 0;
    }
    
    // Calculate usage based on buffer count
    return (buffer_count * 100) / DEBUG_QUEUE_SIZE;
}

/**
 * @brief Get number of dropped messages
 * @return Number of dropped messages
 */
uint32_t Debug_GetDroppedMessages(void)
{
    return debug_dropped_messages;
}

/**
 * @brief Flush debug queue (clear all pending messages)
 */
void Debug_FlushQueue(void)
{
    if (debugMsgQueueHandle == NULL) {
        return;
    }
    
    // CMSIS-RTOS V1 style - flush queue by getting all messages
    osEvent event;
    do {
        event = osMessageGet(debugMsgQueueHandle, 0); // Non-blocking get
    } while (event.status == osEventMessage);
}

/**
 * @brief Process one debug message from queue (call from StartDebugTask)
 * @return 1 if message processed, 0 if no message or error
 */
uint8_t Debug_ProcessMessage(void)
{
    if (debugMsgQueueHandle == NULL || buffer_count == 0) {
        return 0;
    }
    
    // Try to get message index from queue (non-blocking)
    osEvent event = osMessageGet(debugMsgQueueHandle, 0);
    
    if (event.status == osEventMessage) {
        // We have a buffer index
        uint8_t msg_index = (uint8_t)event.value.v;
        
        if (msg_index < DEBUG_QUEUE_SIZE) {
            debug_message_t* msg = &debug_msg_buffer[msg_index];
            
            // Wait for USB to be ready (with timeout)
            uint32_t timeout_start = osKernelSysTick();
            while (!Debug_IsUSBReady()) {
                osDelay(10);
                // Timeout after 1 second
                if ((osKernelSysTick() - timeout_start) > 1000) {
                    break;
                }
            }
            
            // Format final output with all information
            char output_buffer[DEBUG_MAX_MESSAGE_SIZE + 64];
            int output_len = snprintf(output_buffer, sizeof(output_buffer),
                                    "[%08lu][%s][%s] %s",
                                    msg->timestamp,
                                    Debug_GetLevelString(msg->level),
                                    msg->task_name,
                                    msg->message);
            
            // Send via USB CDC
            if (output_len > 0 && Debug_IsUSBReady()) {
                Debug_TransmitWithRetry((uint8_t*)output_buffer, output_len);
            }
            
            // Update buffer management
            buffer_count--;
            
            return 1; // Message processed
        }
    }
    
    return 0; // No message or error
}

/**
 * @brief Queue a debug message
 * @param level Debug level
 * @param message Message string
 * @param length Message length
 * @return 1 if queued successfully, 0 if queue full
 */
static uint8_t Debug_QueueMessage(debug_level_t level, const char* message, uint16_t length)
{
    if (debugMsgQueueHandle == NULL) {
        return 0;
    }
    
    // Check if buffer is full
    if (buffer_count >= DEBUG_QUEUE_SIZE) {
        return 0; // Buffer full
    }
    
    // Prepare message in static buffer
    debug_message_t* msg = &debug_msg_buffer[buffer_write_index];
    msg->level = level;
    msg->timestamp = Debug_GetTimestamp();
    msg->length = (length > DEBUG_MAX_MESSAGE_SIZE - 1) ? DEBUG_MAX_MESSAGE_SIZE - 1 : length;
    
    // Copy task name safely
    const char* task_name = pcTaskGetName(NULL);
    if (task_name != NULL) {
        strncpy(msg->task_name, task_name, sizeof(msg->task_name) - 1);
        msg->task_name[sizeof(msg->task_name) - 1] = '\0';
    } else {
        strcpy(msg->task_name, "Unknown");
    }
    
    // Copy message
    memcpy(msg->message, message, msg->length);
    msg->message[msg->length] = '\0';
    
    // Send buffer index via queue
    if (osMessagePut(debugMsgQueueHandle, buffer_write_index, 0) == osOK) {
        buffer_write_index = (buffer_write_index + 1) % DEBUG_QUEUE_SIZE;
        buffer_count++;
        return 1;
    }
    
    return 0;
}

#if DEBUG_USE_RING_BUFFER
/**
 * @brief Enable/disable ring buffer
 * @param enable 1 to enable, 0 to disable
 */
void Debug_EnableRingBuffer(uint8_t enable)
{
    ring_buffer_enabled = enable;
}

/**
 * @brief Get ring buffer usage percentage
 * @return Usage (0-100%)
 */
uint32_t Debug_GetRingBufferUsage(void)
{
    return (ring_buffer_count * 100) / DEBUG_RING_BUFFER_SIZE;
}

/**
 * @brief Dump ring buffer contents to debug output
 */
void Debug_DumpRingBuffer(void)
{
    if (!ring_buffer_enabled || ring_buffer_count == 0) {
        return;
    }
    
    char temp_buffer[256];
    
    // Read and output ring buffer contents
    while (ring_buffer_count > 0) {
        uint32_t read_len = Debug_RingBufferRead(temp_buffer, sizeof(temp_buffer) - 1);
        if (read_len > 0) {
            temp_buffer[read_len] = '\0';
            if (Debug_IsUSBReady()) {
                Debug_TransmitWithRetry((uint8_t*)temp_buffer, read_len);
            }
        } else {
            break;
        }
    }
}

/**
 * @brief Write data to ring buffer
 * @param data Data to write
 * @param length Data length
 */
static void Debug_RingBufferWrite(const char* data, uint16_t length)
{
    if (ring_buffer_mutex != NULL) {
        osMutexWait(ring_buffer_mutex, osWaitForever);
    }
    
    for (uint16_t i = 0; i < length; i++) {
        debug_ring_buffer[ring_buffer_head] = data[i];
        ring_buffer_head = (ring_buffer_head + 1) % DEBUG_RING_BUFFER_SIZE;
        
        if (ring_buffer_count < DEBUG_RING_BUFFER_SIZE) {
            ring_buffer_count++;
        } else {
            // Buffer full - advance tail (overwrite old data)
            ring_buffer_tail = (ring_buffer_tail + 1) % DEBUG_RING_BUFFER_SIZE;
        }
    }
    
    if (ring_buffer_mutex != NULL) {
        osMutexRelease(ring_buffer_mutex);
    }
}

/**
 * @brief Read data from ring buffer
 * @param buffer Output buffer
 * @param max_length Maximum length to read
 * @return Number of bytes read
 */
static uint32_t Debug_RingBufferRead(char* buffer, uint32_t max_length)
{
    if (ring_buffer_mutex != NULL) {
        osMutexWait(ring_buffer_mutex, osWaitForever);
    }
    
    uint32_t read_count = 0;
    
    while (ring_buffer_count > 0 && read_count < max_length) {
        buffer[read_count] = debug_ring_buffer[ring_buffer_tail];
        ring_buffer_tail = (ring_buffer_tail + 1) % DEBUG_RING_BUFFER_SIZE;
        ring_buffer_count--;
        read_count++;
    }
    
    if (ring_buffer_mutex != NULL) {
        osMutexRelease(ring_buffer_mutex);
    }
    
    return read_count;
}
#endif
