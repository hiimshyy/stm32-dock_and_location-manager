/*
 * shared_data.c
 *
 *  Created on: Aug 26, 2025
 *      Author: tiensy
 */

/* Includes ------------------------------------------------------------------*/
#include "shared_data.h"
#include <string.h>

/* Private variables ---------------------------------------------------------*/
shared_data_t g_shared_data = {0};
imu_data_t g_imu_data = {0};
osMutexId g_shared_data_mutex = NULL;
osThreadId g_system_events_thread = NULL;

/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/**
 * @brief Initialize shared data structures
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef SharedData_Init(void)
{
    // Initialize shared data structure
    memset(&g_shared_data, 0, sizeof(shared_data_t));
    memset(&g_imu_data, 0, sizeof(imu_data_t));

    // Set initial values
    g_shared_data.speed = 0.0f;
    g_shared_data.heading = 0.0f;
    g_shared_data.direction = DIRECTION_STRAIGHT;
    g_shared_data.station_status = STATION_NOT_DOCKED;
    g_shared_data.nfc_uid = 0;
    g_shared_data.system_errors = ERROR_NONE;
    g_shared_data.timestamp = 0;

    // Initialize IMU data
    g_imu_data.calibrated = 0;
    g_imu_data.data_valid = 0;

    // Create mutex for shared data protection
    osMutexDef(SharedDataMutex);
    g_shared_data_mutex = osMutexCreate(osMutex(SharedDataMutex));

    if (g_shared_data_mutex == NULL) {
        return HAL_ERROR;
    }

    // Note: Event flags functionality will be implemented using thread signals
    // or message queues in FreeRTOS v1

    return HAL_OK;
}

/**
 * @brief Update shared data with mutex protection
 * @param data Pointer to new data
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef SharedData_Update(shared_data_t *data)
{
    if (data == NULL) {
        return HAL_ERROR;
    }

    osStatus status = osMutexWait(g_shared_data_mutex, osWaitForever);
    if (status != osOK) {
        return HAL_ERROR;
    }

    // Update timestamp
    data->timestamp = osKernelSysTick();

    // Copy data
    memcpy(&g_shared_data, data, sizeof(shared_data_t));

    osMutexRelease(g_shared_data_mutex);
    return HAL_OK;
}

/**
 * @brief Read shared data with mutex protection
 * @param data Pointer to store read data
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef SharedData_Read(shared_data_t *data)
{
    if (data == NULL) {
        return HAL_ERROR;
    }

    osStatus status = osMutexWait(g_shared_data_mutex, osWaitForever);
    if (status != osOK) {
        return HAL_ERROR;
    }

    // Copy data
    memcpy(data, &g_shared_data, sizeof(shared_data_t));

    osMutexRelease(g_shared_data_mutex);
    return HAL_OK;
}

/**
 * @brief Set system error flag
 * @param error Error flag to set
 * @retval None
 */
void SharedData_SetError(system_error_flags_t error)
{
    osStatus status = osMutexWait(g_shared_data_mutex, osWaitForever);
    if (status == osOK) {
        g_shared_data.system_errors |= error;
        osMutexRelease(g_shared_data_mutex);

        // Note: Event signaling removed - implement with thread signals if needed
    }
}

/**
 * @brief Clear system error flag
 * @param error Error flag to clear
 * @retval None
 */
void SharedData_ClearError(system_error_flags_t error)
{
    osStatus status = osMutexWait(g_shared_data_mutex, osWaitForever);
    if (status == osOK) {
        g_shared_data.system_errors &= ~error;
        osMutexRelease(g_shared_data_mutex);
    }
}

/**
 * @brief Get current system error status
 * @retval uint16_t Current error flags
 */
uint16_t SharedData_GetErrors(void)
{
    uint16_t errors = 0;

    osStatus status = osMutexWait(g_shared_data_mutex, osWaitForever);
    if (status == osOK) {
        errors = g_shared_data.system_errors;
        osMutexRelease(g_shared_data_mutex);
    }

    return errors;
}


