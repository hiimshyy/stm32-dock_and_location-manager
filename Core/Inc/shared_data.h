/*
 * shared_data.h
 *
 *  Created on: Aug 26, 2025
 *      Author: tiensy
 */

#ifndef INC_SHARED_DATA_H_
#define INC_SHARED_DATA_H_


#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Main shared data structure
 */
typedef struct {
    float speed;          ///< Vehicle speed in m/s
    float heading;        ///< Heading angle in degrees (0-360)
    uint8_t direction;    ///< Direction: 0=straight, 1=left, 2=right
    uint8_t station_status; ///< Station status: 0=not_docked, 1=docked
    uint32_t nfc_uid;     ///< NFC/RFID card UID
    uint16_t system_errors; ///< System error flags
    uint32_t timestamp;   ///< Last update timestamp
} shared_data_t;

/**
 * @brief IMU data structure
 */
typedef struct {
    // Raw sensor data
    float ax, ay, az;     ///< Accelerometer data (m/s²)
    float gx, gy, gz;     ///< Gyroscope data (deg/s)
    float mx, my, mz;     ///< Magnetometer data (optional)

    // Alternative naming for compatibility
    float accel_x, accel_y, accel_z;  ///< Same as ax, ay, az
    float gyro_x, gyro_y, gyro_z;     ///< Same as gx, gy, gz

    // Calibration offsets
    float ax_offset, ay_offset, az_offset;
    float gx_offset, gy_offset, gz_offset;

    // Processed data
    float velocity_x, velocity_y, velocity_z;  ///< Calculated velocities
    float heading_angle;           ///< Calculated heading
    float heading;                 ///< Same as heading_angle
    float speed;                   ///< Calculated speed

    // Status flags
    uint8_t calibrated;   ///< Calibration status
    uint8_t data_valid;   ///< Data validity flag
} imu_data_t;

/**
 * @brief System error flags
 */
typedef enum {
    ERROR_NONE          = 0x0000,
    ERROR_IMU_INIT      = 0x0001,
    ERROR_IMU_DATA      = 0x0002,
    ERROR_NFC_INIT      = 0x0004,
    ERROR_NFC_COMM      = 0x0008,
    ERROR_MODBUS_TIMEOUT = 0x0010,
    ERROR_MODBUS_CRC    = 0x0020,
    ERROR_STACK_OVERFLOW = 0x0040,
    ERROR_HEAP_OVERFLOW = 0x0080,
    ERROR_SENSOR_DISCONNECT = 0x0100
} system_error_flags_t;

/**
 * @brief Direction enumeration
 */
typedef enum {
    DIRECTION_STRAIGHT = 0,
    DIRECTION_TURN_LEFT = 1,
    DIRECTION_TURN_RIGHT = 2,
    DIRECTION_UNKNOWN = 3
} vehicle_direction_t;

/**
 * @brief Station status enumeration
 */
typedef enum {
    STATION_NOT_DOCKED = 0,
    STATION_DOCKED = 1,
    STATION_UNKNOWN = 2
} station_status_t;

/* Exported constants --------------------------------------------------------*/
#define SHARED_DATA_VERSION     1
#define MAX_SYSTEM_ERRORS       16

/* FreeRTOS Event Flags */
#define EVENT_STATION_DOCKED    0x01
#define EVENT_STATION_UNDOCKED  0x02
#define EVENT_NFC_SCAN_REQ      0x04
#define EVENT_SYSTEM_ERROR      0x08
#define EVENT_IMU_CALIBRATE     0x10

/* Exported variables --------------------------------------------------------*/
extern shared_data_t g_shared_data;
extern imu_data_t g_imu_data;
extern osMutexId g_shared_data_mutex;
extern osThreadId g_system_events_thread;

/* Exported functions --------------------------------------------------------*/

/**
 * @brief Initialize shared data structures
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef SharedData_Init(void);

/**
 * @brief Update shared data with mutex protection
 * @param data Pointer to new data
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef SharedData_Update(shared_data_t *data);

/**
 * @brief Read shared data with mutex protection
 * @param data Pointer to store read data
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef SharedData_Read(shared_data_t *data);

/**
 * @brief Set system error flag
 * @param error Error flag to set
 * @retval None
 */
void SharedData_SetError(system_error_flags_t error);

/**
 * @brief Clear system error flag
 * @param error Error flag to clear
 * @retval None
 */
void SharedData_ClearError(system_error_flags_t error);

/**
 * @brief Get current system error status
 * @retval uint16_t Current error flags
 */
uint16_t SharedData_GetErrors(void);

#ifdef __cplusplus
}
#endif


#endif /* INC_SHARED_DATA_H_ */
