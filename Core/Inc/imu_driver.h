/*
 * imu_driver.h
 *
 *  Created on: Aug 26, 2025
 *      Author: tiensy
 */

#ifndef INC_IMU_DRIVER_H_
#define INC_IMU_DRIVER_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "shared_data.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief IMU sensor type
 */
typedef enum {
    IMU_TYPE_MPU6050 = 0,
    IMU_TYPE_LSM6DS3,
    IMU_TYPE_BNO055,
    IMU_TYPE_BMI160,
    IMU_TYPE_UNKNOWN
} imu_sensor_type_t;

/**
 * @brief IMU configuration structure
 */
typedef struct {
    imu_sensor_type_t sensor_type;
    I2C_HandleTypeDef *hi2c;
    uint8_t device_address;

    // Sensor ranges
    uint8_t accel_range;    ///< ±2g, ±4g, ±8g, ±16g
    uint8_t gyro_range;     ///< ±250, ±500, ±1000, ±2000 dps
    uint8_t sample_rate;    ///< Sample rate in Hz

    // Filter settings
    uint8_t dlpf_enable;    ///< Digital Low Pass Filter enable
    uint8_t dlpf_config;    ///< DLPF configuration

    // Calibration settings
    uint16_t calibration_samples;  ///< Number of samples for calibration
} imu_config_t;

/**
 * @brief IMU status enumeration
 */
typedef enum {
    IMU_STATUS_OK = 0,
    IMU_STATUS_ERROR,
    IMU_STATUS_NOT_INITIALIZED,
    IMU_STATUS_CALIBRATING,
    IMU_STATUS_DATA_NOT_READY
} imu_status_t;

/* Exported constants --------------------------------------------------------*/

// MPU6050 specific constants
#define MPU6050_ADDRESS         0x68
#define MPU6050_WHO_AM_I_REG    0x75
#define MPU6050_WHO_AM_I_VALUE  0x68

// LSM6DS3 specific constants
#define LSM6DS3_ADDRESS         0x6A
#define LSM6DS3_WHO_AM_I_REG    0x0F
#define LSM6DS3_WHO_AM_I_VALUE  0x69

// BNO055 specific constants
#define BNO055_ADDRESS_A        0x28    ///< Primary I2C address (COM3 low)
#define BNO055_ADDRESS_B        0x29    ///< Alternative I2C address (COM3 high)
#define BNO055_CHIP_ID_REG      0x00    ///< Chip ID register
#define BNO055_CHIP_ID_VALUE    0xA0    ///< Expected chip ID value

// Common constants
#define IMU_GRAVITY             9.81f   ///< Gravity acceleration (m/s²)
#define IMU_DEG_TO_RAD          0.017453292519943295f
#define IMU_RAD_TO_DEG          57.29577951308232f

// Filter constants
#define IMU_ACCEL_FILTER_ALPHA  0.8f    ///< Low-pass filter alpha
#define IMU_GYRO_FILTER_ALPHA   0.9f    ///< Low-pass filter alpha
#define IMU_VELOCITY_DECAY      0.99f   ///< Velocity decay factor

// Detection thresholds
#define IMU_TURN_THRESHOLD      5.0f    ///< Gyro threshold for turn detection (deg/s)
#define IMU_MOTION_THRESHOLD    0.1f    ///< Motion detection threshold (m/s²)

/* Exported variables --------------------------------------------------------*/
extern imu_config_t g_imu_config;

/* Exported functions --------------------------------------------------------*/

/**
 * @brief Initialize IMU sensor
 * @param config Pointer to IMU configuration
 * @retval imu_status_t
 */
imu_status_t IMU_Init(imu_config_t *config);

/**
 * @brief Deinitialize IMU sensor
 * @retval imu_status_t
 */
imu_status_t IMU_DeInit(void);

/**
 * @brief Calibrate IMU sensor (must be stationary)
 * @retval imu_status_t
 */
imu_status_t IMU_Calibrate(void);

/**
 * @brief Read raw sensor data
 * @param ax, ay, az Accelerometer data output
 * @param gx, gy, gz Gyroscope data output
 * @retval imu_status_t
 */
imu_status_t IMU_ReadRaw(float *ax, float *ay, float *az, float *gx, float *gy, float *gz);

/**
 * @brief Process IMU data and update calculations
 * @param dt Time delta in seconds
 * @retval imu_status_t
 */
imu_status_t IMU_ProcessData(float dt);

/**
 * @brief Get processed IMU data
 * @param imu_data Pointer to store IMU data
 * @retval imu_status_t
 */
imu_status_t IMU_GetData(imu_data_t *imu_data);

/**
 * @brief Reset velocity calculations
 * @retval imu_status_t
 */
imu_status_t IMU_ResetVelocity(void);

/**
 * @brief Check if IMU is ready for data reading
 * @retval uint8_t 1 if ready, 0 otherwise
 */
uint8_t IMU_IsDataReady(void);

/**
 * @brief Get current vehicle direction based on gyroscope
 * @retval vehicle_direction_t
 */
vehicle_direction_t IMU_GetDirection(void);

/**
 * @brief Get current speed in m/s
 * @retval float Speed in m/s
 */
float IMU_GetSpeed(void);

/**
 * @brief Get current heading angle in degrees (0-360)
 * @retval float Heading angle in degrees
 */
float IMU_GetHeading(void);

/**
 * @brief Self-test function
 * @retval imu_status_t
 */
imu_status_t IMU_SelfTest(void);

/* Private functions (sensor specific) ---------------------------------------*/

/**
 * @brief MPU6050 specific initialization
 * @retval imu_status_t
 */
imu_status_t IMU_MPU6050_Init(void);

/**
 * @brief MPU6050 read data
 * @param ax, ay, az, gx, gy, gz Output data
 * @retval imu_status_t
 */
imu_status_t IMU_MPU6050_ReadData(float *ax, float *ay, float *az, float *gx, float *gy, float *gz);

/**
 * @brief LSM6DS3 specific initialization
 * @retval imu_status_t
 */
imu_status_t IMU_LSM6DS3_Init(void);

/**
 * @brief LSM6DS3 read data
 * @param ax, ay, az, gx, gy, gz Output data
 * @retval imu_status_t
 */
imu_status_t IMU_LSM6DS3_ReadData(float *ax, float *ay, float *az, float *gx, float *gy, float *gz);

/**
 * @brief BNO055 specific initialization
 * @retval imu_status_t
 */
imu_status_t IMU_BNO055_Init(void);

/**
 * @brief BNO055 read data
 * @param ax, ay, az, gx, gy, gz Output data
 * @retval imu_status_t
 */
imu_status_t IMU_BNO055_ReadData(float *ax, float *ay, float *az, float *gx, float *gy, float *gz);

/**
 * @brief BNO055 read quaternion data (sensor fusion output)
 * @param w, x, y, z Quaternion components
 * @retval imu_status_t
 */
imu_status_t IMU_BNO055_ReadQuaternion(float *w, float *x, float *y, float *z);

/**
 * @brief BNO055 read Euler angles (sensor fusion output)
 * @param heading, roll, pitch Euler angles in degrees
 * @retval imu_status_t
 */
imu_status_t IMU_BNO055_ReadEuler(float *heading, float *roll, float *pitch);

/* Utility functions for BNO055 ------------------------------------------------*/

/**
 * @brief Initialize BNO055 with default configuration
 * @param hi2c I2C handle
 * @retval imu_status_t
 * 
 * @note Example usage:
 * @code
 * // Initialize BNO055 with default settings
 * if (IMU_BNO055_InitDefault(&hi2c1) == IMU_STATUS_OK) {
 *     // Check calibration status
 *     uint8_t sys_cal, gyro_cal, accel_cal, mag_cal;
 *     IMU_BNO055_GetCalibrationStatus(&sys_cal, &gyro_cal, &accel_cal, &mag_cal);
 *     
 *     // Read sensor fusion data
 *     float heading, roll, pitch;
 *     IMU_BNO055_ReadEuler(&heading, &roll, &pitch);
 *     
 *     // Or read quaternion
 *     float w, x, y, z;
 *     IMU_BNO055_ReadQuaternion(&w, &x, &y, &z);
 * }
 * @endcode
 */
imu_status_t IMU_BNO055_InitDefault(I2C_HandleTypeDef *hi2c);

/**
 * @brief Get BNO055 calibration status
 * @param sys_cal, gyro_cal, accel_cal, mag_cal Calibration status (0-3 each)
 * @retval imu_status_t
 */
imu_status_t IMU_BNO055_GetCalibrationStatus(uint8_t *sys_cal, uint8_t *gyro_cal, uint8_t *accel_cal, uint8_t *mag_cal);

#ifdef __cplusplus
}
#endif

#endif /* INC_IMU_DRIVER_H_ */
