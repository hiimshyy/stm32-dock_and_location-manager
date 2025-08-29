/*
 * imu_driver.c
 *
 *  Created on: Aug 26, 2025
 *      Author: tiensy
 */

#include "imu_driver.h"
#include <math.h>
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
typedef struct {
    float ax_offset, ay_offset, az_offset;
    float gx_offset, gy_offset, gz_offset;
    float ax_filtered, ay_filtered, az_filtered;
    float gx_filtered, gy_filtered, gz_filtered;
    float velocity_x, velocity_y, velocity_z;
    float heading;
    float speed;
    uint8_t is_initialized;
    uint8_t is_calibrated;
    imu_status_t status;
} imu_internal_data_t;

/* Private define ------------------------------------------------------------*/
#define IMU_CALIBRATION_DELAY_MS    10
#define IMU_MAX_INIT_ATTEMPTS       3

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static imu_internal_data_t imu_internal = {0};
imu_config_t g_imu_config = {0};

/* Private function prototypes -----------------------------------------------*/
static imu_status_t IMU_WriteRegister(uint8_t reg, uint8_t value);
static imu_status_t IMU_ReadRegister(uint8_t reg, uint8_t *value);
static imu_status_t IMU_ReadRegisters(uint8_t reg, uint8_t *data, uint8_t length);
static imu_status_t IMU_DetectSensor(void);
static void IMU_ApplyFilters(float *ax, float *ay, float *az, float *gx, float *gy, float *gz);
static void IMU_UpdateVelocity(float ax, float ay, float az, float dt);
static void IMU_UpdateHeading(float gz, float dt);

/* Exported variables --------------------------------------------------------*/

/* Public functions ----------------------------------------------------------*/

/**
 * @brief Initialize IMU sensor
 */
imu_status_t IMU_Init(imu_config_t *config)
{
    if (config == NULL || config->hi2c == NULL) {
        return IMU_STATUS_ERROR;
    }

    // Copy configuration
    memcpy(&g_imu_config, config, sizeof(imu_config_t));

    // Reset internal data
    memset(&imu_internal, 0, sizeof(imu_internal_data_t));

    // Detect sensor type
    imu_status_t status = IMU_DetectSensor();
    if (status != IMU_STATUS_OK) {
        return status;
    }

    // Initialize sensor-specific configuration
    switch (g_imu_config.sensor_type) {
        case IMU_TYPE_MPU6050:
            status = IMU_MPU6050_Init();
            break;
        case IMU_TYPE_BNO055:
            status = IMU_BNO055_Init();
            break;
        case IMU_TYPE_LSM6DS3:
            status = IMU_LSM6DS3_Init();
            break;
        default:
            return IMU_STATUS_ERROR;
    }

    if (status == IMU_STATUS_OK) {
        imu_internal.is_initialized = 1;
        imu_internal.status = IMU_STATUS_OK;
    }

    return status;
}

/**
 * @brief Deinitialize IMU sensor
 */
imu_status_t IMU_DeInit(void)
{
    memset(&imu_internal, 0, sizeof(imu_internal_data_t));
    memset(&g_imu_config, 0, sizeof(imu_config_t));
    return IMU_STATUS_OK;
}

/**
 * @brief Calibrate IMU sensor
 */
imu_status_t IMU_Calibrate(void)
{
    if (!imu_internal.is_initialized) {
        return IMU_STATUS_NOT_INITIALIZED;
    }

    imu_internal.status = IMU_STATUS_CALIBRATING;

    float ax_sum = 0, ay_sum = 0, az_sum = 0;
    float gx_sum = 0, gy_sum = 0, gz_sum = 0;
    float ax, ay, az, gx, gy, gz;

    uint16_t samples = g_imu_config.calibration_samples;
    if (samples == 0) samples = 1000; // Default samples

    for (uint16_t i = 0; i < samples; i++) {
        if (IMU_ReadRaw(&ax, &ay, &az, &gx, &gy, &gz) == IMU_STATUS_OK) {
            ax_sum += ax;
            ay_sum += ay;
            az_sum += az;
            gx_sum += gx;
            gy_sum += gy;
            gz_sum += gz;
        }
        HAL_Delay(IMU_CALIBRATION_DELAY_MS);
    }

    // Calculate offsets
    imu_internal.ax_offset = ax_sum / samples;
    imu_internal.ay_offset = ay_sum / samples;
    imu_internal.az_offset = (az_sum / samples) - IMU_GRAVITY; // Remove gravity
    imu_internal.gx_offset = gx_sum / samples;
    imu_internal.gy_offset = gy_sum / samples;
    imu_internal.gz_offset = gz_sum / samples;

    imu_internal.is_calibrated = 1;
    imu_internal.status = IMU_STATUS_OK;

    return IMU_STATUS_OK;
}

/**
 * @brief Read raw sensor data
 */
imu_status_t IMU_ReadRaw(float *ax, float *ay, float *az, float *gx, float *gy, float *gz)
{
    if (!imu_internal.is_initialized) {
        return IMU_STATUS_NOT_INITIALIZED;
    }

    imu_status_t status;

    switch (g_imu_config.sensor_type) {
        case IMU_TYPE_MPU6050:
            status = IMU_MPU6050_ReadData(ax, ay, az, gx, gy, gz);
            break;
        case IMU_TYPE_BNO055:
            status = IMU_BNO055_ReadData(ax, ay, az, gx, gy, gz);
            break;
        case IMU_TYPE_LSM6DS3:
            status = IMU_LSM6DS3_ReadData(ax, ay, az, gx, gy, gz);
            break;
        default:
            return IMU_STATUS_ERROR;
    }

    return status;
}

/**
 * @brief Process IMU data and update calculations
 */
imu_status_t IMU_ProcessData(float dt)
{
    if (!imu_internal.is_initialized) {
        return IMU_STATUS_NOT_INITIALIZED;
    }

    float ax, ay, az, gx, gy, gz;

    if (IMU_ReadRaw(&ax, &ay, &az, &gx, &gy, &gz) != IMU_STATUS_OK) {
        return IMU_STATUS_ERROR;
    }

    // Apply calibration offsets
    if (imu_internal.is_calibrated) {
        ax -= imu_internal.ax_offset;
        ay -= imu_internal.ay_offset;
        az -= imu_internal.az_offset;
        gx -= imu_internal.gx_offset;
        gy -= imu_internal.gy_offset;
        gz -= imu_internal.gz_offset;
    }

    // Apply filters
    IMU_ApplyFilters(&ax, &ay, &az, &gx, &gy, &gz);

    // Update velocity and heading
    IMU_UpdateVelocity(ax, ay, az, dt);
    IMU_UpdateHeading(gz, dt);

    return IMU_STATUS_OK;
}

/**
 * @brief Get processed IMU data
 */
imu_status_t IMU_GetData(imu_data_t *imu_data)
{
    if (!imu_internal.is_initialized || imu_data == NULL) {
        return IMU_STATUS_ERROR;
    }

    imu_data->accel_x = imu_internal.ax_filtered;
    imu_data->accel_y = imu_internal.ay_filtered;
    imu_data->accel_z = imu_internal.az_filtered;
    imu_data->gyro_x = imu_internal.gx_filtered;
    imu_data->gyro_y = imu_internal.gy_filtered;
    imu_data->gyro_z = imu_internal.gz_filtered;
    imu_data->velocity_x = imu_internal.velocity_x;
    imu_data->velocity_y = imu_internal.velocity_y;
    imu_data->velocity_z = imu_internal.velocity_z;
    imu_data->heading = imu_internal.heading;
    imu_data->speed = imu_internal.speed;

    return IMU_STATUS_OK;
}

/**
 * @brief Reset velocity calculations
 */
imu_status_t IMU_ResetVelocity(void)
{
    imu_internal.velocity_x = 0;
    imu_internal.velocity_y = 0;
    imu_internal.velocity_z = 0;
    imu_internal.speed = 0;
    return IMU_STATUS_OK;
}

/**
 * @brief Check if IMU is ready for data reading
 */
uint8_t IMU_IsDataReady(void)
{
    if (!imu_internal.is_initialized) {
        return 0;
    }

    uint8_t status_reg;
    if (IMU_ReadRegister(0x3A, &status_reg) == IMU_STATUS_OK) {
        return (status_reg & 0x01); // Data ready bit
    }
    return 0;
}

/**
 * @brief Get current vehicle direction
 */
vehicle_direction_t IMU_GetDirection(void)
{
    if (!imu_internal.is_initialized) {
        return DIRECTION_UNKNOWN;
    }

    float gz = imu_internal.gz_filtered;

    if (fabsf(gz) < IMU_TURN_THRESHOLD) {
        return DIRECTION_STRAIGHT;
    } else if (gz > IMU_TURN_THRESHOLD) {
        return DIRECTION_TURN_LEFT;
    } else {
        return DIRECTION_TURN_RIGHT;
    }
}

/**
 * @brief Get current speed
 */
float IMU_GetSpeed(void)
{
    return imu_internal.speed;
}

/**
 * @brief Get current heading angle
 */
float IMU_GetHeading(void)
{
    return imu_internal.heading;
}

/**
 * @brief Self-test function
 */
imu_status_t IMU_SelfTest(void)
{
    if (!imu_internal.is_initialized) {
        return IMU_STATUS_NOT_INITIALIZED;
    }

    // Read WHO_AM_I register to verify communication
    uint8_t who_am_i;
    uint8_t expected_value;
    uint8_t who_am_i_reg;

    switch (g_imu_config.sensor_type) {
        case IMU_TYPE_MPU6050:
            who_am_i_reg = MPU6050_WHO_AM_I_REG;
            expected_value = MPU6050_WHO_AM_I_VALUE;
            break;
        case IMU_TYPE_BNO055:
            who_am_i_reg = BNO055_CHIP_ID_REG;
            expected_value = BNO055_CHIP_ID_VALUE;
            break;
        case IMU_TYPE_LSM6DS3:
            who_am_i_reg = LSM6DS3_WHO_AM_I_REG;
            expected_value = LSM6DS3_WHO_AM_I_VALUE;
            break;
        default:
            return IMU_STATUS_ERROR;
    }

    if (IMU_ReadRegister(who_am_i_reg, &who_am_i) != IMU_STATUS_OK) {
        return IMU_STATUS_ERROR;
    }

    return (who_am_i == expected_value) ? IMU_STATUS_OK : IMU_STATUS_ERROR;
}

/* MPU6050 specific functions ------------------------------------------------*/

/**
 * @brief MPU6050 specific initialization
 */
imu_status_t IMU_MPU6050_Init(void)
{
    // Reset MPU6050
    if (IMU_WriteRegister(0x6B, 0x80) != IMU_STATUS_OK) {
        return IMU_STATUS_ERROR;
    }
    HAL_Delay(100);

    // Wake up MPU6050
    if (IMU_WriteRegister(0x6B, 0x00) != IMU_STATUS_OK) {
        return IMU_STATUS_ERROR;
    }

    // Configure accelerometer range (±2g = 0, ±4g = 1, ±8g = 2, ±16g = 3)
    if (IMU_WriteRegister(0x1C, g_imu_config.accel_range << 3) != IMU_STATUS_OK) {
        return IMU_STATUS_ERROR;
    }

    // Configure gyroscope range (±250°/s = 0, ±500°/s = 1, ±1000°/s = 2, ±2000°/s = 3)
    if (IMU_WriteRegister(0x1B, g_imu_config.gyro_range << 3) != IMU_STATUS_OK) {
        return IMU_STATUS_ERROR;
    }

    // Configure DLPF if enabled
    if (g_imu_config.dlpf_enable) {
        if (IMU_WriteRegister(0x1A, g_imu_config.dlpf_config) != IMU_STATUS_OK) {
            return IMU_STATUS_ERROR;
        }
    }

    return IMU_STATUS_OK;
}

/**
 * @brief MPU6050 read data
 */
imu_status_t IMU_MPU6050_ReadData(float *ax, float *ay, float *az, float *gx, float *gy, float *gz)
{
    uint8_t raw_data[14];

    if (IMU_ReadRegisters(0x3B, raw_data, 14) != IMU_STATUS_OK) {
        return IMU_STATUS_ERROR;
    }

    // Convert raw data to signed 16-bit values
    int16_t ax_raw = (int16_t)((raw_data[0] << 8) | raw_data[1]);
    int16_t ay_raw = (int16_t)((raw_data[2] << 8) | raw_data[3]);
    int16_t az_raw = (int16_t)((raw_data[4] << 8) | raw_data[5]);
    int16_t gx_raw = (int16_t)((raw_data[8] << 8) | raw_data[9]);
    int16_t gy_raw = (int16_t)((raw_data[10] << 8) | raw_data[11]);
    int16_t gz_raw = (int16_t)((raw_data[12] << 8) | raw_data[13]);

    // Convert to physical units
    float accel_scale = 2.0f / 32768.0f * (1 << g_imu_config.accel_range) * IMU_GRAVITY;
    float gyro_scale = 250.0f / 32768.0f * (1 << g_imu_config.gyro_range);

    *ax = ax_raw * accel_scale;
    *ay = ay_raw * accel_scale;
    *az = az_raw * accel_scale;
    *gx = gx_raw * gyro_scale;
    *gy = gy_raw * gyro_scale;
    *gz = gz_raw * gyro_scale;

    return IMU_STATUS_OK;
}

/* BNO055 specific functions ------------------------------------------------*/

/**
 * @brief BNO055 specific initialization
 */
imu_status_t IMU_BNO055_Init(void)
{
    // Reset BNO055
    if (IMU_WriteRegister(0x3F, 0x20) != IMU_STATUS_OK) {
        return IMU_STATUS_ERROR;
    }
    HAL_Delay(650); // BNO055 needs 650ms to boot after reset

    // Check chip ID
    uint8_t chip_id;
    if (IMU_ReadRegister(BNO055_CHIP_ID_REG, &chip_id) != IMU_STATUS_OK) {
        return IMU_STATUS_ERROR;
    }
    if (chip_id != BNO055_CHIP_ID_VALUE) {
        return IMU_STATUS_ERROR;
    }

    // Set to config mode
    if (IMU_WriteRegister(0x3D, 0x00) != IMU_STATUS_OK) {
        return IMU_STATUS_ERROR;
    }
    HAL_Delay(25);

    // Set power mode to normal
    if (IMU_WriteRegister(0x3E, 0x00) != IMU_STATUS_OK) {
        return IMU_STATUS_ERROR;
    }
    HAL_Delay(10);

    // Set page 0
    if (IMU_WriteRegister(0x07, 0x00) != IMU_STATUS_OK) {
        return IMU_STATUS_ERROR;
    }

    // Set external crystal
    if (IMU_WriteRegister(0x3F, 0x80) != IMU_STATUS_OK) {
        return IMU_STATUS_ERROR;
    }
    HAL_Delay(10);

    // Set operation mode to NDOF (Nine Degrees of Freedom)
    // This enables sensor fusion
    if (IMU_WriteRegister(0x3D, 0x0C) != IMU_STATUS_OK) {
        return IMU_STATUS_ERROR;
    }
    HAL_Delay(20);

    return IMU_STATUS_OK;
}

/**
 * @brief BNO055 read raw accelerometer and gyroscope data
 */
imu_status_t IMU_BNO055_ReadData(float *ax, float *ay, float *az, float *gx, float *gy, float *gz)
{
    uint8_t accel_data[6], gyro_data[6];

    // Read accelerometer data (registers 0x08-0x0D)
    if (IMU_ReadRegisters(0x08, accel_data, 6) != IMU_STATUS_OK) {
        return IMU_STATUS_ERROR;
    }

    // Read gyroscope data (registers 0x14-0x19)
    if (IMU_ReadRegisters(0x14, gyro_data, 6) != IMU_STATUS_OK) {
        return IMU_STATUS_ERROR;
    }

    // Convert raw data to signed 16-bit values
    int16_t ax_raw = (int16_t)((accel_data[1] << 8) | accel_data[0]);
    int16_t ay_raw = (int16_t)((accel_data[3] << 8) | accel_data[2]);
    int16_t az_raw = (int16_t)((accel_data[5] << 8) | accel_data[4]);
    int16_t gx_raw = (int16_t)((gyro_data[1] << 8) | gyro_data[0]);
    int16_t gy_raw = (int16_t)((gyro_data[3] << 8) | gyro_data[2]);
    int16_t gz_raw = (int16_t)((gyro_data[5] << 8) | gyro_data[4]);

    // Convert to physical units
    // BNO055 accelerometer: 1 m/s² = 100 LSB
    // BNO055 gyroscope: 1 dps = 16 LSB
    *ax = ax_raw / 100.0f;
    *ay = ay_raw / 100.0f;
    *az = az_raw / 100.0f;
    *gx = gx_raw / 16.0f;
    *gy = gy_raw / 16.0f;
    *gz = gz_raw / 16.0f;

    return IMU_STATUS_OK;
}

/**
 * @brief BNO055 read quaternion data (sensor fusion output)
 */
imu_status_t IMU_BNO055_ReadQuaternion(float *w, float *x, float *y, float *z)
{
    uint8_t quat_data[8];

    // Read quaternion data (registers 0x20-0x27)
    if (IMU_ReadRegisters(0x20, quat_data, 8) != IMU_STATUS_OK) {
        return IMU_STATUS_ERROR;
    }

    // Convert to signed 16-bit values
    int16_t w_raw = (int16_t)((quat_data[1] << 8) | quat_data[0]);
    int16_t x_raw = (int16_t)((quat_data[3] << 8) | quat_data[2]);
    int16_t y_raw = (int16_t)((quat_data[5] << 8) | quat_data[4]);
    int16_t z_raw = (int16_t)((quat_data[7] << 8) | quat_data[6]);

    // Convert to normalized quaternion
    // Scale factor is 2^14 = 16384
    *w = w_raw / 16384.0f;
    *x = x_raw / 16384.0f;
    *y = y_raw / 16384.0f;
    *z = z_raw / 16384.0f;

    return IMU_STATUS_OK;
}

/**
 * @brief BNO055 read Euler angles (sensor fusion output)
 */
imu_status_t IMU_BNO055_ReadEuler(float *heading, float *roll, float *pitch)
{
    uint8_t euler_data[6];

    // Read Euler angles (registers 0x1A-0x1F)
    if (IMU_ReadRegisters(0x1A, euler_data, 6) != IMU_STATUS_OK) {
        return IMU_STATUS_ERROR;
    }

    // Convert to signed 16-bit values
    int16_t heading_raw = (int16_t)((euler_data[1] << 8) | euler_data[0]);
    int16_t roll_raw = (int16_t)((euler_data[3] << 8) | euler_data[2]);
    int16_t pitch_raw = (int16_t)((euler_data[5] << 8) | euler_data[4]);

    // Convert to degrees
    // Scale factor is 16 LSB per degree
    *heading = heading_raw / 16.0f;
    *roll = roll_raw / 16.0f;
    *pitch = pitch_raw / 16.0f;

    return IMU_STATUS_OK;
}

/* LSM6DS3 specific functions ------------------------------------------------*/

/**
 * @brief LSM6DS3 specific initialization
 */
imu_status_t IMU_LSM6DS3_Init(void)
{
    // Configure accelerometer (ODR = 104 Hz, FS = ±2g)
    uint8_t accel_config = (0x04 << 4) | (g_imu_config.accel_range << 2);
    if (IMU_WriteRegister(0x10, accel_config) != IMU_STATUS_OK) {
        return IMU_STATUS_ERROR;
    }

    // Configure gyroscope (ODR = 104 Hz, FS = ±250 dps)
    uint8_t gyro_config = (0x04 << 4) | (g_imu_config.gyro_range << 2);
    if (IMU_WriteRegister(0x11, gyro_config) != IMU_STATUS_OK) {
        return IMU_STATUS_ERROR;
    }

    return IMU_STATUS_OK;
}

/**
 * @brief LSM6DS3 read data
 */
imu_status_t IMU_LSM6DS3_ReadData(float *ax, float *ay, float *az, float *gx, float *gy, float *gz)
{
    uint8_t accel_data[6], gyro_data[6];

    // Read accelerometer data
    if (IMU_ReadRegisters(0x28, accel_data, 6) != IMU_STATUS_OK) {
        return IMU_STATUS_ERROR;
    }

    // Read gyroscope data
    if (IMU_ReadRegisters(0x22, gyro_data, 6) != IMU_STATUS_OK) {
        return IMU_STATUS_ERROR;
    }

    // Convert raw data
    int16_t ax_raw = (int16_t)((accel_data[1] << 8) | accel_data[0]);
    int16_t ay_raw = (int16_t)((accel_data[3] << 8) | accel_data[2]);
    int16_t az_raw = (int16_t)((accel_data[5] << 8) | accel_data[4]);
    int16_t gx_raw = (int16_t)((gyro_data[1] << 8) | gyro_data[0]);
    int16_t gy_raw = (int16_t)((gyro_data[3] << 8) | gyro_data[2]);
    int16_t gz_raw = (int16_t)((gyro_data[5] << 8) | gyro_data[4]);

    // Convert to physical units
    float accel_scale = 0.061e-3f * (1 << g_imu_config.accel_range) * IMU_GRAVITY;
    float gyro_scale = 8.75e-3f * (1 << g_imu_config.gyro_range);

    *ax = ax_raw * accel_scale;
    *ay = ay_raw * accel_scale;
    *az = az_raw * accel_scale;
    *gx = gx_raw * gyro_scale;
    *gy = gy_raw * gyro_scale;
    *gz = gz_raw * gyro_scale;

    return IMU_STATUS_OK;
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief Write register via I2C
 */
static imu_status_t IMU_WriteRegister(uint8_t reg, uint8_t value)
{
    HAL_StatusTypeDef hal_status = HAL_I2C_Mem_Write(
        g_imu_config.hi2c,
        g_imu_config.device_address << 1,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        &value,
        1,
        1000
    );

    return (hal_status == HAL_OK) ? IMU_STATUS_OK : IMU_STATUS_ERROR;
}

/**
 * @brief Read single register via I2C
 */
static imu_status_t IMU_ReadRegister(uint8_t reg, uint8_t *value)
{
    HAL_StatusTypeDef hal_status = HAL_I2C_Mem_Read(
        g_imu_config.hi2c,
        g_imu_config.device_address << 1,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        value,
        1,
        1000
    );

    return (hal_status == HAL_OK) ? IMU_STATUS_OK : IMU_STATUS_ERROR;
}

/**
 * @brief Read multiple registers via I2C
 */
static imu_status_t IMU_ReadRegisters(uint8_t reg, uint8_t *data, uint8_t length)
{
    HAL_StatusTypeDef hal_status = HAL_I2C_Mem_Read(
        g_imu_config.hi2c,
        g_imu_config.device_address << 1,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        data,
        length,
        1000
    );

    return (hal_status == HAL_OK) ? IMU_STATUS_OK : IMU_STATUS_ERROR;
}

/**
 * @brief Detect IMU sensor type
 */
static imu_status_t IMU_DetectSensor(void)
{
    uint8_t who_am_i;

    // Try BNO055 first (most common address)
    g_imu_config.device_address = BNO055_ADDRESS_A;
    if (IMU_ReadRegister(BNO055_CHIP_ID_REG, &who_am_i) == IMU_STATUS_OK) {
        if (who_am_i == BNO055_CHIP_ID_VALUE) {
            g_imu_config.sensor_type = IMU_TYPE_BNO055;
            return IMU_STATUS_OK;
        }
    }

    // Try BNO055 alternative address
    g_imu_config.device_address = BNO055_ADDRESS_B;
    if (IMU_ReadRegister(BNO055_CHIP_ID_REG, &who_am_i) == IMU_STATUS_OK) {
        if (who_am_i == BNO055_CHIP_ID_VALUE) {
            g_imu_config.sensor_type = IMU_TYPE_BNO055;
            return IMU_STATUS_OK;
        }
    }

    // Try MPU6050
    g_imu_config.device_address = MPU6050_ADDRESS;
    if (IMU_ReadRegister(MPU6050_WHO_AM_I_REG, &who_am_i) == IMU_STATUS_OK) {
        if (who_am_i == MPU6050_WHO_AM_I_VALUE) {
            g_imu_config.sensor_type = IMU_TYPE_MPU6050;
            return IMU_STATUS_OK;
        }
    }

    // Try LSM6DS3
    g_imu_config.device_address = LSM6DS3_ADDRESS;
    if (IMU_ReadRegister(LSM6DS3_WHO_AM_I_REG, &who_am_i) == IMU_STATUS_OK) {
        if (who_am_i == LSM6DS3_WHO_AM_I_VALUE) {
            g_imu_config.sensor_type = IMU_TYPE_LSM6DS3;
            return IMU_STATUS_OK;
        }
    }

    g_imu_config.sensor_type = IMU_TYPE_UNKNOWN;
    return IMU_STATUS_ERROR;
}

/**
 * @brief Apply low-pass filters
 */
static void IMU_ApplyFilters(float *ax, float *ay, float *az, float *gx, float *gy, float *gz)
{
    // Apply low-pass filter to accelerometer
    imu_internal.ax_filtered = IMU_ACCEL_FILTER_ALPHA * imu_internal.ax_filtered +
                               (1.0f - IMU_ACCEL_FILTER_ALPHA) * (*ax);
    imu_internal.ay_filtered = IMU_ACCEL_FILTER_ALPHA * imu_internal.ay_filtered +
                               (1.0f - IMU_ACCEL_FILTER_ALPHA) * (*ay);
    imu_internal.az_filtered = IMU_ACCEL_FILTER_ALPHA * imu_internal.az_filtered +
                               (1.0f - IMU_ACCEL_FILTER_ALPHA) * (*az);

    // Apply low-pass filter to gyroscope
    imu_internal.gx_filtered = IMU_GYRO_FILTER_ALPHA * imu_internal.gx_filtered +
                               (1.0f - IMU_GYRO_FILTER_ALPHA) * (*gx);
    imu_internal.gy_filtered = IMU_GYRO_FILTER_ALPHA * imu_internal.gy_filtered +
                               (1.0f - IMU_GYRO_FILTER_ALPHA) * (*gy);
    imu_internal.gz_filtered = IMU_GYRO_FILTER_ALPHA * imu_internal.gz_filtered +
                               (1.0f - IMU_GYRO_FILTER_ALPHA) * (*gz);

    *ax = imu_internal.ax_filtered;
    *ay = imu_internal.ay_filtered;
    *az = imu_internal.az_filtered;
    *gx = imu_internal.gx_filtered;
    *gy = imu_internal.gy_filtered;
    *gz = imu_internal.gz_filtered;
}

/**
 * @brief Update velocity estimation
 */
static void IMU_UpdateVelocity(float ax, float ay, float az, float dt)
{
    // Simple velocity integration with decay
    if (fabsf(ax) > IMU_MOTION_THRESHOLD) {
        imu_internal.velocity_x += ax * dt;
    }
    if (fabsf(ay) > IMU_MOTION_THRESHOLD) {
        imu_internal.velocity_y += ay * dt;
    }
    if (fabsf(az) > IMU_MOTION_THRESHOLD) {
        imu_internal.velocity_z += az * dt;
    }

    // Apply velocity decay to prevent drift
    imu_internal.velocity_x *= IMU_VELOCITY_DECAY;
    imu_internal.velocity_y *= IMU_VELOCITY_DECAY;
    imu_internal.velocity_z *= IMU_VELOCITY_DECAY;

    // Calculate speed
    imu_internal.speed = sqrtf(imu_internal.velocity_x * imu_internal.velocity_x +
                               imu_internal.velocity_y * imu_internal.velocity_y);
}

/**
 * @brief Update heading estimation
 */
static void IMU_UpdateHeading(float gz, float dt)
{
    // Integrate gyroscope Z-axis to get heading
    imu_internal.heading += gz * dt;

    // Keep heading in 0-360 range
    while (imu_internal.heading < 0) {
        imu_internal.heading += 360.0f;
    }
    while (imu_internal.heading >= 360.0f) {
        imu_internal.heading -= 360.0f;
    }
}

/* BNO055 Utility Functions ------------------------------------------------*/

/**
 * @brief Initialize BNO055 with default configuration
 */
imu_status_t IMU_BNO055_InitDefault(I2C_HandleTypeDef *hi2c)
{
    // Setup default configuration
    imu_config_t config = {
        .sensor_type = IMU_TYPE_BNO055,
        .hi2c = hi2c,
        .device_address = BNO055_ADDRESS_A,  // Will be auto-detected
        .accel_range = 0,       // ±2g
        .gyro_range = 0,        // ±250°/s
        .sample_rate = 100,     // 100 Hz
        .dlpf_enable = 0,       // Not used for BNO055
        .dlpf_config = 0,       // Not used for BNO055
        .calibration_samples = 1000
    };

    return IMU_Init(&config);
}

/**
 * @brief Get BNO055 calibration status
 */
imu_status_t IMU_BNO055_GetCalibrationStatus(uint8_t *sys_cal, uint8_t *gyro_cal, uint8_t *accel_cal, uint8_t *mag_cal)
{
    if (!imu_internal.is_initialized || g_imu_config.sensor_type != IMU_TYPE_BNO055) {
        return IMU_STATUS_ERROR;
    }

    uint8_t cal_stat;
    if (IMU_ReadRegister(0x35, &cal_stat) != IMU_STATUS_OK) {
        return IMU_STATUS_ERROR;
    }

    // Extract calibration status for each sensor
    *sys_cal = (cal_stat >> 6) & 0x03;      // Bits 7:6
    *gyro_cal = (cal_stat >> 4) & 0x03;     // Bits 5:4
    *accel_cal = (cal_stat >> 2) & 0x03;    // Bits 3:2
    *mag_cal = cal_stat & 0x03;             // Bits 1:0

    return IMU_STATUS_OK;
}
