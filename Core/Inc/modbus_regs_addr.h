/*
 * modbus_regs_addr.h
 *
 *  Created on: Aug 29, 2025
 *      Author: tiensy
 */

#ifndef INC_MODBUS_REGS_ADDR_H_
#define INC_MODBUS_REGS_ADDR_H_

// IMU Registers
#define REG_ACCEL_X       0x0000  // Accelerometer X-axis data
#define REG_ACCEL_Y       0x0001  // Accelerometer Y-axis data
#define REG_ACCEL_Z       0x0002  // Accelerometer Z-axis data
#define REG_GYRO_X        0x0003  // Gyroscope X-axis data
#define REG_GYRO_Y        0x0004  // Gyroscope Y-axis data
#define REG_GYRO_Z        0x0005  // Gyroscope Z-axis data
#define REG_MAG_X         0x0006  // Magnetometer X-axis data
#define REG_MAG_Y         0x0007  // Magnetometer Y-axis data
#define REG_MAG_Z         0x0008  // Magnetometer Z-axis data
#define REG_IMU_STATUS    0x0009  // IMU status
#define REG_IMU_ERROR     0x000A  // IMU error

// Digital Input Registers
#define REG_DI_1          0x000B  // Digital Input 1
#define REG_DI_2          0x000C  // Digital Input 2
#define REG_DI_3          0x000D  // Digital Input 3
#define REG_DI_4          0x000E  // Digital Input 4
#define REG_DI_STATUS     0x000F  // Digital Input Status
#define REG_DI_ERROR      0x0010  // Digital Input Error

//PN532 Registers
#define REG_PN532_DATA_LOW    0x0020  // PN532 data (low word)
#define REG_PN532_DATA_HIGH   0x0021  // PN532 data (high word)
#define REG_PN532_STATUS      0x0022  // PN532 status
#define REG_PN532_ERROR       0x0023  // PN532 error
#define REG_PN532_CARD_TYPE   0x0024  // PN532 card type
#define REG_PN532_CARD_UID    0x0025  // PN532 card UID

// Config Registers
#define REG_IMU_SAMPLE_RATE_CONFIG    0x0030  // Config IMU sample rate
#define REG_DI_DEBOUNCE_TIME_CONFIG   0x0031  // Config di debounce
#define REG_NFC_READ_TIMEOUT_CONFIG   0x0032  // Config nfc read timeout
#define REG_DATA_VALID_STATE          0x0033  // Config data valid state
#define REG_FAULT_REPORTING           0x0034  // Config fault reporting

// System Registers
#define REG_DEVICE_ID         0x0100  // Device ID (Modbus slave address)
#define REG_CONFIG_BAUDRATE   0x0101  // Config baudrate (1=9600, 2=19200, 3=38400,...)
#define REG_CONFIG_PARITY     0x0102  // Config parity (0=None, 1=Even, 2=Odd)
#define REG_CONFIG_STOP_BITS  0x0103  // Config stop bits (1=1, 2=2)
#define REG_MODULE_TYPE       0x0104  // Module type (0x0002 = Power Module)
#define REG_FIRMWARE_VERSION  0x0105  // Firmware version (e.g. 0x0101 = v1.01)
#define REG_HARDWARE_VERSION  0x0106  // Hardware version (e.g. 0x0101 = v1.01)
#define REG_SYSTEM_STATUS     0x0107  // System status (bit field)
#define REG_SYSTEM_ERROR      0x0108  // System error (global error code)
#define REG_RESET_ERROR_CMD   0x0109  // Reset error command (write 1 to reset all error flags)



#endif /* INC_MODBUS_REGS_ADDR_H_ */
