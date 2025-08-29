/*
 * nfc_driver.h
 *
 *  Created on: Aug 26, 2025
 *      Author: tiensy
 */

#ifndef INC_NFC_DRIVER_H_
#define INC_NFC_DRIVER_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "main.h"
#include "shared_data.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief NFC communication interface
 */
typedef enum {
    NFC_INTERFACE_SPI = 0,
    NFC_INTERFACE_I2C,
    NFC_INTERFACE_UART
} nfc_interface_t;

/**
 * @brief NFC card types
 */
typedef enum {
    NFC_CARD_TYPE_106_A = 0,    ///< ISO14443 Type A at 106 kbps
    NFC_CARD_TYPE_106_B,        ///< ISO14443 Type B at 106 kbps
    NFC_CARD_TYPE_212_F,        ///< FeliCa at 212 kbps
    NFC_CARD_TYPE_424_F,        ///< FeliCa at 424 kbps
    NFC_CARD_TYPE_UNKNOWN
} nfc_card_type_t;

/**
 * @brief NFC configuration structure
 */
typedef struct {
    nfc_interface_t interface;
    void *handle;               ///< SPI/I2C/UART handle

    // GPIO pins
    GPIO_TypeDef *rst_port;     ///< Reset pin port
    uint16_t rst_pin;           ///< Reset pin number
    GPIO_TypeDef *irq_port;     ///< IRQ pin port (optional)
    uint16_t irq_pin;           ///< IRQ pin number (optional)

    // SPI specific (if using SPI)
    GPIO_TypeDef *cs_port;      ///< Chip select port
    uint16_t cs_pin;            ///< Chip select pin

    // Configuration
    uint8_t use_irq;            ///< Use IRQ pin for data ready
    uint32_t timeout_ms;        ///< Communication timeout
    uint8_t max_retries;        ///< Max retry attempts
} nfc_config_t;

/**
 * @brief NFC card information
 */
typedef struct {
    nfc_card_type_t type;
    uint8_t uid_length;         ///< UID length (4, 7, or 10 bytes)
    uint8_t uid[10];            ///< Card UID
    uint8_t sak;                ///< Select acknowledge
    uint8_t atqa[2];            ///< Answer to request type A
    char type_string[16];       ///< Human readable card type
    uint32_t uid_32;            ///< UID as 32-bit integer (for compatibility)
} nfc_card_info_t;

/**
 * @brief NFC status enumeration
 */
typedef enum {
    NFC_STATUS_OK = 0,
    NFC_STATUS_ERROR,
    NFC_STATUS_TIMEOUT,
    NFC_STATUS_NO_CARD,
    NFC_STATUS_CARD_REMOVED,
    NFC_STATUS_COMM_ERROR,
    NFC_STATUS_NOT_INITIALIZED,
    NFC_STATUS_BUSY
} nfc_status_t;

/* Exported constants --------------------------------------------------------*/

// PN532 Commands
#define PN532_COMMAND_GETFIRMWAREVERSION    0x02
#define PN532_COMMAND_SAMCONFIGURATION      0x14
#define PN532_COMMAND_INLISTPASSIVETARGET   0x4A
#define PN532_COMMAND_INDATAEXCHANGE        0x40
#define PN532_COMMAND_INDESELECT            0x44
#define PN532_COMMAND_INRELEASE             0x52

// PN532 Response codes
#define PN532_ACK_FRAME                     {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00}
#define PN532_NACK_FRAME                    {0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00}

// SPI specific
#define PN532_SPI_DATAWRITE                 0x01
#define PN532_SPI_STATUSREAD                0x02
#define PN532_SPI_DATAREAD                  0x03

// I2C specific
#define PN532_I2C_ADDRESS                   0x48
#define PN532_I2C_READY                     0x01

// Timeouts
#define NFC_DEFAULT_TIMEOUT                 1000    ///< Default timeout in ms
#define NFC_CARD_DETECT_TIMEOUT            100     ///< Card detection timeout
#define NFC_MAX_RETRIES                     3       ///< Maximum retry attempts

// Buffer sizes
#define NFC_BUFFER_SIZE                     64      ///< Communication buffer size
#define NFC_UID_MAX_LENGTH                  10      ///< Maximum UID length

/* Exported variables --------------------------------------------------------*/
extern nfc_config_t g_nfc_config;
extern nfc_card_info_t g_current_card;

/* Exported functions --------------------------------------------------------*/

/**
 * @brief Initialize PN532 NFC module
 * @param config Pointer to NFC configuration
 * @retval nfc_status_t
 */
nfc_status_t NFC_Init(nfc_config_t *config);

/**
 * @brief Deinitialize PN532 NFC module
 * @retval nfc_status_t
 */
nfc_status_t NFC_DeInit(void);

/**
 * @brief Reset PN532 module
 * @retval nfc_status_t
 */
nfc_status_t NFC_Reset(void);

/**
 * @brief Get firmware version
 * @param version Pointer to store version info
 * @retval nfc_status_t
 */
nfc_status_t NFC_GetFirmwareVersion(uint32_t *version);

/**
 * @brief Configure PN532 for passive target detection
 * @retval nfc_status_t
 */
nfc_status_t NFC_ConfigureMode(void);

/**
 * @brief Scan for NFC/RFID cards
 * @param card_info Pointer to store detected card info
 * @retval nfc_status_t
 */
nfc_status_t NFC_ScanCard(nfc_card_info_t *card_info);

/**
 * @brief Read card UID only (simplified function)
 * @param uid Pointer to store 32-bit UID
 * @retval nfc_status_t
 */
nfc_status_t NFC_ReadCardUID(uint32_t *uid);

/**
 * @brief Check if card is still present
 * @retval nfc_status_t
 */
nfc_status_t NFC_IsCardPresent(void);

/**
 * @brief Release currently selected card
 * @retval nfc_status_t
 */
nfc_status_t NFC_ReleaseCard(void);

/**
 * @brief Read data from MIFARE Classic card
 * @param block_number Block number to read
 * @param data Pointer to store read data (16 bytes)
 * @retval nfc_status_t
 */
nfc_status_t NFC_ReadMifareBlock(uint8_t block_number, uint8_t *data);

/**
 * @brief Write data to MIFARE Classic card
 * @param block_number Block number to write
 * @param data Pointer to data to write (16 bytes)
 * @retval nfc_status_t
 */
nfc_status_t NFC_WriteMifareBlock(uint8_t block_number, uint8_t *data);

/**
 * @brief Self-test function
 * @retval nfc_status_t
 */
nfc_status_t NFC_SelfTest(void);

/**
 * @brief Convert UID array to 32-bit integer
 * @param uid UID array
 * @param length UID length
 * @retval uint32_t 32-bit UID
 */
uint32_t NFC_UIDToUint32(uint8_t *uid, uint8_t length);

/**
 * @brief Get card type string
 * @param type Card type enum
 * @retval const char* Card type string
 */
const char* NFC_GetCardTypeString(nfc_card_type_t type);

/* Utility functions --------------------------------------------------------*/

/**
 * @brief Initialize NFC with I2C interface and default settings
 * @param hi2c I2C handle
 * @param rst_port Reset pin port (can be NULL if no reset control)
 * @param rst_pin Reset pin number
 * @retval nfc_status_t
 * 
 * @note Example usage:
 * @code
 * // Initialize NFC with I2C and reset pin
 * if (NFC_InitI2CDefault(&hi2c1, GPIOA, GPIO_PIN_0) == NFC_STATUS_OK) {
 *     // Scan for card
 *     uint32_t uid;
 *     if (NFC_ReadCardUID(&uid) == NFC_STATUS_OK) {
 *         printf("Card UID: 0x%08X\\n", uid);
 *     }
 * }
 * @endcode
 */
nfc_status_t NFC_InitI2CDefault(I2C_HandleTypeDef *hi2c, GPIO_TypeDef *rst_port, uint16_t rst_pin);

/* Private functions ---------------------------------------------------------*/

/**
 * @brief Low-level communication functions
 */
nfc_status_t NFC_SendCommand(uint8_t *cmd, uint8_t cmd_len, uint8_t *response, uint8_t *response_len);
nfc_status_t NFC_WaitReady(uint32_t timeout);
nfc_status_t NFC_ReadAck(void);

/**
 * @brief Interface specific functions
 */
nfc_status_t NFC_SPI_Init(void);
nfc_status_t NFC_SPI_WriteCommand(uint8_t *cmd, uint8_t len);
nfc_status_t NFC_SPI_ReadResponse(uint8_t *response, uint8_t *len);

nfc_status_t NFC_I2C_Init(void);
nfc_status_t NFC_I2C_WriteCommand(uint8_t *cmd, uint8_t len);
nfc_status_t NFC_I2C_ReadResponse(uint8_t *response, uint8_t *len);

#ifdef __cplusplus
}
#endif

#endif /* INC_NFC_DRIVER_H_ */
