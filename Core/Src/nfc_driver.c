/*
 * nfc_driver.c
 *
 *  Created on: Aug 26, 2025
 *      Author: tiensy
 */

#include "nfc_driver.h"
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
typedef struct {
    uint8_t is_initialized;
    uint8_t card_present;
    uint32_t last_scan_time;
    nfc_status_t status;
} nfc_internal_data_t;

/* Private define ------------------------------------------------------------*/
#define NFC_PREAMBLE                0x00
#define NFC_START_CODE1             0x00
#define NFC_START_CODE2             0xFF
#define NFC_POSTAMBLE               0x00

#define NFC_HOSTTOPN532             0xD4
#define NFC_PN532TOHOST             0xD5

#define NFC_SPI_DELAY               2
#define NFC_RESET_DELAY             400
#define NFC_WAKEUP_DELAY            20

/* Private macro -------------------------------------------------------------*/
#define NFC_CS_LOW()    HAL_GPIO_WritePin(g_nfc_config.cs_port, g_nfc_config.cs_pin, GPIO_PIN_RESET)
#define NFC_CS_HIGH()   HAL_GPIO_WritePin(g_nfc_config.cs_port, g_nfc_config.cs_pin, GPIO_PIN_SET)

/* Private variables ---------------------------------------------------------*/
static nfc_internal_data_t nfc_internal = {0};
static uint8_t nfc_buffer[NFC_BUFFER_SIZE];

/* Exported variables --------------------------------------------------------*/
nfc_config_t g_nfc_config = {0};
nfc_card_info_t g_current_card = {0};

/* Private function prototypes -----------------------------------------------*/
static nfc_status_t NFC_SendFrame(uint8_t *data, uint8_t length);
static nfc_status_t NFC_ReadFrame(uint8_t *data, uint8_t *length);
static uint8_t NFC_CalculateChecksum(uint8_t *data, uint8_t length);
static nfc_status_t NFC_WriteFrame(uint8_t *frame, uint8_t length);
static nfc_status_t NFC_ReadResponse(uint8_t *response, uint8_t *length, uint32_t timeout);
static void NFC_Delay(uint32_t ms);

/* Public functions ----------------------------------------------------------*/

/**
 * @brief Initialize PN532 NFC module
 */
nfc_status_t NFC_Init(nfc_config_t *config)
{
    if (config == NULL || config->handle == NULL) {
        return NFC_STATUS_ERROR;
    }

    // Copy configuration
    memcpy(&g_nfc_config, config, sizeof(nfc_config_t));

    // Reset internal data
    memset(&nfc_internal, 0, sizeof(nfc_internal_data_t));
    memset(&g_current_card, 0, sizeof(nfc_card_info_t));

    // Initialize interface
    nfc_status_t status;
    switch (g_nfc_config.interface) {
        case NFC_INTERFACE_SPI:
            status = NFC_SPI_Init();
            break;
        case NFC_INTERFACE_I2C:
            status = NFC_I2C_Init();
            break;
        default:
            return NFC_STATUS_ERROR;
    }

    if (status != NFC_STATUS_OK) {
        return status;
    }

    // Reset PN532
    status = NFC_Reset();
    if (status != NFC_STATUS_OK) {
        return status;
    }

    // Get firmware version to verify communication
    uint32_t version;
    status = NFC_GetFirmwareVersion(&version);
    if (status != NFC_STATUS_OK) {
        return status;
    }

    // Configure for passive target detection
    status = NFC_ConfigureMode();
    if (status != NFC_STATUS_OK) {
        return status;
    }

    nfc_internal.is_initialized = 1;
    nfc_internal.status = NFC_STATUS_OK;

    return NFC_STATUS_OK;
}

/**
 * @brief Deinitialize PN532 NFC module
 */
nfc_status_t NFC_DeInit(void)
{
    memset(&nfc_internal, 0, sizeof(nfc_internal_data_t));
    memset(&g_nfc_config, 0, sizeof(nfc_config_t));
    memset(&g_current_card, 0, sizeof(nfc_card_info_t));
    return NFC_STATUS_OK;
}

/**
 * @brief Reset PN532 module
 */
nfc_status_t NFC_Reset(void)
{
    if (g_nfc_config.rst_port != NULL) {
        // Hardware reset
        HAL_GPIO_WritePin(g_nfc_config.rst_port, g_nfc_config.rst_pin, GPIO_PIN_RESET);
        NFC_Delay(NFC_RESET_DELAY);
        HAL_GPIO_WritePin(g_nfc_config.rst_port, g_nfc_config.rst_pin, GPIO_PIN_SET);
        NFC_Delay(NFC_RESET_DELAY);
    }

    // Wake up from power down
    if (g_nfc_config.interface == NFC_INTERFACE_SPI) {
        NFC_CS_LOW();
        NFC_Delay(NFC_SPI_DELAY);
        NFC_CS_HIGH();
        NFC_Delay(NFC_WAKEUP_DELAY);
    }

    return NFC_STATUS_OK;
}

/**
 * @brief Get firmware version
 */
nfc_status_t NFC_GetFirmwareVersion(uint32_t *version)
{
    if (!nfc_internal.is_initialized || version == NULL) {
        return NFC_STATUS_ERROR;
    }

    uint8_t cmd[] = {PN532_COMMAND_GETFIRMWAREVERSION};
    uint8_t response[12];
    uint8_t response_len = sizeof(response);

    nfc_status_t status = NFC_SendCommand(cmd, sizeof(cmd), response, &response_len);
    if (status != NFC_STATUS_OK) {
        return status;
    }

    if (response_len >= 4 && response[0] == (PN532_COMMAND_GETFIRMWAREVERSION + 1)) {
        *version = (response[1] << 24) | (response[2] << 16) | (response[3] << 8) | response[4];
        return NFC_STATUS_OK;
    }

    return NFC_STATUS_ERROR;
}

/**
 * @brief Configure PN532 for passive target detection
 */
nfc_status_t NFC_ConfigureMode(void)
{
    if (!nfc_internal.is_initialized) {
        return NFC_STATUS_NOT_INITIALIZED;
    }

    // SAM Configuration: Normal mode, timeout 50ms
    uint8_t sam_cmd[] = {PN532_COMMAND_SAMCONFIGURATION, 0x01, 0x14, 0x01};
    uint8_t response[16];
    uint8_t response_len = sizeof(response);

    nfc_status_t status = NFC_SendCommand(sam_cmd, sizeof(sam_cmd), response, &response_len);
    if (status != NFC_STATUS_OK) {
        return status;
    }

    return NFC_STATUS_OK;
}

/**
 * @brief Scan for NFC/RFID cards
 */
nfc_status_t NFC_ScanCard(nfc_card_info_t *card_info)
{
    if (!nfc_internal.is_initialized || card_info == NULL) {
        return NFC_STATUS_ERROR;
    }

    // InListPassiveTarget command for Type A cards
    uint8_t cmd[] = {PN532_COMMAND_INLISTPASSIVETARGET, 0x01, 0x00};
    uint8_t response[32];
    uint8_t response_len = sizeof(response);

    nfc_status_t status = NFC_SendCommand(cmd, sizeof(cmd), response, &response_len);
    if (status != NFC_STATUS_OK) {
        return status;
    }

    // Check if card detected
    if (response_len < 6 || response[0] != (PN532_COMMAND_INLISTPASSIVETARGET + 1) || response[1] != 0x01) {
        return NFC_STATUS_NO_CARD;
    }

    // Parse card information
    uint8_t idx = 2;
    idx++;  // Skip target number
    card_info->atqa[0] = response[idx++];
    card_info->atqa[1] = response[idx++];
    card_info->sak = response[idx++];
    card_info->uid_length = response[idx++];

    if (card_info->uid_length > 10) {
        return NFC_STATUS_ERROR;
    }

    // Copy UID
    memcpy(card_info->uid, &response[idx], card_info->uid_length);

    // Convert UID to 32-bit integer
    card_info->uid_32 = NFC_UIDToUint32(card_info->uid, card_info->uid_length);

    // Determine card type
    if ((card_info->atqa[0] & 0x40) && (card_info->sak & 0x20)) {
        card_info->type = NFC_CARD_TYPE_106_A;
        strcpy(card_info->type_string, "MIFARE Classic");
    } else if (card_info->sak & 0x40) {
        card_info->type = NFC_CARD_TYPE_106_A;
        strcpy(card_info->type_string, "MIFARE Ultralight");
    } else {
        card_info->type = NFC_CARD_TYPE_106_A;
        strcpy(card_info->type_string, "ISO14443A");
    }

    // Copy to global card info
    memcpy(&g_current_card, card_info, sizeof(nfc_card_info_t));
    nfc_internal.card_present = 1;

    return NFC_STATUS_OK;
}

/**
 * @brief Read card UID only
 */
nfc_status_t NFC_ReadCardUID(uint32_t *uid)
{
    if (uid == NULL) {
        return NFC_STATUS_ERROR;
    }

    nfc_card_info_t card_info;
    nfc_status_t status = NFC_ScanCard(&card_info);

    if (status == NFC_STATUS_OK) {
        *uid = card_info.uid_32;
    }

    return status;
}

/**
 * @brief Check if card is still present
 */
nfc_status_t NFC_IsCardPresent(void)
{
    if (!nfc_internal.is_initialized) {
        return NFC_STATUS_NOT_INITIALIZED;
    }

    // Try to scan for card with short timeout
    nfc_card_info_t temp_card;
    nfc_status_t status = NFC_ScanCard(&temp_card);

    if (status == NFC_STATUS_OK) {
        // Check if it's the same card
        if (nfc_internal.card_present &&
            memcmp(temp_card.uid, g_current_card.uid, temp_card.uid_length) == 0) {
            return NFC_STATUS_OK;
        } else {
            // Different card detected
            memcpy(&g_current_card, &temp_card, sizeof(nfc_card_info_t));
            return NFC_STATUS_OK;
        }
    } else if (status == NFC_STATUS_NO_CARD) {
        if (nfc_internal.card_present) {
            nfc_internal.card_present = 0;
            memset(&g_current_card, 0, sizeof(nfc_card_info_t));
            return NFC_STATUS_CARD_REMOVED;
        }
        return NFC_STATUS_NO_CARD;
    }

    return status;
}

/**
 * @brief Release currently selected card
 */
nfc_status_t NFC_ReleaseCard(void)
{
    if (!nfc_internal.is_initialized) {
        return NFC_STATUS_NOT_INITIALIZED;
    }

    uint8_t cmd[] = {PN532_COMMAND_INRELEASE, 0x00};
    uint8_t response[16];
    uint8_t response_len = sizeof(response);

    nfc_status_t status = NFC_SendCommand(cmd, sizeof(cmd), response, &response_len);

    nfc_internal.card_present = 0;
    memset(&g_current_card, 0, sizeof(nfc_card_info_t));

    return status;
}

/**
 * @brief Read data from MIFARE Classic card
 */
nfc_status_t NFC_ReadMifareBlock(uint8_t block_number, uint8_t *data)
{
    if (!nfc_internal.is_initialized || data == NULL || !nfc_internal.card_present) {
        return NFC_STATUS_ERROR;
    }

    // MIFARE Classic Read command
    uint8_t cmd[] = {
        PN532_COMMAND_INDATAEXCHANGE,
        0x01,  // Target number
        0x30,  // MIFARE Read command
        block_number
    };

    uint8_t response[32];
    uint8_t response_len = sizeof(response);

    nfc_status_t status = NFC_SendCommand(cmd, sizeof(cmd), response, &response_len);
    if (status != NFC_STATUS_OK) {
        return status;
    }

    // Check response
    if (response_len >= 18 && response[0] == (PN532_COMMAND_INDATAEXCHANGE + 1) && response[1] == 0x00) {
        memcpy(data, &response[2], 16);  // Copy 16 bytes of data
        return NFC_STATUS_OK;
    }

    return NFC_STATUS_ERROR;
}

/**
 * @brief Write data to MIFARE Classic card
 */
nfc_status_t NFC_WriteMifareBlock(uint8_t block_number, uint8_t *data)
{
    if (!nfc_internal.is_initialized || data == NULL || !nfc_internal.card_present) {
        return NFC_STATUS_ERROR;
    }

    // MIFARE Classic Write command
    uint8_t cmd[20];
    cmd[0] = PN532_COMMAND_INDATAEXCHANGE;
    cmd[1] = 0x01;  // Target number
    cmd[2] = 0xA0;  // MIFARE Write command
    cmd[3] = block_number;
    memcpy(&cmd[4], data, 16);  // Copy 16 bytes of data

    uint8_t response[16];
    uint8_t response_len = sizeof(response);

    nfc_status_t status = NFC_SendCommand(cmd, sizeof(cmd), response, &response_len);
    if (status != NFC_STATUS_OK) {
        return status;
    }

    // Check response
    if (response_len >= 2 && response[0] == (PN532_COMMAND_INDATAEXCHANGE + 1) && response[1] == 0x00) {
        return NFC_STATUS_OK;
    }

    return NFC_STATUS_ERROR;
}

/**
 * @brief Self-test function
 */
nfc_status_t NFC_SelfTest(void)
{
    if (!nfc_internal.is_initialized) {
        return NFC_STATUS_NOT_INITIALIZED;
    }

    uint32_t version;
    return NFC_GetFirmwareVersion(&version);
}

/**
 * @brief Convert UID array to 32-bit integer
 */
uint32_t NFC_UIDToUint32(uint8_t *uid, uint8_t length)
{
    if (uid == NULL || length == 0) {
        return 0;
    }

    uint32_t result = 0;
    uint8_t bytes_to_use = (length > 4) ? 4 : length;

    for (uint8_t i = 0; i < bytes_to_use; i++) {
        result |= ((uint32_t)uid[i] << (8 * (bytes_to_use - 1 - i)));
    }

    return result;
}

/**
 * @brief Get card type string
 */
const char* NFC_GetCardTypeString(nfc_card_type_t type)
{
    switch (type) {
        case NFC_CARD_TYPE_106_A:
            return "ISO14443A";
        case NFC_CARD_TYPE_106_B:
            return "ISO14443B";
        case NFC_CARD_TYPE_212_F:
            return "FeliCa 212";
        case NFC_CARD_TYPE_424_F:
            return "FeliCa 424";
        default:
            return "Unknown";
    }
}

/* Utility Functions --------------------------------------------------------*/

/**
 * @brief Initialize NFC with I2C interface and default settings
 */
nfc_status_t NFC_InitI2CDefault(I2C_HandleTypeDef *hi2c, GPIO_TypeDef *rst_port, uint16_t rst_pin)
{
    if (hi2c == NULL) {
        return NFC_STATUS_ERROR;
    }

    // Setup default I2C configuration
    nfc_config_t config = {
        .interface = NFC_INTERFACE_I2C,
        .handle = hi2c,
        
        // GPIO pins
        .rst_port = rst_port,
        .rst_pin = rst_pin,
        .irq_port = NULL,          // No IRQ pin
        .irq_pin = 0,
        
        // SPI pins (not used for I2C)
        .cs_port = NULL,
        .cs_pin = 0,
        
        // Configuration
        .use_irq = 0,              // Don't use IRQ
        .timeout_ms = NFC_DEFAULT_TIMEOUT,
        .max_retries = NFC_MAX_RETRIES
    };

    return NFC_Init(&config);
}

/**
 * @brief Send command and receive response
 */
nfc_status_t NFC_SendCommand(uint8_t *cmd, uint8_t cmd_len, uint8_t *response, uint8_t *response_len)
{
    if (!nfc_internal.is_initialized || cmd == NULL || response == NULL || response_len == NULL) {
        return NFC_STATUS_ERROR;
    }

    // Prepare command frame
    uint8_t frame[64];
    frame[0] = NFC_PREAMBLE;
    frame[1] = NFC_START_CODE1;
    frame[2] = NFC_START_CODE2;
    frame[3] = cmd_len + 1;  // Length including direction byte
    frame[4] = ~(cmd_len + 1) + 1;  // Length checksum
    frame[5] = NFC_HOSTTOPN532;  // Direction

    memcpy(&frame[6], cmd, cmd_len);

    uint8_t checksum = NFC_CalculateChecksum(&frame[5], cmd_len + 1);
    frame[6 + cmd_len] = checksum;
    frame[7 + cmd_len] = NFC_POSTAMBLE;

    uint8_t frame_len = cmd_len + 8;

    // Send command
    nfc_status_t status;
    switch (g_nfc_config.interface) {
        case NFC_INTERFACE_SPI:
            status = NFC_SPI_WriteCommand(frame, frame_len);
            break;
        case NFC_INTERFACE_I2C:
            status = NFC_I2C_WriteCommand(frame, frame_len);
            break;
        default:
            return NFC_STATUS_ERROR;
    }

    if (status != NFC_STATUS_OK) {
        return status;
    }

    // Wait for and read ACK
    status = NFC_ReadAck();
    if (status != NFC_STATUS_OK) {
        return status;
    }

    // Read response
    return NFC_ReadResponse(response, response_len, g_nfc_config.timeout_ms);
}

/**
 * @brief Wait for PN532 ready
 */
nfc_status_t NFC_WaitReady(uint32_t timeout)
{
    uint32_t start_time = HAL_GetTick();

    while ((HAL_GetTick() - start_time) < timeout) {
        if (g_nfc_config.use_irq && g_nfc_config.irq_port != NULL) {
            if (HAL_GPIO_ReadPin(g_nfc_config.irq_port, g_nfc_config.irq_pin) == GPIO_PIN_RESET) {
                return NFC_STATUS_OK;
            }
        } else {
            // Poll status (only I2C supported for now)
            if (g_nfc_config.interface == NFC_INTERFACE_I2C) {
                // For I2C, we don't need to poll status - assume ready
                return NFC_STATUS_OK;
            }
            #ifdef HAL_SPI_MODULE_ENABLED
            else if (g_nfc_config.interface == NFC_INTERFACE_SPI) {
                // SPI status polling
                uint8_t status;
                NFC_CS_LOW();
                uint8_t cmd = PN532_SPI_STATUSREAD;
                HAL_SPI_Transmit((SPI_HandleTypeDef*)g_nfc_config.handle, &cmd, 1, 100);
                HAL_SPI_Receive((SPI_HandleTypeDef*)g_nfc_config.handle, &status, 1, 100);
                NFC_CS_HIGH();

                if (status == PN532_I2C_READY) {
                    return NFC_STATUS_OK;
                }
            }
            #endif
        }
        NFC_Delay(10);
    }

    return NFC_STATUS_TIMEOUT;
}

/**
 * @brief Read ACK frame
 */
nfc_status_t NFC_ReadAck(void)
{
    uint8_t ack_frame[] = PN532_ACK_FRAME;
    uint8_t buffer[6];
    uint8_t length = 6;

    nfc_status_t status = NFC_WaitReady(1000);
    if (status != NFC_STATUS_OK) {
        return status;
    }

    switch (g_nfc_config.interface) {
        case NFC_INTERFACE_SPI:
            status = NFC_SPI_ReadResponse(buffer, &length);
            break;
        case NFC_INTERFACE_I2C:
            status = NFC_I2C_ReadResponse(buffer, &length);
            break;
        default:
            return NFC_STATUS_ERROR;
    }

    if (status == NFC_STATUS_OK && length == 6) {
        if (memcmp(buffer, ack_frame, 6) == 0) {
            return NFC_STATUS_OK;
        }
    }

    return NFC_STATUS_ERROR;
}

/* SPI Interface Functions ---------------------------------------------------*/

#ifdef HAL_SPI_MODULE_ENABLED
/**
 * @brief SPI initialization
 */
nfc_status_t NFC_SPI_Init(void)
{
    if (g_nfc_config.cs_port == NULL) {
        return NFC_STATUS_ERROR;
    }

    NFC_CS_HIGH();
    return NFC_STATUS_OK;
}

/**
 * @brief SPI write command
 */
nfc_status_t NFC_SPI_WriteCommand(uint8_t *cmd, uint8_t len)
{
    if (cmd == NULL) {
        return NFC_STATUS_ERROR;
    }

    NFC_CS_LOW();
    uint8_t write_cmd = PN532_SPI_DATAWRITE;
    HAL_StatusTypeDef hal_status = HAL_SPI_Transmit((SPI_HandleTypeDef*)g_nfc_config.handle,
                                                   &write_cmd, 1, 1000);
    if (hal_status == HAL_OK) {
        hal_status = HAL_SPI_Transmit((SPI_HandleTypeDef*)g_nfc_config.handle, cmd, len, 1000);
    }
    NFC_CS_HIGH();

    return (hal_status == HAL_OK) ? NFC_STATUS_OK : NFC_STATUS_COMM_ERROR;
}

/**
 * @brief SPI read response
 */
nfc_status_t NFC_SPI_ReadResponse(uint8_t *response, uint8_t *len)
{
    if (response == NULL || len == NULL) {
        return NFC_STATUS_ERROR;
    }

    NFC_CS_LOW();
    uint8_t read_cmd = PN532_SPI_DATAREAD;
    HAL_StatusTypeDef hal_status = HAL_SPI_Transmit((SPI_HandleTypeDef*)g_nfc_config.handle,
                                                   &read_cmd, 1, 1000);
    if (hal_status == HAL_OK) {
        hal_status = HAL_SPI_Receive((SPI_HandleTypeDef*)g_nfc_config.handle, response, *len, 1000);
    }
    NFC_CS_HIGH();

    return (hal_status == HAL_OK) ? NFC_STATUS_OK : NFC_STATUS_COMM_ERROR;
}

#else
/**
 * @brief SPI functions disabled - SPI module not enabled
 */
nfc_status_t NFC_SPI_Init(void)
{
    return NFC_STATUS_ERROR;  // SPI not supported
}

nfc_status_t NFC_SPI_WriteCommand(uint8_t *cmd, uint8_t len)
{
    (void)cmd; (void)len;  // Suppress unused parameter warnings
    return NFC_STATUS_ERROR;  // SPI not supported
}

nfc_status_t NFC_SPI_ReadResponse(uint8_t *response, uint8_t *len)
{
    (void)response; (void)len;  // Suppress unused parameter warnings
    return NFC_STATUS_ERROR;  // SPI not supported
}
#endif  /* HAL_SPI_MODULE_ENABLED */

/* I2C Interface Functions ---------------------------------------------------*/

/**
 * @brief I2C initialization
 */
nfc_status_t NFC_I2C_Init(void)
{
    return NFC_STATUS_OK;
}

/**
 * @brief I2C write command
 */
nfc_status_t NFC_I2C_WriteCommand(uint8_t *cmd, uint8_t len)
{
    if (cmd == NULL) {
        return NFC_STATUS_ERROR;
    }

    HAL_StatusTypeDef hal_status = HAL_I2C_Master_Transmit((I2C_HandleTypeDef*)g_nfc_config.handle,
                                                          PN532_I2C_ADDRESS << 1, cmd, len, 1000);

    return (hal_status == HAL_OK) ? NFC_STATUS_OK : NFC_STATUS_COMM_ERROR;
}

/**
 * @brief I2C read response
 */
nfc_status_t NFC_I2C_ReadResponse(uint8_t *response, uint8_t *len)
{
    if (response == NULL || len == NULL) {
        return NFC_STATUS_ERROR;
    }

    HAL_StatusTypeDef hal_status = HAL_I2C_Master_Receive((I2C_HandleTypeDef*)g_nfc_config.handle,
                                                         PN532_I2C_ADDRESS << 1, response, *len, 1000);

    return (hal_status == HAL_OK) ? NFC_STATUS_OK : NFC_STATUS_COMM_ERROR;
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief Calculate checksum for PN532 frame
 */
static uint8_t NFC_CalculateChecksum(uint8_t *data, uint8_t length)
{
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < length; i++) {
        checksum += data[i];
    }
    return (~checksum + 1);
}

/**
 * @brief Read response frame
 */
static nfc_status_t NFC_ReadResponse(uint8_t *response, uint8_t *length, uint32_t timeout)
{
    if (response == NULL || length == NULL) {
        return NFC_STATUS_ERROR;
    }

    nfc_status_t status = NFC_WaitReady(timeout);
    if (status != NFC_STATUS_OK) {
        return status;
    }

    uint8_t frame[64];
    uint8_t frame_len = sizeof(frame);

    switch (g_nfc_config.interface) {
        case NFC_INTERFACE_SPI:
            status = NFC_SPI_ReadResponse(frame, &frame_len);
            break;
        case NFC_INTERFACE_I2C:
            status = NFC_I2C_ReadResponse(frame, &frame_len);
            break;
        default:
            return NFC_STATUS_ERROR;
    }

    if (status != NFC_STATUS_OK) {
        return status;
    }

    // Parse response frame
    if (frame_len < 6 || frame[0] != NFC_PREAMBLE ||
        frame[1] != NFC_START_CODE1 || frame[2] != NFC_START_CODE2) {
        return NFC_STATUS_ERROR;
    }

    uint8_t data_len = frame[3];
    if (data_len + 6 > frame_len || frame[4] != (~data_len + 1)) {
        return NFC_STATUS_ERROR;
    }

    // Verify checksum
    uint8_t checksum = NFC_CalculateChecksum(&frame[5], data_len);
    if (frame[5 + data_len] != checksum) {
        return NFC_STATUS_ERROR;
    }

    // Copy response data (skip direction byte)
    uint8_t response_data_len = data_len - 1;
    if (response_data_len > *length) {
        response_data_len = *length;
    }

    memcpy(response, &frame[6], response_data_len);
    *length = response_data_len;

    return NFC_STATUS_OK;
}

/**
 * @brief Delay function
 */
static void NFC_Delay(uint32_t ms)
{
    HAL_Delay(ms);
}
