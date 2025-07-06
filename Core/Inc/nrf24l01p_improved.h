#ifndef __NRF24L01P_IMPROVED_H__
#define __NRF24L01P_IMPROVED_H__

#include <stdbool.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"
extern SPI_HandleTypeDef hspi1;
/* Hardware Configuration Macros */
#define NRF24L01P_SPI_HANDLE                 (&hspi1) ///< SPI handle for nRF24L01+
#define NRF24L01P_SPI_CS_PORT                GPIOA    ///< GPIO port for Chip Select
#define NRF24L01P_SPI_CS_PIN                 GPIO_PIN_1 ///< GPIO pin for Chip Select
#define NRF24L01P_CE_PORT                    GPIOA    ///< GPIO port for Chip Enable
#define NRF24L01P_CE_PIN                     GPIO_PIN_0 ///< GPIO pin for Chip Enable
#define NRF24L01P_IRQ_PORT                   GPIOA    ///< GPIO port for IRQ
#define NRF24L01P_IRQ_PIN                    GPIO_PIN_8 ///< GPIO pin for IRQ
#define NRF24L01P_IRQ_ENABLE                 0         ///< Enable IRQ (1 to enable, 0 to disable)
#define NRF24L01P_DEFAULT_STATIC_PAYLOAD_BYTES 8      ///< Default static payload length (1-32 bytes)

/* nRF24L01+ Commands */
#define NRF24L01P_CMD_R_REGISTER             0x00 ///< Read register command
#define NRF24L01P_CMD_W_REGISTER             0x20 ///< Write register command
#define NRF24L01P_CMD_R_RX_PAYLOAD           0x61 ///< Read RX payload command
#define NRF24L01P_CMD_W_TX_PAYLOAD           0xA0 ///< Write TX payload command
#define NRF24L01P_CMD_FLUSH_TX               0xE1 ///< Flush TX FIFO command
#define NRF24L01P_CMD_FLUSH_RX               0xE2 ///< Flush RX FIFO command
#define NRF24L01P_CMD_REUSE_TX_PL            0xE3 ///< Reuse TX payload command
#define NRF24L01P_CMD_R_RX_PL_WID            0x60 ///< Read RX payload width command
#define NRF24L01P_CMD_W_ACK_PAYLOAD          0xA8 ///< Write ACK payload command
#define NRF24L01P_CMD_W_TX_PAYLOAD_NOACK     0xB0 ///< Write TX payload without ACK command
#define NRF24L01P_CMD_NOP                    0xFF ///< No operation command

/* nRF24L01+ Registers */
#define NRF24L01P_REG_CONFIG                 0x00 ///< Configuration register
#define NRF24L01P_REG_EN_AA                  0x01 ///< Enable auto-acknowledgment
#define NRF24L01P_REG_EN_RXADDR              0x02 ///< Enable RX addresses
#define NRF24L01P_REG_SETUP_AW               0x03 ///< Setup address width
#define NRF24L01P_REG_SETUP_RETR             0x04 ///< Setup auto-retransmission
#define NRF24L01P_REG_RF_CH                  0x05 ///< RF channel
#define NRF24L01P_REG_RF_SETUP               0x06 ///< RF setup
#define NRF24L01P_REG_STATUS                 0x07 ///< Status register
#define NRF24L01P_REG_OBSERVE_TX             0x08 ///< Observe TX (read-only)
#define NRF24L01P_REG_RPD                    0x09 ///< Received power detector (read-only)
#define NRF24L01P_REG_RX_ADDR_P0             0x0A ///< RX address for pipe 0
#define NRF24L01P_REG_RX_ADDR_P1             0x0B ///< RX address for pipe 1
#define NRF24L01P_REG_RX_ADDR_P2             0x0C ///< RX address for pipe 2
#define NRF24L01P_REG_RX_ADDR_P3             0x0D ///< RX address for pipe 3
#define NRF24L01P_REG_RX_ADDR_P4             0x0E ///< RX address for pipe 4
#define NRF24L01P_REG_RX_ADDR_P5             0x0F ///< RX address for pipe 5
#define NRF24L01P_REG_TX_ADDR                0x10 ///< TX address
#define NRF24L01P_REG_RX_PW_P0               0x11 ///< RX payload width for pipe 0
#define NRF24L01P_REG_RX_PW_P1               0x12 ///< RX payload width for pipe 1
#define NRF24L01P_REG_RX_PW_P2               0x13 ///< RX payload width for pipe 2
#define NRF24L01P_REG_RX_PW_P3               0x14 ///< RX payload width for pipe 3
#define NRF24L01P_REG_RX_PW_P4               0x15 ///< RX payload width for pipe 4
#define NRF24L01P_REG_RX_PW_P5               0x16 ///< RX payload width for pipe 5
#define NRF24L01P_REG_FIFO_STATUS            0x17 ///< FIFO status
#define NRF24L01P_REG_DYNPD                  0x1C ///< Dynamic payload length
#define NRF24L01P_REG_FEATURE                0x1D ///< Feature register

/* Register Bit Definitions */
#define NRF24L01P_CONFIG_PRIM_RX_BIT         0 ///< Primary RX mode bit
#define NRF24L01P_CONFIG_PWR_UP_BIT          1 ///< Power-up bit
#define NRF24L01P_CONFIG_CRCO_BIT            2 ///< CRC length bit
#define NRF24L01P_CONFIG_EN_CRC              3 ///< Enable CRC bit

/* STATUS Register Bits */
#define NRF24L01P_STATUS_MAX_RT_BIT          4 ///< Maximum retransmit interrupt bit
#define NRF24L01P_STATUS_TX_DS_BIT           5 ///< TX data sent interrupt bit
#define NRF24L01P_STATUS_RX_DR_BIT           6 ///< RX data ready interrupt bit

/* RF_SETUP Register Bits */
#define NRF24L01P_RF_SETUP_RF_DR_LOW_BIT     5 ///< Low data rate bit (250 kbps)
#define NRF24L01P_RF_SETUP_RF_DR_HIGH_BIT    3 ///< High data rate bit (2 Mbps)
#define NRF24L01P_RF_SETUP_RF_PWR_MSK        0x06 ///< RF power mask (bits 1-2)
#define NRF24L01P_FIFO_STATUS_RX_EMPTY_BIT   0 ///< RX FIFO empty bit

/* Enum for RF Channels (common values) */
typedef enum {
    NRF24L01P_CHANNEL_2400MHZ = 2400, ///< RF channel 0 (2400 MHz)
    NRF24L01P_CHANNEL_2402MHZ = 2402, ///< RF channel 2 (2402 MHz, default)
    NRF24L01P_CHANNEL_2410MHZ = 2410, ///< RF channel 10
    NRF24L01P_CHANNEL_2425MHZ = 2425, ///< RF channel 25
    NRF24L01P_CHANNEL_2450MHZ = 2450, ///< RF channel 50
    NRF24L01P_CHANNEL_2475MHZ = 2475, ///< RF channel 75
    NRF24L01P_CHANNEL_2500MHZ = 2500, ///< RF channel 100
    NRF24L01P_CHANNEL_2525MHZ = 2525  ///< RF channel 125
} nrf24l01p_rf_channel_t;

/* Enum for Data Rates */
typedef enum {
    NRF24L01P_DATA_RATE_250KBPS = 2, ///< 250 kbps data rate
    NRF24L01P_DATA_RATE_1MBPS   = 0, ///< 1 Mbps data rate
    NRF24L01P_DATA_RATE_2MBPS   = 1  ///< 2 Mbps data rate
} nrf24l01p_data_rate_t;

/* Enum for Output Power */
typedef enum {
    NRF24L01P_PWR_0DBM  = 3, ///< 0 dBm output power
    NRF24L01P_PWR_6DBM  = 2, ///< -6 dBm output power
    NRF24L01P_PWR_12DBM = 1, ///< -12 dBm output power
    NRF24L01P_PWR_18DBM = 0  ///< -18 dBm output power
} nrf24l01p_output_power_t;

/* Enum for CRC Length */
typedef enum {
    NRF24L01P_CRC_1_BYTE = 1, ///< 1-byte CRC
    NRF24L01P_CRC_2_BYTES = 2 ///< 2-byte CRC
} nrf24l01p_crc_length_t;

/* Enum for Address Width */
typedef enum {
    NRF24L01P_ADDRESS_WIDTH_3_BYTES = 3, ///< 3-byte address width
    NRF24L01P_ADDRESS_WIDTH_4_BYTES = 4, ///< 4-byte address width
    NRF24L01P_ADDRESS_WIDTH_5_BYTES = 5  ///< 5-byte address width
} nrf24l01p_address_width_t;

/* Enum for Retransmit Count (common values) */
typedef enum {
    NRF24L01P_RETRANSMIT_COUNT_0  = 0,  ///< No retransmission
    NRF24L01P_RETRANSMIT_COUNT_1  = 1,  ///< 1 retransmission
    NRF24L01P_RETRANSMIT_COUNT_3  = 3,  ///< 3 retransmissions (default)
    NRF24L01P_RETRANSMIT_COUNT_5  = 5,  ///< 5 retransmissions
    NRF24L01P_RETRANSMIT_COUNT_10 = 10, ///< 10 retransmissions
    NRF24L01P_RETRANSMIT_COUNT_15 = 15  ///< 15 retransmissions (max)
} nrf24l01p_retransmit_count_t;

/* Enum for Retransmit Delay (common values, multiples of 250 µs) */
typedef enum {
    NRF24L01P_RETRANSMIT_DELAY_250US  = 250,  ///< 250 µs delay
    NRF24L01P_RETRANSMIT_DELAY_500US  = 500,  ///< 500 µs delay
    NRF24L01P_RETRANSMIT_DELAY_1000US = 1000, ///< 1000 µs delay
    NRF24L01P_RETRANSMIT_DELAY_2000US = 2000, ///< 2000 µs delay
    NRF24L01P_RETRANSMIT_DELAY_4000US = 4000  ///< 4000 µs delay (max)
} nrf24l01p_retransmit_delay_t;

/* Error Codes */
typedef enum {
    NRF24L01P_OK = 0,           ///< Operation successful
    NRF24L01P_ERR_INVALID_PARAM, ///< Invalid parameter
    NRF24L01P_ERR_SPI,          ///< SPI communication error
    NRF24L01P_ERR_NO_DATA       ///< No data in RX FIFO
} nrf24l01p_status_t;

/* Main Functions */
/**
 * @brief Initialize nRF24L01+ in RX mode.
 * @param channel RF channel (2400-2525 MHz).
 * @param data_rate Data rate (250 kbps, 1 Mbps, 2 Mbps).
 * @param payload_width Payload width in bytes (1-32).
 * @param crc_length CRC length in bytes (1 or 2).
 * @param address_width Address width in bytes (3-5).
 * @param retransmit_count Number of retransmit attempts (0-15).
 * @param retransmit_delay Retransmit delay in us (250-4000, multiple of 250).
 * @param rx_address RX address for pipe 0 (5 bytes max).
 * @return NRF24L01P_OK on success, error code otherwise.
 */
nrf24l01p_status_t nrf24l01p_rx_init(nrf24l01p_rf_channel_t channel,
                                     nrf24l01p_data_rate_t data_rate,
                                     uint8_t payload_width,
                                     nrf24l01p_crc_length_t crc_length,
                                     nrf24l01p_address_width_t address_width,
                                     nrf24l01p_retransmit_count_t retransmit_count,
                                     nrf24l01p_retransmit_delay_t retransmit_delay,
                                     const uint8_t *rx_address);

/**
 * @brief Initialize nRF24L01+ in TX mode.
 * @param channel RF channel (2400-2525 MHz).
 * @param data_rate Data rate (250 kbps, 1 Mbps, 2 Mbps).
 * @param crc_length CRC length in bytes (1 or 2).
 * @param address_width Address width in bytes (3-5).
 * @param retransmit_count Number of retransmit attempts (0-44).
 * @param retransmit_delay Retransmit delay in us (250-4000, multiple of 250).
 * @param tx_address TX address (5 bytes max).
 * @return NRF24L01P_OK on success, error code otherwise.
 */
nrf24l01p_status_t nrf24l01p_tx_init(nrf24l01p_rf_channel_t channel,
                                     nrf24l01p_data_rate_t data_rate,
                                     nrf24l01p_crc_length_t crc_length,
                                     nrf24l01p_address_width_t address_width,
                                     nrf24l01p_retransmit_count_t retransmit_count,
                                     nrf24l01p_retransmit_delay_t retransmit_delay,
                                     const uint8_t *tx_address);

/**
 * @brief Receive data in RX mode.
 * @param rx_payload Buffer to store received payload.
 * @return NRF24L01P_OK on success, error code otherwise.
 */
nrf24l01p_status_t nrf24l01p_rx_receive(uint8_t *rx_payload);

/**
 * @brief Transmit data in TX mode.
 * @param tx_payload Buffer containing payload to transmit.
 * @return NRF24L01P_OK on success, error code otherwise.
 */
nrf24l01p_status_t nrf24l01p_tx_transmit(const uint8_t *tx_payload);

/**
 * @brief Handle TX interrupts (TX_DS or MAX_RT).
 * @return NRF24L01P_OK on success, error code otherwise.
 */
nrf24l01p_status_t nrf24l01p_tx_irq(void);

/**
 * @brief Handle RX interrupts (RX_DR).
 * @return NRF24L01P_OK on success, error code otherwise.
 */
nrf24l01p_status_t nrf24l01p_rx_irq(uint8_t *rx_payload);

/* Sub Functions */
/**
 * @brief Reset nRF24L01+ to default state.
 * @return NRF24L01P_OK on success, error code otherwise.
 */
nrf24l01p_status_t nrf24l01p_reset(void);

/**
 * @brief Set nRF24L01+ to Primary RX mode.
 * @return NRF24L01P_OK on success, error code otherwise.
 */
nrf24l01p_status_t nrf24l01p_prx_mode(void);

/**
 * @brief Set nRF24L01+ to Primary TX mode.
 * @return NRF24L01P_OK on success, error code otherwise.
 */
nrf24l01p_status_t nrf24l01p_ptx_mode(void);

/**
 * @brief Power up the nRF24L01+.
 * @return NRF24L01P_OK on success, error code otherwise.
 */
nrf24l01p_status_t nrf24l01p_power_up(void);

/**
 * @brief Power down the nRF24L01+.
 * @return NRF24L01P_OK on success, error code otherwise.
 */
nrf24l01p_status_t nrf24l01p_power_down(void);

/**
 * @brief Get status register value.
 * @return Status register value or 0xFF on error.
 */
uint8_t nrf24l01p_get_status(void);

/**
 * @brief Get FIFO status register value.
 * @return FIFO status register value or 0xFF on error.
 */
uint8_t nrf24l01p_get_fifo_status(void);

/**
 * @brief Set RX payload width for pipe 0.
 * @param bytes Payload width (1-32 bytes).
 * @return NRF24L01P_OK on success, error code otherwise.
 */
nrf24l01p_status_t nrf24l01p_rx_set_payload_width(uint8_t bytes);

/**
 * @brief Read RX FIFO payload.
 * @param rx_payload Buffer to store payload.
 * @return NRF24L01P_OK on success, error code otherwise.
 */
nrf24l01p_status_t nrf24l01p_read_rx_fifo(uint8_t *rx_payload);

/**
 * @brief Write TX FIFO payload.
 * @param tx_payload Buffer containing payload.
 * @return NRF24L01P_OK on success, error code otherwise.
 */
nrf24l01p_status_t nrf24l01p_write_tx_fifo(const uint8_t *tx_payload);

/**
 * @brief Flush RX FIFO.
 * @return NRF24L01P_OK on success, error code otherwise.
 */
nrf24l01p_status_t nrf24l01p_flush_rx_fifo(void);

/**
 * @brief Flush TX FIFO.
 * @return NRF24L01P_OK on success, error code otherwise.
 */
nrf24l01p_status_t nrf24l01p_flush_tx_fifo(void);

/**
 * @brief Clear RX data ready interrupt.
 * @return NRF24L01P_OK on success, error code otherwise.
 */
nrf24l01p_status_t nrf24l01p_clear_rx_dr(void);

/**
 * @brief Clear TX data sent interrupt.
 * @return NRF24L01P_OK on success, error code otherwise.
 */
nrf24l01p_status_t nrf24l01p_clear_tx_ds(void);

/**
 * @brief Clear maximum retransmit interrupt.
 * @return NRF24L01P_OK on success, error code otherwise.
 */
nrf24l01p_status_t nrf24l01p_clear_max_rt(void);

/**
 * @brief Set RF channel.
 * @param channel Channel frequency (2400-2525 MHz).
 * @return NRF24L01P_OK on success, error code otherwise.
 */
nrf24l01p_status_t nrf24l01p_set_rf_channel(nrf24l01p_rf_channel_t channel);

/**
 * @brief Set RF output power.
 * @param power Output power level.
 * @return NRF24L01P_OK on success, error code otherwise.
 */
nrf24l01p_status_t nrf24l01p_set_rf_tx_output_power(nrf24l01p_output_power_t power);

/**
 * @brief Set RF data rate.
 * @param data_rate Data rate (250 kbps, 1 Mbps, 2 Mbps).
 * @return NRF24L01P_OK on success, error code otherwise.
 */
nrf24l01p_status_t nrf24l01p_set_rf_data_rate(nrf24l01p_data_rate_t data_rate);

/**
 * @brief Set CRC length.
 * @param bytes CRC length (1 or 2 bytes).
 * @return NRF24L01P_OK on success, error code otherwise.
 */
nrf24l01p_status_t nrf24l01p_set_crc_length(nrf24l01p_crc_length_t bytes);

/**
 * @brief Set address width.
 * @param bytes Address width (3-5 bytes).
 * @return NRF24L01P_OK on success, error code otherwise.
 */
nrf24l01p_status_t nrf24l01p_set_address_width(nrf24l01p_address_width_t bytes);

/**
 * @brief Set auto-retransmit count.
 * @param count Number of retransmit attempts (0-15).
 * @return NRF24L01P_OK on success, error code otherwise.
 */
nrf24l01p_status_t nrf24l01p_set_auto_retransmit_count(nrf24l01p_retransmit_count_t count);

/**
 * @brief Set auto-retransmit delay.
 * @param us Delay in microseconds (250-4000, multiple of 250).
 * @return NRF24L01P_OK on success, error code otherwise.
 */
nrf24l01p_status_t nrf24l01p_set_auto_retransmit_delay(nrf24l01p_retransmit_delay_t us);

/**
 * @brief Set RX address for a pipe.
 * @param pipe Pipe number (0-5).
 * @param address Address buffer (up to 5 bytes).
 * @param length Address length (3-5 bytes).
 * @return NRF24L01P_OK on success, error code otherwise.
 */
nrf24l01p_status_t nrf24l01p_set_rx_address(uint8_t pipe, const uint8_t *address, nrf24l01p_address_width_t length);

/**
 * @brief Set TX address.
 * @param address Address buffer (up to 5 bytes).
 * @param length Address length (3-5 bytes).
 * @return NRF24L01P_OK on success, error code otherwise.
 */
nrf24l01p_status_t nrf24l01p_set_tx_address(const uint8_t *address, nrf24l01p_address_width_t length);

#endif /* __NRF24L01P_IMPROVED_H__ */