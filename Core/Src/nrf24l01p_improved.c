#include "nrf24l01p_improved.h"
#include <stdio.h>

/* Private Functions */
static void delay_us(uint32_t us) 
{
    if (us > 0) 
    {
        for (volatile uint32_t i = 0; i < us; i++); // Ensure CSN is high for at least 10 µs
    }
}

static void cs_high(void) 
{
    HAL_GPIO_WritePin(NRF24L01P_SPI_CS_PORT, NRF24L01P_SPI_CS_PIN, GPIO_PIN_SET);
    delay_us(100); // Ensure CSN is high for at least 10 µs
}

static void cs_low(void) 
{
    HAL_GPIO_WritePin(NRF24L01P_SPI_CS_PORT, NRF24L01P_SPI_CS_PIN, GPIO_PIN_RESET);
    delay_us(100); // Ensure CSN is high for at least 10 µs
}

static void ce_high(void) 
{
    HAL_GPIO_WritePin(NRF24L01P_CE_PORT, NRF24L01P_CE_PIN, GPIO_PIN_SET);
    delay_us(100); // Ensure CSN is high for at least 10 µs
}

static void ce_low(void) 
{
    HAL_GPIO_WritePin(NRF24L01P_CE_PORT, NRF24L01P_CE_PIN, GPIO_PIN_RESET);
    delay_us(100); // Ensure CSN is high for at least 10 µs
}

static nrf24l01p_status_t spi_transfer(uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len) 
{
    if (tx_buf == NULL || rx_buf == NULL || len == 0) 
    {
        //printf("Invalid parameters in spi_transfer\n");
        return NRF24L01P_ERR_INVALID_PARAM;
    }

    cs_low();
    HAL_StatusTypeDef hal_status = HAL_SPI_TransmitReceive(NRF24L01P_SPI_HANDLE, tx_buf, rx_buf, len, 2000);
    cs_high();

    if (hal_status != HAL_OK) 
    {
        //printf("SPI transfer failed\n");
        return NRF24L01P_ERR_SPI;
    }
    return NRF24L01P_OK;
}

static nrf24l01p_status_t read_register(uint8_t reg, uint8_t *value) {
    if (value == NULL) {
        //printf("Invalid pointer in read_register\n");
        return NRF24L01P_ERR_INVALID_PARAM;
    }

    uint8_t tx_buf[2] = {NRF24L01P_CMD_R_REGISTER | (reg & 0x1F), 0xFF};
    uint8_t rx_buf[2];

    nrf24l01p_status_t status = spi_transfer(tx_buf, rx_buf, 2);

    if (status != NRF24L01P_OK) {
        //printf("SPI read failed for register 0x%02X\n", reg);
        return status;
    }

    *value = rx_buf[1];
    return NRF24L01P_OK;
}

static nrf24l01p_status_t write_register(uint8_t reg, uint8_t value) 
{
    uint8_t tx_buf[2] = {NRF24L01P_CMD_W_REGISTER | (reg & 0x1F), value};
    uint8_t rx_buf[2];

    nrf24l01p_status_t status = spi_transfer(tx_buf, rx_buf, 2);

    if (status != NRF24L01P_OK) 
    {
        //printf("SPI write failed for register 0x%02X\n", reg);
        return status;
    }

    /* 
        Nếu là thanh ghi STATUS hoặc FIFO_STATUS, không cần so sánh giá trị vì các thanh ghi này thể tự động cập nhật lại giá trị 
        + NRF24L01P_REG_STATUS: Các bit RX_DR, TX_DS, MAX_RT bị clear tự động khi ghi vào
        + NRF24L01P_REG_FIFO_STATUS: Các bit TX_EMPTY, RX_EMPTY cũng tự động cập nhật khi ghi vào
        Do đó, chỉ cần đọc lại để xác nhận ghi thành công, không cần so sánh giá trị cứng.
        Việc này giúp tránh lỗi khi ghi vào các thanh ghi này, vì chúng có thể tự động thay đổi giá trị ngay sau khi ghi.
        Nếu cần so sánh giá trị cứng, có thể bỏ qua phần này.
    */
    if (reg == NRF24L01P_REG_STATUS || reg == NRF24L01P_REG_FIFO_STATUS) 
    {
        uint8_t read_back = 0;
        if (read_register(reg, &read_back) != NRF24L01P_OK) 
        {   
//          printf("Lỗi SPI khi đọc lại STATUS/FIFO_STATUS\n");
            return NRF24L01P_ERR_SPI;
        }
//       printf("Đã ghi reg 0x%02X, đọc lại=0x%02X (không so sánh cứng)\n", reg, read_back);
        return NRF24L01P_OK;
    }

    // Với các thanh ghi khác phải kiểm tra giá trị chính xác
    uint8_t read_back = 0;
    if (read_register(reg, &read_back) != NRF24L01P_OK || read_back != value) 
    {
//      printf("Ghi reg 0x%02X thất bại, expected=0x%02X, got=0x%02X\n", reg, value, read_back);
        return NRF24L01P_ERR_SPI;
    }

    return NRF24L01P_OK;
}

/* Initialize IRQ for RF24L01 */
static nrf24l01p_status_t nrf24l01p_irq_init(void) 
{
#if NRF24L01P_IRQ_ENABLE == 1
    // Thiết lập callback cho ngắt (cả rising và falling edge)
    int status = gpioSetAlertFunc(NRF24L01P_IRQ_GPIO, nrf24l01p_callback);
    if (status < 0) 
    {
        return NRF24L01P_ERR_SPI;
    }
#endif
    return NRF24L01P_OK;
}

/* 
    Callback function for RF24L01 
    Depending on the library system, there will be different types of callback functions.
*/
#if NRF24L01P_IRQ_ENABLE == 1
static void nrf24l01p_callback(int gpio, int level, uint32_t tick) 
{
    if (gpio == NRF24L01P_IRQ_GPIO && level == 0) 
    {
        uint8_t rx_payload[RX_PAYLOAD_SIZE];
        nrf24l01p_status_t status = nrf24l01p_rx_irq(rx_payload);
        if (status == NRF24L01P_OK) 
        {
            printf("Received data: ");
            for (int i = 0; i < RX_PAYLOAD_SIZE; i++)
            {
                printf("%02X ", rx_payload[i]);
            }
            printf("\n");
        } 
        else 
        {
            printf("Failed to receive data\n");
        }
    }
}
#endif

/* Initialize SPI and GPIO */
nrf24l01p_status_t nrf24l01p_spi_init(void) 
{
   
}

/* Deinitialize SPI and GPIO */
void nrf24l01p_spi_deinit(void) 
{

}


/* Main Functions */
nrf24l01p_status_t nrf24l01p_rx_init(nrf24l01p_rf_channel_t channel,
                                     nrf24l01p_data_rate_t data_rate,
                                     uint8_t payload_width,
                                     nrf24l01p_crc_length_t crc_length,
                                     nrf24l01p_address_width_t address_width,
                                     nrf24l01p_retransmit_count_t retransmit_count,
                                     nrf24l01p_retransmit_delay_t retransmit_delay,
                                     const uint8_t *rx_address) 
{
    // Validate parameters
    if (channel < 2400 || channel > 2525 || payload_width > 32 ||
        (crc_length != NRF24L01P_CRC_1_BYTE && crc_length != NRF24L01P_CRC_2_BYTES) ||
        (address_width < NRF24L01P_ADDRESS_WIDTH_3_BYTES || address_width > NRF24L01P_ADDRESS_WIDTH_5_BYTES) ||
        retransmit_count > NRF24L01P_RETRANSMIT_COUNT_15 ||
        retransmit_delay < NRF24L01P_RETRANSMIT_DELAY_250US ||
        retransmit_delay > NRF24L01P_RETRANSMIT_DELAY_4000US ||
        (retransmit_delay % 250 != 0) || rx_address == NULL) 
    {
        //printf("Invalid parameters in rx_init\n");
        return NRF24L01P_ERR_INVALID_PARAM;
    }

    nrf24l01p_status_t status = nrf24l01p_reset();
    if (status != NRF24L01P_OK) 
    {
        //printf("Reset failed: %d\n", status);
        return status;
    }

    // Disable auto-ACK for simplicity
    status = write_register(NRF24L01P_REG_EN_AA, 0x00);
    if (status != NRF24L01P_OK) return status;

    // Enable RX pipe 0
    status = write_register(NRF24L01P_REG_EN_RXADDR, 0x01);
    if (status != NRF24L01P_OK) return status;

    status = nrf24l01p_prx_mode();
    if (status != NRF24L01P_OK) return status;

    status = nrf24l01p_power_up();
    if (status != NRF24L01P_OK) return status;
    delay_us(200); // Wait 2 ms for power-up

    status = nrf24l01p_rx_set_payload_width(payload_width);
    if (status != NRF24L01P_OK) return status;

    status = nrf24l01p_set_rf_channel(channel);
    if (status != NRF24L01P_OK) return status;

    status = nrf24l01p_set_rf_data_rate(data_rate);
    if (status != NRF24L01P_OK) return status;

    status = nrf24l01p_set_rf_tx_output_power(NRF24L01P_PWR_0DBM);
    if (status != NRF24L01P_OK) return status;

    status = nrf24l01p_set_crc_length(crc_length);
    if (status != NRF24L01P_OK) return status;

    status = nrf24l01p_set_address_width(address_width);
    if (status != NRF24L01P_OK) return status;

    status = nrf24l01p_set_auto_retransmit_count(retransmit_count);
    if (status != NRF24L01P_OK) return status;

    status = nrf24l01p_set_auto_retransmit_delay(retransmit_delay);
    if (status != NRF24L01P_OK) return status;

    status = nrf24l01p_set_rx_address(0, rx_address, address_width);
    if (status != NRF24L01P_OK) return status;

    // Debug: Print key registers
    uint8_t config, rf_ch, rx_pw_p0, en_aa, en_rxaddr;
    read_register(NRF24L01P_REG_CONFIG, &config);
    read_register(NRF24L01P_REG_RF_CH, &rf_ch);
    read_register(NRF24L01P_REG_RX_PW_P0, &rx_pw_p0);
    read_register(NRF24L01P_REG_EN_AA, &en_aa);
    read_register(NRF24L01P_REG_EN_RXADDR, &en_rxaddr);
    
    uint8_t tx_buf[6] = {NRF24L01P_CMD_R_REGISTER | NRF24L01P_REG_RX_ADDR_P0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t rx_buf[6];
    
    nrf24l01p_status_t status_addr = spi_transfer(tx_buf, rx_buf, 6);
		
    if (status_addr != NRF24L01P_OK) 
    {
        //printf("SPI error reading RX_ADDR_P0\n");
        return status_addr;
    }

	// uint8_t rx_addr[5];			
    // for (int i = 0; i < 5; i++) {
    //     rx_addr[i] = rx_buf[i + 1];
    // }
    // printf("RX Config: CONFIG=0x%02X, RF_CH=0x%02X, RX_PW_P0=0x%02X, EN_AA=0x%02X, EN_RXADDR=0x%02X, RX_ADDR_P0=%02X %02X %02X %02X %02X\n",config, rf_ch, rx_pw_p0, en_aa, en_rxaddr, rx_addr[0], rx_addr[1], rx_addr[2], rx_addr[3], rx_addr[4]);

    ce_high(); // Enter RX mode
    return NRF24L01P_OK;
}

nrf24l01p_status_t nrf24l01p_tx_init(nrf24l01p_rf_channel_t channel,
                                     nrf24l01p_data_rate_t data_rate,
                                     nrf24l01p_crc_length_t crc_length,
                                     nrf24l01p_address_width_t address_width,
                                     nrf24l01p_retransmit_count_t retransmit_count,
                                     nrf24l01p_retransmit_delay_t retransmit_delay,
                                     const uint8_t *tx_address) 
{
    // Validate parameters
    if (channel < 2400 || channel > 2525 ||
        (crc_length != NRF24L01P_CRC_1_BYTE && crc_length != NRF24L01P_CRC_2_BYTES) ||
        (address_width < NRF24L01P_ADDRESS_WIDTH_3_BYTES || address_width > NRF24L01P_ADDRESS_WIDTH_5_BYTES) ||
        retransmit_count > NRF24L01P_RETRANSMIT_COUNT_15 ||
        retransmit_delay < NRF24L01P_RETRANSMIT_DELAY_250US ||
        retransmit_delay > NRF24L01P_RETRANSMIT_DELAY_4000US ||
        (retransmit_delay % 250 != 0) || tx_address == NULL) 
    {
        //printf("Invalid parameters in tx_init\n");
        return NRF24L01P_ERR_INVALID_PARAM;
    }

    nrf24l01p_status_t status = nrf24l01p_reset();
    if (status != NRF24L01P_OK) 
    {
        //printf("Reset failed: %d\n", status);
        return status;
    }

    // Disable auto-ACK for simplicity
    status = write_register(NRF24L01P_REG_EN_AA, 0x00);
    if (status != NRF24L01P_OK) return status;

    status = nrf24l01p_ptx_mode();
    if (status != NRF24L01P_OK) return status;

    status = nrf24l01p_power_up();
    if (status != NRF24L01P_OK) return status;
    delay_us(200); // Wait 2 ms for power-up

    status = nrf24l01p_set_rf_channel(channel);
    if (status != NRF24L01P_OK) return status;

    status = nrf24l01p_set_rf_data_rate(data_rate);
    if (status != NRF24L01P_OK) return status;

    status = nrf24l01p_set_rf_tx_output_power(NRF24L01P_PWR_0DBM);
    if (status != NRF24L01P_OK) return status;

    status = nrf24l01p_set_crc_length(crc_length);
    if (status != NRF24L01P_OK) return status;

    status = nrf24l01p_set_address_width(address_width);
    if (status != NRF24L01P_OK) return status;

    status = nrf24l01p_set_auto_retransmit_count(retransmit_count);
    if (status != NRF24L01P_OK) return status;

    status = nrf24l01p_set_auto_retransmit_delay(retransmit_delay);
    if (status != NRF24L01P_OK) return status;

    status = nrf24l01p_set_tx_address(tx_address, address_width);
    if (status != NRF24L01P_OK) return status;

    // Debug: Print key registers
    uint8_t config, rf_ch, setup_retr;
    read_register(NRF24L01P_REG_CONFIG, &config);
    read_register(NRF24L01P_REG_RF_CH, &rf_ch);
    read_register(NRF24L01P_REG_SETUP_RETR, &setup_retr);
    
    uint8_t tx_buf[6] = {NRF24L01P_CMD_R_REGISTER | NRF24L01P_REG_TX_ADDR, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t rx_buf[6];
    
    nrf24l01p_status_t status_addr = spi_transfer(tx_buf, rx_buf, 6);

    if (status_addr != NRF24L01P_OK) 
    {
        //printf("SPI error reading TX_ADDR\n");
        return status_addr;
    }

    // uint8_t tx_addr[5];
    // for (int i = 0; i < 5; i++) 
    // {
    //     tx_addr[i] = rx_buf[i + 1];
    // }
    // printf("TX Config: CONFIG=0x%02X, RF_CH=0x%02X, SETUP_RETR=0x%02X, TX_ADDR=%02X %02X %02X %02X %02X\n",config, rf_ch, setup_retr, tx_addr[0], tx_addr[1], tx_addr[2], tx_addr[3], tx_addr[4]);

    return NRF24L01P_OK;
}

nrf24l01p_status_t nrf24l01p_rx_receive(uint8_t *rx_payload) 
{
    if (rx_payload == NULL) 
    {
        //printf("Invalid payload pointer in rx_receive\n");
        return NRF24L01P_ERR_INVALID_PARAM;
    }

    nrf24l01p_status_t status = nrf24l01p_read_rx_fifo(rx_payload);
    if (status != NRF24L01P_OK) return status;

    return nrf24l01p_clear_rx_dr();
}

nrf24l01p_status_t nrf24l01p_tx_transmit(const uint8_t *tx_payload) 
{
    if (tx_payload == NULL) 
    {
        //printf("Invalid payload pointer in tx_transmit\n");
        return NRF24L01P_ERR_INVALID_PARAM;
    }

    nrf24l01p_status_t status = nrf24l01p_write_tx_fifo(tx_payload);
    if (status != NRF24L01P_OK) return status;

    ce_high();
    delay_us(150); // Pulse CE for ~15 µs
    ce_low();
    return NRF24L01P_OK;
}

nrf24l01p_status_t nrf24l01p_tx_irq(void) 
{
    uint8_t status = nrf24l01p_get_status();
    if (status == 0xFF) 
    {
        //printf("SPI error in tx_irq\n");
        return NRF24L01P_ERR_SPI;
    }

    nrf24l01p_status_t ret = NRF24L01P_OK;
    if (status & (1 << NRF24L01P_STATUS_TX_DS_BIT)) 
    {
        ret = nrf24l01p_clear_tx_ds();
    } 
    else if (status & (1 << NRF24L01P_STATUS_MAX_RT_BIT)) 
    {
        ret = nrf24l01p_clear_max_rt();
    }
    return ret;
}

nrf24l01p_status_t nrf24l01p_rx_irq(uint8_t *rx_payload) 
{
    if (rx_payload == NULL) 
    {
        //printf("Invalid payload pointer in rx_irq\n");
        return NRF24L01P_ERR_INVALID_PARAM;
    }

    uint8_t status = nrf24l01p_get_status();
    if (status == 0xFF) 
    {
        //printf("SPI error in rx_irq\n");
        return NRF24L01P_ERR_SPI;
    }

    if (status & (1 << NRF24L01P_STATUS_RX_DR_BIT)) 
    {
        nrf24l01p_status_t ret = nrf24l01p_read_rx_fifo(rx_payload);
        if (ret != NRF24L01P_OK) return ret;
        return nrf24l01p_clear_rx_dr();
    }
    return NRF24L01P_OK;
}

/* Sub Functions */
nrf24l01p_status_t nrf24l01p_reset(void) 
{
    
    ce_low();

    // Clear FIFOs first
    nrf24l01p_status_t status = nrf24l01p_flush_rx_fifo();
    if (status != NRF24L01P_OK) return status;
    status = nrf24l01p_flush_tx_fifo();
    if (status != NRF24L01P_OK) return status;

    // Reset registers to default values
    status = write_register(NRF24L01P_REG_CONFIG, 0x08); // Power down, PRX=0
    if (status != NRF24L01P_OK) return status;
    status = write_register(NRF24L01P_REG_EN_AA, 0x00); // Disable auto-ACK
    if (status != NRF24L01P_OK) return status;
    status = write_register(NRF24L01P_REG_EN_RXADDR, 0x00); // Disable all pipes
    if (status != NRF24L01P_OK) return status;
    status = write_register(NRF24L01P_REG_SETUP_AW, 0x03); // 5-byte address
    if (status != NRF24L01P_OK) return status;
    status = write_register(NRF24L01P_REG_SETUP_RETR, 0x00); // No retransmits
    if (status != NRF24L01P_OK) return status;
    status = write_register(NRF24L01P_REG_RF_CH, 0x02); // Channel 2 (2402 MHz)
    if (status != NRF24L01P_OK) return status;
    status = write_register(NRF24L01P_REG_RF_SETUP, 0x07); // 1 Mbps, 0 dBm
    if (status != NRF24L01P_OK) return status;
    status = write_register(NRF24L01P_REG_STATUS, 0x70); // Clear interrupts
    if (status != NRF24L01P_OK) return status;
    status = write_register(NRF24L01P_REG_RX_PW_P0, 0x00);
    if (status != NRF24L01P_OK) return status;
    status = write_register(NRF24L01P_REG_RX_PW_P1, 0x00);
    if (status != NRF24L01P_OK) return status;
    status = write_register(NRF24L01P_REG_RX_PW_P2, 0x00);
    if (status != NRF24L01P_OK) return status;
    status = write_register(NRF24L01P_REG_RX_PW_P3, 0x00);
    if (status != NRF24L01P_OK) return status;
    status = write_register(NRF24L01P_REG_RX_PW_P4, 0x00);
    if (status != NRF24L01P_OK) return status;
    status = write_register(NRF24L01P_REG_RX_PW_P5, 0x00);
    if (status != NRF24L01P_OK) return status;
    status = write_register(NRF24L01P_REG_DYNPD, 0x00); // Disable dynamic payload
    if (status != NRF24L01P_OK) return status;
    status = write_register(NRF24L01P_REG_FEATURE, 0x00); // Disable special features
    if (status != NRF24L01P_OK) return status;

    uint8_t clear_addr[5] = {0};
    for (uint8_t i = 0; i < 6; i++) 
    {
        nrf24l01p_set_rx_address(i, clear_addr, 5); 
    }

    return NRF24L01P_OK;
}

nrf24l01p_status_t nrf24l01p_prx_mode(void) 
{
    uint8_t config;
    nrf24l01p_status_t status = read_register(NRF24L01P_REG_CONFIG, &config);
    if (status != NRF24L01P_OK) return status;

    config |= (1 << NRF24L01P_CONFIG_PRIM_RX_BIT) | (1 << NRF24L01P_CONFIG_EN_CRC);
    return write_register(NRF24L01P_REG_CONFIG, config);
}

nrf24l01p_status_t nrf24l01p_ptx_mode(void) 
{
    uint8_t config;
    nrf24l01p_status_t status = read_register(NRF24L01P_REG_CONFIG, &config);
    if (status != NRF24L01P_OK) return status;

    config &= ~(1 << NRF24L01P_CONFIG_PRIM_RX_BIT);
    config |= (1 << NRF24L01P_CONFIG_EN_CRC);
    return write_register(NRF24L01P_REG_CONFIG, config);
}

nrf24l01p_status_t nrf24l01p_read_rx_fifo(uint8_t *rx_payload) 
{
    if (rx_payload == NULL) 
    {
        //printf("Invalid payload pointer in read_rx_fifo\n");
        return NRF24L01P_ERR_INVALID_PARAM;
    }

    uint8_t tx_buf[NRF24L01P_DEFAULT_STATIC_PAYLOAD_BYTES + 1];
    uint8_t rx_buf[NRF24L01P_DEFAULT_STATIC_PAYLOAD_BYTES + 1];
    tx_buf[0] = NRF24L01P_CMD_R_RX_PAYLOAD;
    for (int i = 1; i <= NRF24L01P_DEFAULT_STATIC_PAYLOAD_BYTES; i++) 
    {
        tx_buf[i] = 0xFF;
    }
    
    nrf24l01p_status_t status = spi_transfer(tx_buf, rx_buf, NRF24L01P_DEFAULT_STATIC_PAYLOAD_BYTES + 1);

    if (status != NRF24L01P_OK) 
    {
        //printf("SPI error in read_rx_fifo\n");
        return status;
    }

    for (int i = 0; i < NRF24L01P_DEFAULT_STATIC_PAYLOAD_BYTES; i++) 
    {
        rx_payload[i] = rx_buf[i + 1];
    }
    return NRF24L01P_OK;
}

nrf24l01p_status_t nrf24l01p_write_tx_fifo(const uint8_t *tx_payload) 
{
    if (tx_payload == NULL) 
    {
        //printf("Invalid payload pointer in write_tx_fifo\n");
        return NRF24L01P_ERR_INVALID_PARAM;
    }

    uint8_t tx_buf[NRF24L01P_DEFAULT_STATIC_PAYLOAD_BYTES + 1];
    uint8_t rx_buf[NRF24L01P_DEFAULT_STATIC_PAYLOAD_BYTES + 1];
    tx_buf[0] = NRF24L01P_CMD_W_TX_PAYLOAD;
    for (int i = 0; i < NRF24L01P_DEFAULT_STATIC_PAYLOAD_BYTES; i++) 
    {
        tx_buf[i + 1] = tx_payload[i];
    }

    nrf24l01p_status_t status = spi_transfer(tx_buf, rx_buf, NRF24L01P_DEFAULT_STATIC_PAYLOAD_BYTES + 1);

    if (status != NRF24L01P_OK) 
    {
        //printf("SPI error in write_tx_fifo\n");
        return status;
    }
		
    return NRF24L01P_OK;
}

nrf24l01p_status_t nrf24l01p_flush_rx_fifo(void) 
{
    uint8_t tx_buf[1] = {NRF24L01P_CMD_FLUSH_RX};
    uint8_t rx_buf[1];

    nrf24l01p_status_t status = spi_transfer(tx_buf, rx_buf, 1);
    
    if (status != NRF24L01P_OK) 
    {
        //printf("SPI error in flush_rx_fifo\n");
        return status;
    }
		
    return NRF24L01P_OK;
}

nrf24l01p_status_t nrf24l01p_flush_tx_fifo(void) 
{
    uint8_t tx_buf[1] = {NRF24L01P_CMD_FLUSH_TX};
    uint8_t rx_buf[1];

    nrf24l01p_status_t status = spi_transfer(tx_buf, rx_buf, 1);

    if (status != NRF24L01P_OK) 
    {
        //printf("SPI error in flush_tx_fifo\n");
        return status;
    }
    return NRF24L01P_OK;
}

uint8_t nrf24l01p_get_status(void) 
{
    uint8_t tx_buf[1] = {NRF24L01P_CMD_NOP};
    uint8_t rx_buf[1];

    nrf24l01p_status_t status = spi_transfer(tx_buf, rx_buf, 1);

    if (status != NRF24L01P_OK) 
    {
        //printf("SPI error in get_status\n");
        return 0xFF;
    }
    return rx_buf[0];
}

uint8_t nrf24l01p_get_fifo_status(void) 
{
    uint8_t value;
    if (read_register(NRF24L01P_REG_FIFO_STATUS, &value) != NRF24L01P_OK) 
    {
        //printf("Failed to read FIFO status\n");
        return 0xFF;
    }
    return value;
}

nrf24l01p_status_t nrf24l01p_rx_set_payload_width(uint8_t bytes) 
{
    if (bytes > 32) 
    {
        //printf("Invalid payload width: %d\n", bytes);
        return NRF24L01P_ERR_INVALID_PARAM;
    }
    return write_register(NRF24L01P_REG_RX_PW_P0, bytes);
}

nrf24l01p_status_t nrf24l01p_clear_rx_dr(void) 
{
    uint8_t status = nrf24l01p_get_status();
    if (status == 0xFF) 
    {
        //printf("SPI error in clear_rx_dr\n");
        return NRF24L01P_ERR_SPI;
    }

    status |= (1 << NRF24L01P_STATUS_RX_DR_BIT);
    return write_register(NRF24L01P_REG_STATUS, status);
}

nrf24l01p_status_t nrf24l01p_clear_tx_ds(void) 
{
    uint8_t status = nrf24l01p_get_status();
    if (status == 0xFF) 
    {
        //printf("SPI error in clear_tx_ds\n");
        return NRF24L01P_ERR_SPI;
    }

    status |= (1 << NRF24L01P_STATUS_TX_DS_BIT);
    return write_register(NRF24L01P_REG_STATUS, status);
}

nrf24l01p_status_t nrf24l01p_clear_max_rt(void) 
{
    uint8_t status = nrf24l01p_get_status();
    if (status == 0xFF) 
    {
        //printf("SPI error in clear_max_rt\n");
        return NRF24L01P_ERR_SPI;
    }

    status |= (1 << NRF24L01P_STATUS_MAX_RT_BIT);
    return write_register(NRF24L01P_REG_STATUS, status);
}

nrf24l01p_status_t nrf24l01p_power_up(void) 
{
    uint8_t config;
    nrf24l01p_status_t status = read_register(NRF24L01P_REG_CONFIG, &config);
    if (status != NRF24L01P_OK) return status;

    config |= (1 << NRF24L01P_CONFIG_PWR_UP_BIT);
    return write_register(NRF24L01P_REG_CONFIG, config);
}

nrf24l01p_status_t nrf24l01p_power_down(void) 
{
    uint8_t config;
    nrf24l01p_status_t status = read_register(NRF24L01P_REG_CONFIG, &config);
    if (status != NRF24L01P_OK) return status;

    config &= ~(1 << NRF24L01P_CONFIG_PWR_UP_BIT);
    return write_register(NRF24L01P_REG_CONFIG, config);
}

nrf24l01p_status_t nrf24l01p_set_crc_length(nrf24l01p_crc_length_t length) 
{
    if (length != NRF24L01P_CRC_1_BYTE && length != NRF24L01P_CRC_2_BYTES) 
    {
        //printf("Invalid CRC length: %d\n", length);
        return NRF24L01P_ERR_INVALID_PARAM;
    }

    uint8_t config;
    nrf24l01p_status_t status = read_register(NRF24L01P_REG_CONFIG, &config);
    if (status != NRF24L01P_OK) return status;

    config |= (1 << NRF24L01P_CONFIG_EN_CRC); // Enable CRC
    if (length == NRF24L01P_CRC_1_BYTE) 
    {
        config &= ~(1 << NRF24L01P_CONFIG_CRCO_BIT);
    } 
    else 
    {
        config |= (1 << NRF24L01P_CONFIG_CRCO_BIT);
    }
    return write_register(NRF24L01P_REG_CONFIG, config);
}

nrf24l01p_status_t nrf24l01p_set_address_width(nrf24l01p_address_width_t width) 
{
    if (width < NRF24L01P_ADDRESS_WIDTH_3_BYTES || width > NRF24L01P_ADDRESS_WIDTH_5_BYTES) 
    {
        //printf("Invalid address width: %d\n", width);
        return NRF24L01P_ERR_INVALID_PARAM;
    }
    return write_register(NRF24L01P_REG_SETUP_AW, width - 2);
}

nrf24l01p_status_t nrf24l01p_set_auto_retransmit_count(nrf24l01p_retransmit_count_t count) 
{
    if (count > NRF24L01P_RETRANSMIT_COUNT_15) 
    {
        //printf("Invalid retransmit count: %d\n", count);
        return NRF24L01P_ERR_INVALID_PARAM;
    }

    uint8_t setup_retr;
    nrf24l01p_status_t status = read_register(NRF24L01P_REG_SETUP_RETR, &setup_retr);
    if (status != NRF24L01P_OK) return status;

    setup_retr = (setup_retr & 0xF0) | (count & 0x0F);
    return write_register(NRF24L01P_REG_SETUP_RETR, setup_retr);
}

nrf24l01p_status_t nrf24l01p_set_auto_retransmit_delay(nrf24l01p_retransmit_delay_t delay) 
{
    if (delay < NRF24L01P_RETRANSMIT_DELAY_250US || delay > NRF24L01P_RETRANSMIT_DELAY_4000US || (delay % 250 != 0)) 
    {
        //printf("Invalid retransmit delay: %d\n", delay);
        return NRF24L01P_ERR_INVALID_PARAM;
    }

    uint8_t setup_retr;
    nrf24l01p_status_t status = read_register(NRF24L01P_REG_SETUP_RETR, &setup_retr);
    if (status != NRF24L01P_OK) return status;

    setup_retr = (setup_retr & 0x0F) | (((delay / 250) - 1) << 4);
    return write_register(NRF24L01P_REG_SETUP_RETR, setup_retr);
}

nrf24l01p_status_t nrf24l01p_set_rf_channel(nrf24l01p_rf_channel_t channel) 
{
    if (channel < 2400 || channel > 2525) 
    {
        //printf("Invalid RF channel: %d\n", channel);
        return NRF24L01P_ERR_INVALID_PARAM;
    }
    return write_register(NRF24L01P_REG_RF_CH, channel - 2400);
}

nrf24l01p_status_t nrf24l01p_set_rf_tx_output_power(nrf24l01p_output_power_t power) 
{
    if (power > NRF24L01P_PWR_0DBM) 
    {
        //printf("Invalid power level: %d\n", power);
        return NRF24L01P_ERR_INVALID_PARAM;
    }

    uint8_t rf_setup;
    nrf24l01p_status_t status = read_register(NRF24L01P_REG_RF_SETUP, &rf_setup);
    if (status != NRF24L01P_OK) return status;

    rf_setup = (rf_setup & ~NRF24L01P_RF_SETUP_RF_PWR_MSK) | (power << 1);
    return write_register(NRF24L01P_REG_RF_SETUP, rf_setup);
}

nrf24l01p_status_t nrf24l01p_set_rf_data_rate(nrf24l01p_data_rate_t data_rate) 
{
    if (data_rate > NRF24L01P_DATA_RATE_250KBPS) 
    {
        //printf("Invalid data rate: %d\n", data_rate);
        return NRF24L01P_ERR_INVALID_PARAM;
    }

    uint8_t rf_setup;
    nrf24l01p_status_t status = read_register(NRF24L01P_REG_RF_SETUP, &rf_setup);
    if (status != NRF24L01P_OK) return status;

    rf_setup &= ~(1 << NRF24L01P_RF_SETUP_RF_DR_LOW_BIT | 1 << NRF24L01P_RF_SETUP_RF_DR_HIGH_BIT);
    switch (data_rate) 
    {
        case NRF24L01P_DATA_RATE_1MBPS:
            break;
        case NRF24L01P_DATA_RATE_2MBPS:
            rf_setup |= (1 << NRF24L01P_RF_SETUP_RF_DR_HIGH_BIT);
            break;
        case NRF24L01P_DATA_RATE_250KBPS:
            rf_setup |= (1 << NRF24L01P_RF_SETUP_RF_DR_LOW_BIT);
            break;
    }
    return write_register(NRF24L01P_REG_RF_SETUP, rf_setup);
}

nrf24l01p_status_t nrf24l01p_set_rx_address(uint8_t pipe, const uint8_t *address, nrf24l01p_address_width_t length) 
{
    if (pipe > 5 || address == NULL || length < NRF24L01P_ADDRESS_WIDTH_3_BYTES || length > NRF24L01P_ADDRESS_WIDTH_5_BYTES) 
    {
        //printf("Invalid parameters in set_rx_address: pipe=%d, length=%d\n", pipe, length);
        return NRF24L01P_ERR_INVALID_PARAM;
    }

    uint8_t reg = NRF24L01P_REG_RX_ADDR_P0 + pipe;
    uint8_t tx_buf[6];
    uint8_t rx_buf[6];
    tx_buf[0] = NRF24L01P_CMD_W_REGISTER | (reg & 0x1F);
    for (int i = 0; i < length; i++) 
    {
        tx_buf[i + 1] = address[i];
    }

    nrf24l01p_status_t status = spi_transfer(tx_buf, rx_buf, length + 1);

    if (status != NRF24L01P_OK) 
    {
        //printf("SPI error in set_rx_address for pipe %d\n", pipe);
        return status;
    }

    // Verify address
    uint8_t read_addr[5];
    tx_buf[0] = NRF24L01P_CMD_R_REGISTER | (reg & 0x1F);
    for (int i = 1; i <= length; i++) 
    {
        tx_buf[i] = 0xFF;
    }
    
    status = spi_transfer(tx_buf, rx_buf, length + 1);
    
    if (status != NRF24L01P_OK) 
    {
        //printf("SPI error in set_rx_address verification for pipe %d\n", pipe);
        return status;
    }
    for (int i = 0; i < length; i++) 
    {
        read_addr[i] = rx_buf[i + 1];
        if (read_addr[i] != address[i]) 
        {
            //printf("Address verification failed for pipe %d: expected %02X, got %02X\n", pipe, address[i], read_addr[i]);
            return NRF24L01P_ERR_SPI;
        }
    }
    return NRF24L01P_OK;
}

nrf24l01p_status_t nrf24l01p_set_tx_address(const uint8_t *address, nrf24l01p_address_width_t length) 
{
    if (address == NULL || length < NRF24L01P_ADDRESS_WIDTH_3_BYTES || length > NRF24L01P_ADDRESS_WIDTH_5_BYTES) 
    {
        //printf("Invalid parameters in set_tx_address: length=%d\n", length);
        return NRF24L01P_ERR_INVALID_PARAM;
    }

    uint8_t tx_buf[6];
    uint8_t rx_buf[6];
    tx_buf[0] = NRF24L01P_CMD_W_REGISTER | NRF24L01P_REG_TX_ADDR;
    for (int i = 0; i < length; i++) 
    {
        tx_buf[i + 1] = address[i];
    }

    nrf24l01p_status_t status = spi_transfer(tx_buf, rx_buf, length + 1);

    if (status != NRF24L01P_OK) 
    {
        //printf("SPI error in set_tx_address\n");
        return status;
    }

    // Verify address
    uint8_t read_addr[5];
    tx_buf[0] = NRF24L01P_CMD_R_REGISTER | NRF24L01P_REG_TX_ADDR;
    for (int i = 1; i <= length; i++) 
    {
        tx_buf[i] = 0xFF;
    }
    
    status = spi_transfer(tx_buf, rx_buf, length + 1);
		
    if (status != NRF24L01P_OK) 
    {
        //printf("SPI error in set_tx_address verification\n");
        return status;
    }
		
    for (int i = 0; i < length; i++) 
    {
        read_addr[i] = rx_buf[i + 1];
        if (read_addr[i] != address[i]) 
        {
            //printf("TX address verification failed: expected %02X, got %02X\n", address[i], read_addr[i]);
            return NRF24L01P_ERR_SPI;
        }
    }
    return NRF24L01P_OK;
}
