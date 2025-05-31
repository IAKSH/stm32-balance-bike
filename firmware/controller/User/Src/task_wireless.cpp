#include "main.h"
#include "spi.h"
#include "tasks.h"
#include "data_proc.h"
#include <cstdio>
#include <drivers/nrf24l01p/nrf24l01p.h>

static uint8_t tx_address[5] = {0x0, 0x0, 0x0, 0x0, 0x01};

static void cs_high(void) {
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);
}

static void cs_low(void) {
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET);
}

static void ce_high(void) {
    HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_SET);
}

static void ce_low(void) {
    HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_RESET);
}

static void spi_transmit(uint8_t *data, uint16_t len) {
    HAL_SPI_Transmit(&hspi1, data, len, 2000);
}

static void spi_receive(uint8_t *data, uint16_t len) {
    HAL_SPI_Receive(&hspi1, data, len, 2000);
}

static void spi_trans_receive(uint8_t *tx, uint8_t *rx, uint16_t len) {
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, len, 2000);
}

static NRF24L01PState nrf24_state;
static WirelessState wireless_state;

static void setup() {
    nrf24l01p_create_state(&nrf24_state);
    wireless_create_state(&wireless_state);

    nrf24_state.__impl.ce_high = ce_high;
    nrf24_state.__impl.ce_low = ce_low;
    nrf24_state.__impl.cs_high = cs_high;
    nrf24_state.__impl.cs_low = cs_low;
    nrf24_state.__impl.spi_receive = spi_receive;
    nrf24_state.__impl.spi_transmit = spi_transmit;
    nrf24_state.__impl.spi_transmit_receive = spi_trans_receive;
    nrf24_state.__impl.get_tick = HAL_GetTick;

    wireless_state.nrf24 = &nrf24_state;

    nrf24l01p_use_state(&nrf24_state);
    wireless_use_state(&wireless_state);

    nrf24l01p_set_mode_tx(2500, NRF24L01P_AIR_DATA_RATE_1Mbps);

    if (!nrf24l01p_check()) {
        printf("nrf24l01+ no response\n");
        Error_Handler(); 
    }

    nrf24l01p_set_tx_addr(tx_address,5);
    nrf24l01p_set_rx_addr(0,tx_address,5);
}

void wireless_send_task(void *argument) {
    setup();
    while (1) {
        CommandPacket command = {
            .version = 0x00,
            .type = COMMAND_MOVE
        };

        data_packaing(&command);

        wireless_send(&command, sizeof(command));
        osDelay(100);
    }
}