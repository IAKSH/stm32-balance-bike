#include "main.h"
#include <utils/pid.h>
#include <drivers/nrf24l01p/nrf24l01p.h>
#include <protocols/wireless.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#define NRF24L01P_SPI                     (&hspi2)
#define NRF24L01P_SPI_CS_PIN_PORT         WIRELESS_CSN_GPIO_Port 
#define NRF24L01P_SPI_CS_PIN_NUMBER       WIRELESS_CSN_Pin
#define NRF24L01P_CE_PIN_PORT             WIRELESS_CE_GPIO_Port
#define NRF24L01P_CE_PIN_NUMBER           WIRELESS_CE_Pin
#define NRF24L01P_IRQ_PIN_PORT            EXIT7_WIRELESS_IRQ_GPIO_Port
#define NRF24L01P_IRQ_PIN_NUMBER          EXIT7_WIRELESS_IRQ_Pin

static uint8_t rx_address[5] = {0x0, 0x0, 0x0, 0x0, 0x01};

static void command_move_handler(CommandPacket command) {
    printf("speed: {%d, %d}\n", command.payload.move.speed[0], command.payload.move.speed[1]);
}

static void command_pid_handler(CommandPacket command) {
    if(command.payload.pid.write) {
        extern PID pid_speed,pid_angle;
        pid_speed.Kp = command.payload.pid.speed_pid.kp;
        pid_speed.Ki = command.payload.pid.speed_pid.ki;
        pid_speed.Kd = command.payload.pid.speed_pid.kd;

        pid_angle.Kp = command.payload.pid.angle_pid.kp;
        pid_angle.Ki = command.payload.pid.angle_pid.ki;
        pid_angle.Kd = command.payload.pid.angle_pid.kd;
    }
    else {
        nrf24l01p_set_mode_tx(2500, NRF24L01P_AIR_DATA_RATE_1Mbps);
        nrf24l01p_set_tx_addr(rx_address,5);

        // TODO:
        

        nrf24l01p_set_mode_rx(2500, NRF24L01P_AIR_DATA_RATE_1Mbps);
        //nrf24l01p_set_rx_addr(0,rx_address,5);
    }
}

static void cs_high(void) {
    HAL_GPIO_WritePin(NRF24L01P_SPI_CS_PIN_PORT, NRF24L01P_SPI_CS_PIN_NUMBER, GPIO_PIN_SET);
}

static void cs_low(void) {
    HAL_GPIO_WritePin(NRF24L01P_SPI_CS_PIN_PORT, NRF24L01P_SPI_CS_PIN_NUMBER, GPIO_PIN_RESET);
}

static void ce_high(void) {
    HAL_GPIO_WritePin(NRF24L01P_CE_PIN_PORT, NRF24L01P_CE_PIN_NUMBER, GPIO_PIN_SET);
}

static void ce_low(void) {
    HAL_GPIO_WritePin(NRF24L01P_CE_PIN_PORT, NRF24L01P_CE_PIN_NUMBER, GPIO_PIN_RESET);
}

static void spi_transmit(uint8_t *data, uint16_t len) {
    HAL_SPI_Transmit(NRF24L01P_SPI, data, len, 2000);
}

static void spi_receive(uint8_t *data, uint16_t len) {
    HAL_SPI_Receive(NRF24L01P_SPI, data, len, 2000);
}

static void spi_trans_receive(uint8_t *tx, uint8_t *rx, uint16_t len) {
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, tx, rx, len, 2000);
}

osMessageQueueId_t get_command_queue(void) {
    return command_queue;
}

void control_task(void* arg) {
    NRF24L01PState nrf24_state;
    WirelessState wireless_state;
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
    wireless_state.__impl.get_queue = get_command_queue;

    nrf24l01p_use_state(&nrf24_state);
    wireless_use_state(&wireless_state);

    nrf24l01p_set_mode_rx(2500, NRF24L01P_AIR_DATA_RATE_1Mbps);

    if (!nrf24l01p_check()) {
        printf("nrf24l01+ no response\n");
        Error_Handler();
    }

    nrf24l01p_set_rx_addr(0,rx_address,5);

    CommandPacket command;

    while(1) {
        wireless_receive(&command,sizeof(CommandPacket));
        switch(command.type) {
        case COMMAND_MOVE:
            command_move_handler(command);
            break;
        case COMMAND_PID:
            command_pid_handler(command);
            break;
        default:
            break;
        }
    }
}
