#include "system.h"
#include "adc.h"
#include "nrf24l01p.h"
#include "stdio.h"

uint8_t tx_address[5] = {0x0,0x0,0x0,0x0,0x01};

void system_init(void)
{
    HAL_ADCEx_Calibration_Start(&hadc1);

    nrf24l01p_set_mode_tx(2500, NRF24L01P_AIR_DATA_RATE_1Mbps);
    if (!nrf24l01p_check())
    {
        printf("nrf24l01p abnormal\n");
    }

    printf("test!\n");

    nrf24l01p_set_tx_addr(tx_address, 5);
    nrf24l01p_set_rx_addr(0, tx_address, 5);
}