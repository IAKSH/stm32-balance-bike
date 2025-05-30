#include "main.h"
#include "cmsis_os2.h"
#include "tasks.h"
#include "gyro.h"
#include "data_proc.h"
#include "stdio.h"

static uint8_t i2c_read(uint16_t dev_addr, uint16_t reg_addr, uint16_t data_size, uint8_t *p_data)
{
    // osMutexAcquire(i2c_bus_mutex, osWaitForever);
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(&hi2c1, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, p_data, data_size, 0xF);
    // osMutexRelease(i2c_bus_mutex);
    return ret;
}

static uint8_t i2c_write(uint16_t dev_addr, uint16_t reg_addr, uint16_t data_size, uint8_t *p_data)
{
    // osMutexAcquire(i2c_bus_mutex, osWaitForever);
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Write(&hi2c1, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, p_data, data_size, 0xF);
    // osMutexRelease(i2c_bus_mutex);
    return ret;
}

static void nop(void)
{
    __NOP();
}

#define NRF24L01P_SPI (&hspi1)
#define NRF24L01P_SPI_CS_PIN_PORT CSN_GPIO_Port
#define NRF24L01P_SPI_CS_PIN_NUMBER CSN_Pin
#define NRF24L01P_CE_PIN_PORT CE_GPIO_Port
#define NRF24L01P_CE_PIN_NUMBER CE_Pin
#define NRF24L01P_IRQ_PIN_PORT IRQ_GPIO_Port
#define NRF24L01P_IRQ_PIN_NUMBER IRQ_Pin

static uint8_t tx_address[5] = {0x0, 0x0, 0x0, 0x0, 0x01};

static void cs_high(void)
{
    HAL_GPIO_WritePin(NRF24L01P_SPI_CS_PIN_PORT, NRF24L01P_SPI_CS_PIN_NUMBER, GPIO_PIN_SET);
}

static void cs_low(void)
{
    HAL_GPIO_WritePin(NRF24L01P_SPI_CS_PIN_PORT, NRF24L01P_SPI_CS_PIN_NUMBER, GPIO_PIN_RESET);
}

static void ce_high(void)
{
    HAL_GPIO_WritePin(NRF24L01P_CE_PIN_PORT, NRF24L01P_CE_PIN_NUMBER, GPIO_PIN_SET);
}

static void ce_low(void)
{
    HAL_GPIO_WritePin(NRF24L01P_CE_PIN_PORT, NRF24L01P_CE_PIN_NUMBER, GPIO_PIN_RESET);
}

static void spi_transmit(uint8_t *data, uint16_t len)
{
    HAL_SPI_Transmit(NRF24L01P_SPI, data, len, 2000);
}

static void spi_receive(uint8_t *data, uint16_t len)
{
    HAL_SPI_Receive(NRF24L01P_SPI, data, len, 2000);
}

static void spi_trans_receive(uint8_t *tx, uint8_t *rx, uint16_t len)
{
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, tx, rx, len, 2000);
}

float mpu6050_pitch, mpu_6050_roll, mpu_6050_yaw;

void control_task(void *argument)
{
    GyroState state;
    gyro_create_state(&state);
    state.__impl.delay_ms = HAL_Delay;
    state.__impl.get_ms = HAL_GetTick;
    state.__impl.i2c_read = i2c_read;
    state.__impl.i2c_write = i2c_write;
    state.__impl.nop = nop;

    gyro_use_state(&state);

    int mpu6050_init_ret;
    while ((mpu6050_init_ret = gyro_init()) != 0)
    {
        printf("MPU6050 init failed, ret = %d\n", mpu6050_init_ret);
        osDelay(100);
    }

    osEventFlagsSet(mpu6050_init_event, EVENT_FLAG_GYRO_INITIALIZED);

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
    // wireless_state.__impl.get_queue = get_command_queue;

    nrf24l01p_use_state(&nrf24_state);
    wireless_use_state(&wireless_state);

    nrf24l01p_set_mode_tx(2500, NRF24L01P_AIR_DATA_RATE_1Mbps);

    if (!nrf24l01p_check())
    {
        printf("nrf24l01+ no response\n");
        Error_Handler();
    }

    printf("wireless init\n");

    nrf24l01p_set_tx_addr(tx_address, 5);
    nrf24l01p_set_rx_addr(0, tx_address, 5);

    CommandPacket command = {
        .version = 0x00,
        .type = COMMAND_MOVE};
    while (1)
    {
        // gyro_get_data(&mpu6050_pitch, &mpu_6050_roll, &mpu_6050_yaw);

        // data_packaing(&command);

        wireless_send(&command, sizeof(CommandPacket));
        osDelay(100);
    }
}