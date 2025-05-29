/*
#include "mpu6050.h"
#include "i2c.h"
#include "stdio.h"

uint8_t check;
uint8_t data;

HAL_StatusTypeDef mpu6050_write_register(uint16_t reg ,uint8_t data)
{
    HAL_StatusTypeDef sta=HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR << 1, reg, 1, &data, 1, 100);
    return sta;
}

void mpu6050_init(void)
{
    // 读取WHO_AM_I寄存器，确认设备存在
    if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR << 1, 0x75, 1, &check, 1, 100) != HAL_OK) {
        printf("MPU6050 not found!\n");
        return;
    }

    HAL_Delay(100);
    printf("WHO_AM_I: 0x%02X\r\n", check);

    if (check == 0x68) {
        // 复位MPU6050
        data=0x80;

        if (mpu6050_write_register(MPU_PWR_MGMT1_REG,0x08) != HAL_OK) {
            printf("Reset failed!\n");
            return;
        }
        HAL_Delay(100);

        if (mpu6050_write_register(MPU_PWR_MGMT1_REG,0x01) != HAL_OK) {
            printf("Wakeup failed!\n");
            return;
        }
        // 所有轴永不待机
        data=0x00;
        //HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR << 1, MPU_PWR_MGMT2_REG, 1, &data, 1, 100);
        mpu6050_write_register(MPU_PWR_MGMT2_REG,0x00);
        // 设置采样频率
        data=0x00;
        // HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR << 1, MPU_SAMPLE_RATE_REG, 1, &data, 1, 100);
        mpu6050_write_register(MPU_SAMPLE_RATE_REG,0x00);
        // 设置配置寄存器
        data=0x04;
        // HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR << 1, MPU_CFG_REG, 1, &data, 1, 100);
        mpu6050_write_register(MPU_CFG_REG,0x04);
        // 设置陀螺仪满量程范围，不进行自检
        data=0x18;
        //HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR << 1, MPU_GYRO_CFG_REG, 1, &data, 1, 100);
        mpu6050_write_register(MPU_GYRO_CFG_REG,0x18);
        // 设置加速度计满量程范围
        data=0x08;
        // HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR << 1, MPU_ACCEL_CFG_REG, 1, &data, 1, 100);
        mpu6050_write_register(MPU_GYRO_CFG_REG,0x08);

        printf("MPU6050 init OK\r\n");
    } else {
        printf("MPU6050 ID error!\n");
    }
}
int16_t accel_x, accel_y, accel_z;
void mpu6050_read_accel_data(void)
{

    uint8_t rec_data[6];

    if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR << 1, 0x3B, 1, rec_data, 6, 100) == HAL_OK) {
        accel_x = (int16_t)((rec_data[0] << 8) | rec_data[1]);
        accel_y = (int16_t)((rec_data[2] << 8) | rec_data[3]);
        accel_z = (int16_t)((rec_data[4] << 8) | rec_data[5]);
        printf("Accel: X=%d, Y=%d, Z=%d\r\n", accel_x, accel_y, accel_z);
    } else {
        printf("I2C Read Error!\n");
    }
}
int16_t gyro_x,gyro_y,gyro_z;
void mpu6050_read_gyro_data(void)
{
    uint8_t rec_data[6];
    if(HAL_I2C_Mem_Read(&hi2c1,MPU6050_ADDR<<1,MPU_GYRO_XOUTH_REG,1,rec_data,6,100) == HAL_OK)
    {
        gyro_x =(uint16_t)((rec_data[0]<<8) | rec_data[1]);
        gyro_y =(uint16_t)((rec_data[2]<<8) | rec_data[3]);
        gyro_z =(uint16_t)((rec_data[4]<<8) | rec_data[5]);
        printf("gyro_x = %d, gyro_y = %d, gyro_z = %d\r\n",gyro_x,gyro_y,gyro_z);

    }
    else
    {
        printf("I2C Read Error!\n");
    }
}

*/

// dmp 库

#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "math.h"
#include "stdio.h"
#include "cmsis_os.h"

void gyro_control_angle(void);

extern int angle;
extern float pitch, roll, yaw;

static signed char gyro_orientation[9] = {-1, 0, 0,
                                          0, -1, 0,
                                          0, 0, 1};

static unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7; // error
    return b;
}

static unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;

    return scalar;
}

static int run_self_test(void)
{
    int result;
    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);
    if (result == 0x3)
    {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
    }
    else
    {
        return -1;
    }

    return 0;
}

int mpu6050_dmp_init(void)
{
    int result;
    struct int_param_s int_param;

    // mpu init
    result = mpu_init(&int_param);
    printf("mpu_init: %d\r\n", result);
    if (result != 0)
    {
        return -1;
    }
    // 设置传感器
    result = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    printf("mpu_set_sensors: %d\r\n", result);
    if (result != 0)
    {
        return -2;
    }
    // 设置FIFO
    result = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    printf("mpu_configure_fifo: %d\r\n", result);
    if (result != 0)
    {
        return -3;
    }
    // 设置采样频率
    result = mpu_set_sample_rate(DEFAULT_MPU_HZ);
    printf("mpu_set_sample_rate: %d\r\n", result);
    if (result != 0)
    {
        return -4;
    }
    // dmp加载固件
    result = dmp_load_motion_driver_firmware();
    printf("dmp_load_motion_driver_firmware: %d\r\n", result);
    if (result != 0)
    {
        return -5;
    }
    // 设置陀螺仪方向
    result = dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
    printf("dmp_set_orientation: %d\r\n", result);
    if (result != 0)
    {
        return -6;
    }

    //设置dmp 功能
    result = dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
                                DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL |
                                DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL);
    printf("dmp_enable_feature: %d\r\n", result);
    if (result != 0)
    {
        return -7;
    }

    //设置输出速率
    result = dmp_set_fifo_rate(DEFAULT_MPU_HZ);
    printf("dmp_set_fifo_rate: %d\r\n", result);
    if (result != 0)
    {
        return -8;
    }

    //自检
    result = run_self_test();
    printf("run_self_test: %d\r\n", result);
    if (result != 0)
    {
        return -9;
    }

    //使能dmp
    result = mpu_set_dmp_state(1);
    printf("mpu_set_dmp_state: %d\r\n", result);
    if (result != 0)
    {
        return -10;
    }

    mpu_reset_fifo();  // 确保 FIFO 干净

    return 0;
}


int mpu6050_dmp_get_data(float *pitch, float *roll, float *yaw)
{
    float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
    short gyro[3];
    short accel[3];
    long quat[4];
    unsigned long timestamp;
    short sensors;
    unsigned char more;
    if (dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more))
    {
        return -1;
    }

    if (sensors & INV_WXYZ_QUAT)
    {
        q0 = quat[0] / Q30;
        q1 = quat[1] / Q30;
        q2 = quat[2] / Q30;
        q3 = quat[3] / Q30;

        *pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3;                                    // pitch
        *roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3;     // roll
        *yaw = atan2(2 * (q0 * q3 + q1 * q2), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.3; // yaw
    }

    return 0;
}

void mpu6050_init(void)
{
    int mpu6050_init_ret = mpu6050_dmp_init();
    if (mpu6050_init_ret != 0)
    {
        printf("MPU6050 init failed!,ret=%d\r\n",mpu6050_init_ret);
        while (mpu6050_init_ret)
        {
            osDelay(100);
            mpu6050_init_ret = mpu6050_dmp_init();
        }
    }
    else
    {
        printf("MPU6050 init success!\r\n");
    }
}

