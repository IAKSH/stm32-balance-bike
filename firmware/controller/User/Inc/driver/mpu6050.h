#ifndef __MPU6050_H__
#define __MPU6050_H__

#include "main.h"

// #define MPU6050_ADDR         0x68  // 7位地址

// //部分宏定义数据
// //MPU6050 AD0控制脚
 
// //#define MPU_ACCEL_OFFS_REG		0X06	//accel_offs寄存器,可读取版本号,寄存器手册未提到
// //#define MPU_PROD_ID_REG			0X0C	//prod id寄存器,在寄存器手册未提到
// #define MPU_SELF_TESTX_REG		0X0D	//自检寄存器X
// #define MPU_SELF_TESTY_REG		0X0E	//自检寄存器Y
// #define MPU_SELF_TESTZ_REG		0X0F	//自检寄存器Z
// #define MPU_SELF_TESTA_REG		0X10	//自检寄存器A
// #define MPU_SAMPLE_RATE_REG		0X19	//采样频率分频器
// #define MPU_CFG_REG				0X1A	//配置寄存器
// #define MPU_GYRO_CFG_REG		0X1B	//陀螺仪配置寄存器
// #define MPU_ACCEL_CFG_REG		0X1C	//加速度计配置寄存器
// #define MPU_MOTION_DET_REG		0X1F	//运动检测阀值设置寄存器
 
// #define MPU_ACCEL_XOUTH_REG		0X3B	//加速度值,X轴高8位寄存器
// #define MPU_ACCEL_XOUTL_REG		0X3C	//加速度值,X轴低8位寄存器
// #define MPU_ACCEL_YOUTH_REG		0X3D	//加速度值,Y轴高8位寄存器
// #define MPU_ACCEL_YOUTL_REG		0X3E	//加速度值,Y轴低8位寄存器
// #define MPU_ACCEL_ZOUTH_REG		0X3F	//加速度值,Z轴高8位寄存器
// #define MPU_ACCEL_ZOUTL_REG		0X40	//加速度值,Z轴低8位寄存器
 
// #define MPU_TEMP_OUTH_REG		0X41	//温度值高八位寄存器
// #define MPU_TEMP_OUTL_REG		0X42	//温度值低8位寄存器
 
// #define MPU_GYRO_XOUTH_REG		0X43	//陀螺仪值,X轴高8位寄存器
// #define MPU_GYRO_XOUTL_REG		0X44	//陀螺仪值,X轴低8位寄存器
// #define MPU_GYRO_YOUTH_REG		0X45	//陀螺仪值,Y轴高8位寄存器
// #define MPU_GYRO_YOUTL_REG		0X46	//陀螺仪值,Y轴低8位寄存器
// #define MPU_GYRO_ZOUTH_REG		0X47	//陀螺仪值,Z轴高8位寄存器
// #define MPU_GYRO_ZOUTL_REG		0X48	//陀螺仪值,Z轴低8位寄存器
 
// #define MPU_I2CMST_DELAY_REG	0X67	//IIC主机延时管理寄存器
// #define MPU_SIGPATH_RST_REG		0X68	//信号通道复位寄存器
// #define MPU_MDETECT_CTRL_REG	0X69	//运动检测控制寄存器
// #define MPU_USER_CTRL_REG		0X6A	//用户控制寄存器
// #define MPU_PWR_MGMT1_REG		0X6B	//电源管理寄存器1
// #define MPU_PWR_MGMT2_REG		0X6C	//电源管理寄存器2 

// #define MPU_DEVICE_ID_REG		0X75	//器件ID寄存器
 

// void mpu6050_init(void);
// void mpu6050_read_accel_data(void);
// void mpu6050_read_gyro_data(void);


#define DEFAULT_MPU_HZ 200

#define Q30  1073741824.0f


void mpu6050_init(void);
int mpu6050_dmp_init(void);
int mpu6050_dmp_get_data(float *pitch, float *roll, float *yaw);

void mpu6050_task(void *argument);
#endif
