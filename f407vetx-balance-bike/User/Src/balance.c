#include "motor.h"
#include "mpu6050.h"
#include "tasks.h"
#include "cmsis_os.h"
#include "main.h"
#include "math.h"
#include "pid.h"

float mpu6050_pitch,mpu_6050_roll,mpu_6050_yaw;
float motorSpeedA,motorSpeedB;

#define ANGLE_KP 350.0f
#define ANGLE_KI 0.25f
#define ANGLE_KD 6.0f 

#define SPEED_KP 1.6f
#define SPEED_KI 7.5f// 用这个刹车
#define SPEED_KD 0.0f

/* 初始化 PID 参数，此处参数需要根据系统实际情况做具体调试 */ 
PID pid_angle  = {
    .Kp = ANGLE_KP, .Ki = ANGLE_KI, .Kd =ANGLE_KD,      // 外环：车体倾角控制
    .target = 0.0f,
    .error = 0.0f,
    .integral = 0.0f,
    .output = 0.0f
};

PID pid_speed = {
    .Kp = SPEED_KP, .Ki = SPEED_KI, .Kd =SPEED_KD,        // 内环：右轮速度控制
    .target = 0.0f,
    .error = 0.0f,
    .integral = 0.0f,
    .output = 0.0f
};

void balance_task(void *argument)
{
    mpu6050_init();
    osEventFlagsSet(event,EVENT_FLAG_GYRO_INITIALIZED);

    motor_init();
    motor_set_dirtect(MOTOR_A,MOTOR_FORWARD);
    motor_set_dirtect(MOTOR_B,MOTOR_FORWARD);

    while (1)
    {
        //获取mpu6050数据
        osSemaphoreAcquire(mpu6050DataReadySemaphore,osWaitForever);
        mpu6050_dmp_get_data(&mpu6050_pitch,&mpu_6050_roll,&mpu_6050_yaw);

        //更新电机速度
        motor_update_speed(&motorSpeedA,&motorSpeedB);

        //跌倒保护
        if(fabs(mpu6050_pitch)>=40.0f)
        {
            motor_set_dirtect(MOTOR_A,MOTOR_STOP);
            motor_set_dirtect(MOTOR_B,MOTOR_STOP);
            continue;
        }

        // 设定采样周期（dt，单位：秒），该值需要依据实际任务周期确定
        float dt=0.005f;     //设置采样频率

        float current_angle = mpu6050_pitch;

        /* 外环 PID 控制：目标倾角设为 0°（竖直状态），计算倾角修正量 */

        pid_angle.target = 0.0f;
        float angle_correctoin = pid_calc(&pid_angle,current_angle,dt);

        /* 内环 PID 控制：以外环输出作为左右轮目标速度
           注：如果系统是对称设计，两个轮子的 PID 参数可以一样 */

        float average_speed = (motorSpeedA +motorSpeedB)/2.0f;

        pid_speed.target = angle_correctoin;
        float speed = pid_calc(&pid_speed,average_speed,dt);

        float speed_a = speed;
        float speed_b =speed;

        if (speed_a > 7200.0f) speed_a = 7200.0f;
        if (speed_b < -7200.0f) speed_b = -7200.0f;

        motor_set_speed(MOTOR_A,(int)speed_a);
        motor_set_speed(MOTOR_B,(int)speed_b);
    }
}