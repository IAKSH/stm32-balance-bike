#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "nrf24l01p.h"
#ifndef WIRELESS_TX_ONLY
#include <cmsis_os2.h>
#endif

#define WIRELESS_TIMEOUT_MS 100

typedef enum {
    COMMAND_MOVE,COMMAND_VOLT,COMMAND_CAM_ROTATE,COMMAND_CAM_SHOT,COMMAND_PID
} __CommandType;

typedef enum {
    GRAY_SCALE,R8G8B8,R5G6B5,R5G5B5,R4G4B4
} __ColorFormat;

typedef uint8_t CommandType;
typedef uint8_t ColorFormat;

typedef struct {
    uint8_t version;
    CommandType type;
    union {
        struct {
            int speed[2];
        } move;
        struct {
            float angle[2];
        } cam_rotate;
        struct {
            uint16_t size[2];
            uint8_t color_format;
        } cam_shot;
        struct {
            bool write;
            bool copy_state;
            struct val {
                float kp,ki,kd;
            } angle_pid,speed_pid;
        } pid;
    } payload;
} CommandPacket;


typedef struct {
    NRF24L01PState* nrf24;
#ifndef WIRELESS_TX_ONLY
    struct {
        osMessageQueueId_t (*get_queue)(void);
    } __impl;
#endif
} WirelessState;

void wireless_create_state(WirelessState* s);
void wireless_use_state(WirelessState* s);

void wireless_irq(void);
void wireless_send(void* data,uint16_t len);

#ifndef WIRELESS_TX_ONLY
void wireless_receive(void* data,uint16_t len);
#endif

#ifdef __cplusplus
}
#endif