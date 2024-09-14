/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : protocol_common.h
 *  Description  : This file contains Bus communication control function
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:03:07
 *  LastEditTime : 2023-08-22 10:06:52
 */


#ifndef PROTOCOL_COMMON_H
#define PROTOCOL_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif 

#include "stm32f4xx_hal.h"

#define PROTOCOL_SEND_BUFF_LEN  256
#define PROTOCOL_RECEIVE_BUFF_LEN 128

typedef enum {
    Protocol_STATE_NULL      = 0,
    Protocol_STATE_CONNECTED = 1,
    Protocol_STATE_LOST      = 2,
    Protocol_STATE_ERROR     = 3,
    Protocol_STATE_PENDING   = 4
} Protocol_CommStateEnum;

typedef struct {
    float pitch;
    float yaw;
    float yaw_total;
    float gyro_z;
    float temperature;
    uint8_t motor_mode;

} Protocol_GimbalDataTypeDef;

typedef struct {
    Protocol_CommStateEnum state;
    Protocol_GimbalDataTypeDef gimbal;

    uint8_t usb_watchBuff[PROTOCOL_RECEIVE_BUFF_LEN];
    uint8_t usb_sendBuff[PROTOCOL_SEND_BUFF_LEN];

    uint8_t uart_watchBuff[PROTOCOL_RECEIVE_BUFF_LEN];
    uint8_t uart_sendBuff[PROTOCOL_SEND_BUFF_LEN];
    uint32_t last_rx_tick;
    float rx_dt;

    uint32_t last_tx_tick;
    float tx_dt;

    uint32_t last_gim_rx_tick;
    float gim_rx_dt;
} Protocol_DataTypeDef;


extern Protocol_DataTypeDef Protocol_Data;


void Protocol_InitProtocol(void);
Protocol_DataTypeDef* Protocol_GetBusDataPtr(void);
uint8_t Protocol_IsProtocolOffline(void);
void Protocol_SendBlockError(void);
void Protocol_ResetProtocolData(void);
static uint8_t Protocol_UartVerify(uint8_t buff[], uint32_t rxdatalen);
void Protocol_SendProtocolData(void);
void Protocol_DecodeData(uint8_t buff[], uint32_t rxdatalen);
void Protocol_UartDecode(uint8_t buff[], uint32_t rxdatalen);
void Protocol_UartCallBack(UART_HandleTypeDef* huart);

#endif

#ifdef __cplusplus
}
#endif
