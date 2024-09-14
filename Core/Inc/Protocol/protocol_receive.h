/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : protocol_receive.h
 *  Description  : This file is for receive communication
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:10:43
 *  LastEditTime : 2023-08-09 00:24:32
 */


#ifndef PROTOCOL_RECEIVE_H
#define PROTOCOL_RECEIVE_H

#ifdef __cplusplus
extern "C" {
#endif 

#include "stm32f4xx_hal.h"

#define Const_Protocol_Receive_BUFF_SIZE 5


typedef struct {
    uint32_t (*bus_func)(uint8_t *buff);
} Protocol_ReceiveEntry;

extern Protocol_ReceiveEntry ProtocolCmd_Receive[Const_Protocol_Receive_BUFF_SIZE];


#endif

#ifdef __cplusplus
}
#endif
