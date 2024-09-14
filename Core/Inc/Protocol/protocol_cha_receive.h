/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : protocol_cha_recevie.h
 *  Description  : This file is for receive communication
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:10:43
 *  LastEditTime : 2023-08-04 19:14:58
 */


#ifndef PROTOCOL_CHA_RECEIVE_H
#define PROTOCOL_CHA_RECEIVE_H

#ifdef __cplusplus
extern "C" {
#endif 

#include "stm32f4xx_hal.h"

#define Const_Protocol_Cha_Receive_BUFF_SIZE 3


typedef struct {
    uint32_t (*bus_func)(uint8_t *buff);
} Protocol_ChaReceiveEntry;

extern Protocol_ChaReceiveEntry ProtocolCmd_ChaReceive[Const_Protocol_Cha_Receive_BUFF_SIZE];


#endif

#ifdef __cplusplus
}
#endif
