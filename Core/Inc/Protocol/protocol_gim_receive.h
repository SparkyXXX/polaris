/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : protocol_gim_recevie.h
 *  Description  : This file is for receive communication
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:10:43
 *  LastEditTime : 2023-08-04 19:14:58
 */


#ifndef PROTOCOL_GIM_RECEIVE_H
#define PROTOCOL_GIM_RECEIVE_H

#ifdef __cplusplus
extern "C" {
#endif 

#include "stm32f4xx_hal.h"

#define Const_Protocol_Gim_Receive_BUFF_SIZE 1


typedef struct {
    uint32_t (*bus_func)(uint8_t *buff);
} Protocol_GimReceiveEntry;

extern Protocol_GimReceiveEntry ProtocolCmd_GimReceive[Const_Protocol_Gim_Receive_BUFF_SIZE];


#endif

#ifdef __cplusplus
}
#endif
