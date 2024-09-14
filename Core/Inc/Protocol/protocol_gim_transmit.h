/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : protocol_gim_transmit.h
 *  Description  : This file is for transmit communication
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:18:52
 *  LastEditTime : 2023-08-22 10:04:27
 */


#ifndef PROTOCOL_GIM_TRANSMIT_H
#define PROTOCOL_GIM_TRANSMIT_H

#ifdef __cplusplus
extern "C" {
#endif 

#include "stm32f4xx_hal.h"

#define Const_Protocol_Gim_Transmit_BUFF_SIZE 3

typedef struct {
    uint32_t (*bus_func)(uint8_t *buff);
} Protocol_GimSendEntry;


extern Protocol_GimSendEntry ProtocolCmd_GimSend[Const_Protocol_Gim_Transmit_BUFF_SIZE];



#endif

#ifdef __cplusplus
}
#endif
