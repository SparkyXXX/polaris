/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : protocol_transmit.h
 *  Description  : This file is for transmit communication
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:18:52
 *  LastEditTime : 2023-08-15 23:17:31
 */


#ifndef PROTOCOL_TRANSMIT_H
#define PROTOCOL_TRANSMIT_H

#ifdef __cplusplus
extern "C" {
#endif 

#include "stm32f4xx_hal.h"

#define Const_Protocol_Transmit_BUFF_SIZE 9

typedef struct {
    uint32_t (*bus_func)(uint8_t *buff);
} Protocol_SendEntry;


extern Protocol_SendEntry ProtocolCmd_Send[Const_Protocol_Transmit_BUFF_SIZE];



#endif

#ifdef __cplusplus
}
#endif
