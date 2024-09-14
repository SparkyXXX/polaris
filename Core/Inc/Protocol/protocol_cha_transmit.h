/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : protocol_cha_transmit.h
 *  Description  : This file is for transmit communication
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:18:52
 *  LastEditTime : 2023-08-21 23:36:02
 */


#ifndef PROTOCOL_CHA_TRANSMIT_H
#define PROTOCOL_CHA_TRANSMIT_H

#ifdef __cplusplus
extern "C" {
#endif 

#include "stm32f4xx_hal.h"

#define Const_Protocol_Cha_Transmit_BUFF_SIZE 1

typedef struct {
    uint32_t (*bus_func)(uint8_t *buff);
} Protocol_ChaSendEntry;


extern Protocol_ChaSendEntry ProtocolCmd_ChaSend[Const_Protocol_Cha_Transmit_BUFF_SIZE];



#endif

#ifdef __cplusplus
}
#endif
