/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : app_communicate.c
 *  Description  : Communication Application
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:44:07
 *  LastEditTime : 2023-08-23 17:40:51
 */


#include "app_communicate.h"
#include "protocol_common.h"
#include "util_usb.h"
#include "sys_dwt.h"



void Comm_Task(void const * argument) {
    if (Main_Type != Main_Head)
	    osDelay(500);
    forever {
        Protocol_SendProtocolData();
      osDelay(2);
    }
}
