/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : app_shell.h
 *  Description  : shell application
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:43:44
 *  LastEditTime : 2023-08-12 03:45:02
 */



#ifndef APP_SHELL_H
#define APP_SHELL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"

#define Const_Shell_RX_BUFF_LEN  64
#define Const_Shell_TX_BUFF_LEN 1024

typedef struct {
    uint8_t Shell_RxData[Const_Shell_RX_BUFF_LEN];
    uint8_t Shell_TxData[Const_Shell_TX_BUFF_LEN];
    uint32_t rx_len;
} Shell_DataTypeDef;

void Shell_Task(void const * argument);
void Shell_Init(void);
Shell_DataTypeDef* Shell_GetShellDataPtr(void);
void Shell_RXCallback(UART_HandleTypeDef* huart);

#endif

#ifdef __cplusplus
}

#endif
