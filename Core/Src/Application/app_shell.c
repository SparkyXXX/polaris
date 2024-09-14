/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : app_shell.c
 *  Description  : shell application
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:43:29
 *  LastEditTime : 2023-08-24 17:45:51
 */


#include "app_shell.h"
#include "string.h"
#include "stdio.h"
#include "lib_str.h"
#include "semphr.h"
#include "task.h"
#include "sys_const.h"
#include "sys_cmd.h"
#include "sys_dwt.h"
#include "util_uart.h"
#include "FreeRTOS_CLI.h"

Shell_DataTypeDef ShellData;

void Shell_Task(void const * argument) {
    Shell_DataTypeDef* shell = Shell_GetShellDataPtr();

    char *pcOutputString;
    BaseType_t xReturned;
    static uint8_t login_flag = 0;
	pcOutputString = FreeRTOS_CLIGetOutputBuffer();
    Uart_SendMessageDMA(Const_Shell_UART_HANDLER, (uint8_t*)Shell_WelComMessage, (uint16_t)strlen(Shell_WelComMessage));
    
	forever {
		if (xSemaphoreTake(Shell_RxCMDHandle, 0) == pdTRUE) {
            uint32_t tx_len = 0;
            memset(shell->Shell_TxData, 0x00, Const_Shell_TX_BUFF_LEN);

            if (login_flag == 0) {
                if (strcmp(Str_RemoveCRLF((char*)shell->Shell_RxData, shell->rx_len), Shell_Password) == 0) {
                    login_flag = 1;
                    strncpy((char*)shell->Shell_TxData, Shell_LoginSuccess, (uint16_t)strlen(Shell_LoginSuccess));
		    	    tx_len += (uint16_t)strlen(Shell_LoginSuccess);   
                }
                else {
                    strncpy((char*)shell->Shell_TxData, Shell_LoginFailed, (uint16_t)strlen(Shell_LoginFailed));
		    	    tx_len += (uint16_t)strlen(Shell_LoginFailed);                      
                }
            }
            else {
                strncpy((char*)shell->Shell_TxData, (char*)shell->Shell_RxData, shell->rx_len);
                tx_len += shell->rx_len;
		        do {
		        	xReturned = FreeRTOS_CLIProcessCommand(Str_RemoveCRLF((char*)shell->Shell_RxData, shell->rx_len), pcOutputString, configCOMMAND_INT_MAX_OUTPUT_SIZE);
                    strncpy((char*)shell->Shell_TxData + tx_len, pcOutputString, (uint16_t)strlen(pcOutputString));
		        	tx_len += (uint16_t)strlen(pcOutputString);

		        } while(xReturned != pdFALSE);
                char *ret_path = CMD_GetNowPathStr();

                strncpy((char*)shell->Shell_TxData + tx_len, Shell_EndOfMessage, (uint16_t)strlen(Shell_EndOfMessage));
		        tx_len += (uint16_t)strlen(Shell_EndOfMessage);
                strncpy((char*)shell->Shell_TxData + tx_len, Shell_PolarRoot, (uint16_t)strlen(Shell_PolarRoot));
		        tx_len += (uint16_t)strlen(Shell_PolarRoot);
                strncpy((char*)shell->Shell_TxData + tx_len, ret_path, (uint16_t)strlen(ret_path));
		        tx_len += (uint16_t)strlen(ret_path);
                strncpy((char*)shell->Shell_TxData + tx_len, ">", (uint16_t)strlen(">"));
		        tx_len += (uint16_t)strlen(">");
            }
            
		    Uart_SendMessageDMA(Const_Shell_UART_HANDLER, shell->Shell_TxData, tx_len);
        }
        osDelay(10);
	}
}


void Shell_Init() {
    Shell_DataTypeDef* shell = Shell_GetShellDataPtr();

    CMD_RegisterCLICommands();
    Uart_InitUartDMA(Const_Shell_UART_HANDLER);
    Uart_ReceiveDMA(Const_Shell_UART_HANDLER, shell->Shell_RxData, Const_Shell_RX_BUFF_LEN);
}


Shell_DataTypeDef* Shell_GetShellDataPtr() {
    return &ShellData;
}


void Shell_RXCallback(UART_HandleTypeDef* huart) {
    Shell_DataTypeDef* shell = Shell_GetShellDataPtr();

    /* clear DMA transfer complete flag */
    __HAL_DMA_DISABLE(huart->hdmarx);

    /* handle uart data from DMA */
    int rxdatalen = Const_Shell_RX_BUFF_LEN - Uart_DMACurrentDataCounter(huart->hdmarx->Instance);
	xSemaphoreGiveFromISR(Shell_RxCMDHandle, NULL);
    shell->rx_len = rxdatalen;
	
    /* restart dma transmission */
    __HAL_DMA_SET_COUNTER(huart->hdmarx, Const_Shell_RX_BUFF_LEN);
    __HAL_DMA_ENABLE(huart->hdmarx);
}

