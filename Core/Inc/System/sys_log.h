/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : sys_log.h
 *  Description  : This files contains the log
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:22:36
 *  LastEditTime : 2023-08-12 03:34:21
 */


#ifndef SYS_LOG_H
#define SYS_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "time.h"

#define Const_LOG_BUFF_LEN  128

typedef enum {
    LOG_NULL    = 0,
    LOG_SDCARD  = 1,
    LOG_UART    = 2,
    LOG_ALLTYPE = 3,
} Log_SaveModeEnum;

typedef struct {
    Log_SaveModeEnum saveType;
    uint8_t LOG_RxData[Const_LOG_BUFF_LEN];
    FIL file;
    char *log_name;
} Log_DataTypeDef;

void LOG_Init(void);
void LOG_Printf(Log_SaveModeEnum mode, const char *fmt,...);
Log_DataTypeDef* Log_GetLogDataPtr(void); 
void LOG_RXCallback(UART_HandleTypeDef* huart);
uint64_t Time_To_Unix(void);
void Unix_To_Time(uint32_t unix_tick);
uint32_t self_mktime(struct tm tm_now);
void self_gmtime(struct tm *tm_time, uint32_t timestamp);

#endif

#ifdef __cplusplus
}

#endif
