/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : app_watchDog.h
 *  Description  : Watch Dog Application
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:45:32
 *  LastEditTime : 2023-08-21 15:59:51
 */


#ifndef APP_WATCHDOG_H
#define APP_WATCHDOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"

#define Const_WatchDog_Bowl_Num 7

typedef struct {
    float ins_dt;
    float comm_rx_dt;
    float comm_tx_dt;
	float comm_gim_rx_dt;
    float quad_dt;
    float remote_dt;
    float wheelleg_dt;
    float gimbal_dt;
    float gimbal_fdb_dt;

    uint32_t Dog_Bowl;
} WatchDog_DataTypeDef;

typedef struct {
    uint8_t (*feed)(void);
} WatchDog_FeedEntry;

extern WatchDog_FeedEntry WatchDog_Bowls[Const_WatchDog_Bowl_Num];

void WatchDog_Task(void const * argument);
WatchDog_DataTypeDef* WatchDog_GetWatchDogDataPtr(void);
void WatchDog_Feed(void);
uint32_t WatchDoge_CheckDogBowl(void);
void WatchDog_StopAllOutput(void);

#endif

#ifdef __cplusplus
}

#endif
