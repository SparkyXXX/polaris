/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : sys_softTimer.h
 *  Description  : This file contains the soft timers task
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 13:49:27
 *  LastEditTime : 2023-08-13 00:19:09
 */


#ifndef SYS_SOFTTIMER_H
#define SYS_SOFTTIMER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "cmsis_os.h"

extern osTimerId Beeper_TimerHandle;

void SoftTimer_StartAll(void);
void BeeperTimerCallback(void const * argument);

extern float temp_adc, vrefint_adc, vbat_adc;

#endif

#ifdef __cplusplus
}
#endif
