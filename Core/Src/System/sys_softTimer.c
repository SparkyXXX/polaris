/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : sys_softTimer.c
 *  Description  : This file contains the soft timers task
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 13:49:14
 *  LastEditTime : 2023-08-07 02:46:54
 */


#include "sys_softTimer.h"
#include "periph_beeper.h"
#include "periph_led.h"
#include "periph_motor.h"
#include "util_adc.h"

float temp_adc, vrefint_adc, vbat_adc;

/** 
  * @brief          Start All Timer and Set cycle 
  * @retval        
 */
void SoftTimer_StartAll() {
    osTimerStart(Beeper_TimerHandle, 250);
}


/** 
  * @brief         For beeper Timer Callback
  * @param         argument
  * @retval        
 */
void BeeperTimerCallback(void const * argument) {
    Beeper_RefreshBeeper(&Beeper_MainBeeper);
    LED_Refresh();
    Adc_RefreshData();
    temp_adc = Adc_GetValue("TEMPSENSOR");
    vrefint_adc = Adc_GetValue("VREFINT");
    vbat_adc = Adc_GetValue("VBat");
}

