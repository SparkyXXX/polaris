/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : periph_beeper.c
 *  Description  : This file contains the beeper driver
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 00:44:53
 *  LastEditTime : 2023-08-24 13:05:58
 */


#include "periph_beeper.h"
#include "stdlib.h"
#include "string.h"

Beeper_BeeperTypeDef Beeper_MainBeeper;

/**
  * @brief      Initialize all beeper
  * @param      NULL
  * @retval     NULL
  */
void Beeper_InitAllBeepers() {
    Beeper_InitBeeper(&Beeper_MainBeeper, &htim1, TIM_CHANNEL_1);
}


/**
  * @brief      Initialize the beeper
  * @param      beeper: Pointer to beeper object
  * @param      htim: Timer handle
  * @param      ch: PWM channel number
  * @retval     NULL
  */
void Beeper_InitBeeper(Beeper_BeeperTypeDef* beeper, TIM_HandleTypeDef* htim, uint32_t ch) {
    beeper->state = Beeper_OFF;
    PWM_InitPWM(&(beeper->pwm), htim, ch);
    PWM_SetPWMFreq(&(beeper->pwm), 500);
    PWM_SetPWMDuty(&(beeper->pwm), 1);
    PWM_StartPWM(&(beeper->pwm));
}


/**
  * @brief      Return to beeper status
  * @param      beeper: The pointer points to the actuator object
  * @retval     beeper status
  */
Beeper_BeeperStateEnum Beeper_GetBeeperState(Beeper_BeeperTypeDef* beeper) {
    return beeper->state;
}


/**
  * @brief      Set the beeper state
  * @param      beeper: The pointer points to the actuator object
  * @retval     NULL
  */
int Beeper_SetState(Beeper_BeeperTypeDef* beeper, Beeper_BeeperStateEnum state, 
                         uint32_t frq, uint8_t music_num, uint32_t *buff) {
    if (beeper->state == state) return 0;
    beeper->state = state;
    beeper->music_step = 0;

    if (state == Beeper_ON) {
        beeper->buff[0] = frq;
        return BEEPER_OK;
    }
    else if ((state == Beeper_ON_ONCE) || (state == Beeper_ON_TWICE) 
            || (state == Beeper_ON_THRICE) || (state == Beeper_ON_FOUR_TIMES)) {
        memset(beeper->buff , 0, MUSIC_BEEPER_LENGTH * sizeof(uint32_t));
        for (int i = 0; i < MUSIC_BEEPER_LENGTH; i = i + (int)(state) + 1) {
            beeper->buff[i] = frq;
        }
        return BEEPER_OK;
    }
    else if (state == Beeper_MUSIC) {
        if (music_num >= MUSIC_BEEPER_MAXNUM) {
            return BEEPER_MUSIC_OVERFLOW;
        }
        else {
            memset(beeper->buff, 0, MUSIC_BEEPER_LENGTH * sizeof(uint32_t));
            memcpy(beeper->buff, BEEPER_MUSIC[music_num], MUSIC_BEEPER_LENGTH * sizeof(uint32_t));
            beeper->music_step = 0;
            return BEEPER_OK;
        }
    }
    else if (state == Beeper_OFF) {
        memset(beeper->buff, 0, MUSIC_BEEPER_LENGTH * sizeof(uint32_t));
        return BEEPER_OK; 
    }
    else {
        beeper->state = Beeper_OFF;
        memset(beeper->buff, 0, MUSIC_BEEPER_LENGTH * sizeof(uint32_t));
        return BEEPER_ERROR_CMD;
    }
}


/** 
  * @brief          Refresh the beeper output
  * @param          beeper: The pointer points to the actuator object   
  * @retval         NULL
 */
void Beeper_RefreshBeeper(Beeper_BeeperTypeDef* beeper) {
    if (beeper->music_step >= MUSIC_BEEPER_LENGTH - 1)
        beeper->music_step = 0;

    if (beeper->state == Beeper_ON) {
        if (beeper->buff[0] == 0) {
            PWM_SetPWMDuty(&(beeper->pwm), 1);
        }
        else {
            PWM_SetPWMDuty(&(beeper->pwm), 0.5f);
            PWM_SetPWMFreq(&(beeper->pwm), beeper->buff[0]);
        }
    }
    else if (beeper->state != Beeper_OFF) {
        if (beeper->buff[beeper->music_step] == 0) {
            PWM_SetPWMDuty(&(beeper->pwm), 1);
        }
        else {
            PWM_SetPWMDuty(&(beeper->pwm), 0.5f);
            PWM_SetPWMFreq(&(beeper->pwm), beeper->buff[beeper->music_step]);
        }
    }

    if (beeper->state == Beeper_OFF)
        PWM_SetPWMDuty(&beeper->pwm, 1);


    beeper->music_step++;
}
