/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : periph_beeper.h
 *  Description  : This file contains the beeper driver
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 00:45:11
 *  LastEditTime : 2023-03-29 17:51:15
 */


#ifndef BEEPER_PERIPH_H
#define BEEPER_PERIPH_H

#ifdef __cplusplus
extern "C" {
#endif


#include "util_pwm.h"
#include "lib_music.h"

#define BEEPER_OK               0
#define BEEPER_MUSIC_OVERFLOW   -1
#define BEEPER_ERROR_CMD        -2

typedef enum {
    Beeper_OFF           = 0,      
    Beeper_ON_ONCE       = 1,
    Beeper_ON_TWICE      = 2,
    Beeper_ON_THRICE     = 3,
    Beeper_ON_FOUR_TIMES = 4, 
    Beeper_ON            = 5,
    Beeper_MUSIC         = 6,
    Beeper_EX_MUSIC      = 7,
} Beeper_BeeperStateEnum;

typedef struct {
    Beeper_BeeperStateEnum state;

    uint32_t music_step;
    uint32_t buff[MUSIC_BEEPER_LENGTH];
    PWM_PWMHandleTypeDef pwm;
} Beeper_BeeperTypeDef;


void Beeper_InitAllBeepers(void);
void Beeper_InitBeeper(Beeper_BeeperTypeDef* beeper, TIM_HandleTypeDef* htim, uint32_t ch);
Beeper_BeeperStateEnum Beeper_GetBeeperState(Beeper_BeeperTypeDef* beeper);
int Beeper_SetState(Beeper_BeeperTypeDef* beeper, Beeper_BeeperStateEnum state, 
                        uint32_t frq, uint8_t music_num, uint32_t *buff);
void Beeper_RefreshBeeper(Beeper_BeeperTypeDef* beeper);

extern Beeper_BeeperTypeDef Beeper_MainBeeper;

#endif

#ifdef __cplusplus
}
#endif
