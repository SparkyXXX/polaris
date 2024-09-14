/*
 *  Project      : Polaris Robot
 * 
 *  FilePath     : periph_led.h
 *  Description  : This file contains the led periph control functions
 *  LastEditors  : Polaris
 *  Date         : 2022-04-16 22:53:30
 *  LastEditTime : 2023-01-24 02:13:13
 */


#ifndef LED_PERIPH_H
#define LED_PERIPH_H

#ifdef __cplusplus
extern "C" {
#endif 


#include "gpio.h"


typedef enum {
    LED_OFF         = 0,
    LED_ON          = 1,
    LED_FLASHING    = 2,
} LED_LEDStateEnum;

typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
    LED_LEDStateEnum state;
    uint8_t cnt;
} LED_LEDTypeDef;

void LED_InitAllLEDs(void);
void LED_InitLED(LED_LEDTypeDef* led, GPIO_TypeDef* port, uint16_t pin, LED_LEDStateEnum init_state);
LED_LEDStateEnum LED_GetLEDState(LED_LEDTypeDef* led);
void LED_Refresh(void);
void LED_SetLEDState(LED_LEDTypeDef* led, LED_LEDStateEnum state);
void LED_AllOff(void);

extern LED_LEDTypeDef LED_GREEN;
extern LED_LEDTypeDef LED_RED;

#endif

#ifdef __cplusplus
}
#endif
