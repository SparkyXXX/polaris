/*
 *  Project      : Polaris Robot
 * 
 *  FilePath     : periph_led.c
 *  Description  : This file contains the led periph control functions
 *  LastEditors  : Polaris
 *  Date         : 2022-04-16 22:53:07
 *  LastEditTime : 2023-08-12 03:09:00
 */


#include "periph_led.h"


LED_LEDTypeDef LED_GREEN, LED_RED;

/**
  * @brief      Initialize all onboard LEDs
  * @param      NULL
  * @retval     NULL
  */
void LED_InitAllLEDs() {
   LED_InitLED(&LED_GREEN,  GPIOC, GPIO_PIN_13, LED_FLASHING);
   LED_InitLED(&LED_RED,    GPIOA, GPIO_PIN_15, LED_ON);
}


/**
  * @brief      Read LED status
  * @param      led: Pointer to LED object
  * @retval     LED status
  */
LED_LEDStateEnum LED_GetLEDState(LED_LEDTypeDef* led) {
    return led->state;
}


/**
  * @brief      Set LED status
  * @param      led: Pointer to LED object
  * @param      state: LED status
  * @retval     NULL
  */
void LED_SetLEDState(LED_LEDTypeDef* led, LED_LEDStateEnum state) {
    led->state = state;
}


/**
  * @brief      LED status refresh
  * @retval     NULL
  */
void LED_Refresh() {
    if (LED_GREEN.state == LED_OFF) {
        HAL_GPIO_WritePin(LED_GREEN.port, LED_GREEN.pin, GPIO_PIN_RESET);
    }
    else if (LED_GREEN.state == LED_ON) {
        HAL_GPIO_WritePin(LED_GREEN.port, LED_GREEN.pin, GPIO_PIN_SET);
    }
    else if (LED_GREEN.state == LED_FLASHING) {
        LED_GREEN.cnt++;
        if (LED_GREEN.cnt >= 20) {
            LED_GREEN.cnt = 0;
            HAL_GPIO_TogglePin(LED_GREEN.port, LED_GREEN.pin);
        }
    }

    if (LED_RED.state == LED_OFF) {
        HAL_GPIO_WritePin(LED_RED.port, LED_RED.pin, GPIO_PIN_RESET);
    }
    else if (LED_RED.state == LED_ON) {
        HAL_GPIO_WritePin(LED_RED.port, LED_RED.pin, GPIO_PIN_SET);
    }
    else if (LED_RED.state == LED_FLASHING) {
        LED_RED.cnt++;
        if (LED_RED.cnt >= 20) {
            LED_RED.cnt = 0;
            HAL_GPIO_TogglePin(LED_RED.port, LED_RED.pin);
        }
    }   
}

/**
  * @brief      Turn off all LEDs
  * @param      NULL
  * @retval     NULL
  */
void LED_AllOff() {
    LED_SetLEDState(&LED_GREEN, LED_OFF);
    LED_SetLEDState(&LED_RED, LED_OFF);
}


/**
  * @brief      Initialize single LED
  * @param      led: Pointer to LED object
  * @param      port: Led corresponding GPIO port number
  * @param      pin: Led corresponding GPIO pin number
  * @param      init_state: Led initial state
  * @retval     NULL
  */
void LED_InitLED(LED_LEDTypeDef* led, GPIO_TypeDef* port, uint16_t pin, LED_LEDStateEnum init_state) {
    led->port   = port;
    led->pin    = pin;
    led->cnt    = 0;
    LED_SetLEDState(led, init_state);
}
