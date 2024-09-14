/*
 * @Project: Hatrix Robot
 * @Author: Hatrix
 * @Date: 2023-10-30 23:16:07
 * @LastEditors: Hatrix
 * @LastEditTime: 2023-11-13 01:13:08
 */
/*
 *  Project      : Polaris Robot
 * 
 *  FilePath     : periph_key.h
 *  Description  : This file contains key basic functions
 *  LastEditors  : Polaris
 *  Date         : 2022-04-16 22:53:30
 *  LastEditTime : 2023-08-04 02:30:05
 */


#ifndef KEY_PERIPH_H
#define KEY_PERIPH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "util_gpio.h"

#define LONG_PRESS_EVENT    0x02
#define SHORT_PRESS_EVENT   0x01

#define LONG_PRESS_TIME         1000
#define FLASE_TRIGGER_TIME      80

void Key_KeyEventHandler(GPIO_GPIOTypeDef *gpio);
void Key_TriggerKeyEventHandler(GPIO_GPIOTypeDef *gpio);

#ifdef __cplusplus
}
#endif

#endif
