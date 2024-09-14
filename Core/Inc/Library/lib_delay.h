/*
 * @Project: Hatrix Robot
 * @Author: Hatrix
 * @Date: 2023-10-30 23:16:07
 * @LastEditors: Hatrix
 * @LastEditTime: 2023-11-06 23:45:45
 */
/*
 *  Project      : Polaris Robot
 * 
 *  FilePath     : lib_delay.h
 *  Description  : This file contains the functions of delay
 *  LastEditors  : Polaris
 *  Date         : 2022-04-29 12:36:26
 *  LastEditTime : 2023-02-01 03:00:31
 */


#ifndef DELAY_LIB_H
#define DELAY_LIB_H

#ifdef __cplusplus
extern "C" {
#endif 

#include "main.h"

void Delay_Init(void);
void Delay_us(uint16_t nus);
void Delay_ms(uint16_t nms);


#endif

#ifdef __cplusplus
}
#endif
