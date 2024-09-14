/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : app_quadruped.h
 *  Description  : Quadruped robot application
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:42:33
 *  LastEditTime : 2023-08-04 23:31:05
 */


#ifndef APP_QUADRUPED_H
#define APP_QUADRUPED_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "module_quadruped.h"


void Quadruped_Task(void const * argument);

#endif

#ifdef __cplusplus
}

#endif
