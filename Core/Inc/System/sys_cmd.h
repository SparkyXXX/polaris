/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : sys_cmd.h
 *  Description  : Command Line Control Manager
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:31:15
 *  LastEditTime : 2023-08-14 20:02:04
 */


#ifndef SYS_CMD_H
#define SYS_CMD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "FreeRTOS_CLI.h"
#include "lib_tree.h"

#define CMD_COMMAND_NUM 9
#define NOW_PATH_LENGTH 256

extern Tree_NodeTypeDef* CurTreeNode;

void CMD_RegisterCLICommands(void);
char *CMD_GetNowPathStr(void);

#endif

#ifdef __cplusplus
}

#endif
