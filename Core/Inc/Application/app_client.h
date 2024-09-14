/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : app_client.h
 *  Description  : Client Application
 *  LastEditors  : Polaris
 *  Date         : 2023-01-24 01:44:43
 *  LastEditTime : 2023-08-24 18:31:20
 */


#ifndef APP_CLIENT_H
#define APP_CLIENT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"

#include "lib_list.h"
#include "lib_str.h"
#include "alg_math.h"

#define CLIENT_TASK_PERIOD 33
#define CLIENT_PAGE_REFRESH_PERIOD      60.0f
#define CLIENT_PAGE_STAY_PERIOD         6000.0f

typedef struct {
    char name[3][15];
    float *value[3];
    
    char page_name[15];
    void (*double_press_handler)(void);
} Client_PageTypeDef;

typedef enum {
    CLIENT_NULL         = 0,
    CLIENT_START        = 1,
    CLIENT_ON           = 2,
    CLIENT_ERROR_1      = 3,
    CLIENT_ERROR_2      = 4,
    CLIENT_ERROR_3      = 5,
    CLIENT_PAGE         = 6
} Client_InterfaceEnum;

typedef struct {
    List_TypeDef *Client_ScreenDisplay;
    List_NodeTypeDef* cur_page_node;

    uint8_t next_page_flag;
    uint8_t previous_page_flag;
    Client_InterfaceEnum interface_flag;
    osEvent func_event;
} Client_DataTypeDef;

typedef void (*Client_KeyEventHandlerTypeDef)(void);

void Client_Task(void const * argument);
void Client_Init(void);
void Client_PageInit(void);
Client_DataTypeDef* Client_GetClientPtr(void);
void Client_CreateNewPage(Client_PageTypeDef *page, const char *val1_name, void *val_1,
                                                    const char *val2_name, void *val_2,
                                                    const char *val3_name, void *val_3,
                                                    const char *page_name,
                                                    Client_KeyEventHandlerTypeDef key_double);
static void Client_DisplayCurrentPage(void);
void Client_KeytHandler(uint32_t message);
void Client_BigKeyLongHandler(void);
static void Client_DisplayInterface(void);
void Client_BeeperControl(void);
void Client_ClearErrorFlag(void);
void Client_ChangeInterface(Client_InterfaceEnum intface);


#endif

#ifdef __cplusplus
}

#endif
