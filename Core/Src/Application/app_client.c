/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : app_client.c
 *  Description  : Client Application
 *  LastEditors  : Polaris
 *  Date         : 2023-01-24 01:44:29
 *  LastEditTime : 2023-08-24 23:23:34
 */


#include "app_client.h"
#include "sys_dwt.h"
#include "lib_gif.h"
#include "periph_key.h"
#include "periph_beeper.h"
#include "periph_oled.h"
#include "periph_bmp280.h"
#include "module_hop.h"
#include "module_wheel.h"
#include "module_quadruped.h"
#include "module_gimbal.h"
#include "app_ins.h"
#include "app_wheelLeg.h"

Client_PageTypeDef Client_InsState;
Client_PageTypeDef Client_ServoState;
Client_PageTypeDef Client_LeftLegState;
Client_PageTypeDef Client_LeftMotorState;
Client_PageTypeDef Client_RightLegState;
Client_PageTypeDef Client_RightMotorState;
Client_PageTypeDef Client_LQRState;
Client_PageTypeDef Client_QuadLeftFontState;
Client_PageTypeDef Client_QuadLeftBackState;
Client_PageTypeDef Client_QuadRightFontState;
Client_PageTypeDef Client_QuadRightBackState;

Client_DataTypeDef ClientData;

void Client_Task(void const * argument) {
    Client_DataTypeDef* client = Client_GetClientPtr();
    DWT_DataTypeDef* dwt = DWT_GetDWTDataPtr();

    static float page_stay_period;
    static uint32_t bmp280_tick = 0;
    page_stay_period = dwt->SysTime.ms_tick;
    
    forever {
        if (Main_Type != Main_Head) {
            Client_DisplayInterface();
            Client_BeeperControl();
    
            client->func_event = osMessageGet(Key_TriggerQueueHandle, 1);
            if (client->func_event.status == osEventMessage) {
                page_stay_period = dwt->SysTime.ms_tick;
                Client_KeytHandler(client->func_event.value.v);
            }

            if ((dwt->SysTime.ms_tick - page_stay_period) >= CLIENT_PAGE_STAY_PERIOD) 
                Client_ChangeInterface(CLIENT_ON);            
            
        }
        
        if ((bmp280_tick > 30)) {
            osDelay(10);
            bmp280_tick = 0;
            BMP280_DecodeData();
        }
        bmp280_tick++;
        osDelay(CLIENT_TASK_PERIOD);
    }
}

/**
  * @brief          Client initialization
  * @param          NULL
  * @retval         NULL
  */
void Client_Init() {
    Client_DataTypeDef* client = Client_GetClientPtr();

    client->Client_ScreenDisplay = List_New();
    OLED_DisplayOn();
    Client_PageInit();
    OLED_OperateGram(PEN_CLEAR);
    OLED_RefreshGram();
    Client_ChangeInterface(CLIENT_START);

    client->cur_page_node = List_At(client->Client_ScreenDisplay, 0);
}


/**
  * @brief          Initialize all displayed pages
  * @param          NULL
  * @retval         NULL
  */
void Client_PageInit() {
    INS_INSTypeDef *ins = INS_GetINSPtr();
    Hop_DataTypeDef *hop_data = Hop_GetHopDataPtr();
    Wheel_DataTypeDef *wheel = Wheel_GetWheelDataPtr();
    Quad_QuadrupedDataTypeDef *quad = Quad_GetQuadDataPtr();
    WheelLeg_DataTypeDef *wheelleg = WheelLeg_GetWheelLegPtr();
    Gimbal_DataTypeDef *gimbal = Gimbal_GetGimbalPtr();

    Client_CreateNewPage(&Client_InsState, "pitch\0", &ins->Pitch, "yaw\0", &ins->Yaw, "roll\0", &ins->Roll, "Ins_Data\0", Client_BigKeyLongHandler);
    Client_CreateNewPage(&Client_ServoState, "lim_ang\0", &Motor_GimbalMotor.encoder.limited_angle, "lim_off\0", &Motor_GimbalMotor.encoder.init_offset, "gim_con\0", &gimbal->control_state, "Serv_GimState\0", Client_BigKeyLongHandler);
    Client_CreateNewPage(&Client_LeftLegState, "len\0", &hop_data->left.legLen, "angle\0", &hop_data->left.vir_ang, "Fn\0", &hop_data->left.Fn, "Left_Leg\0", Client_BigKeyLongHandler);
    Client_CreateNewPage(&Client_RightLegState, "len\0", &hop_data->right.legLen, "angle\0", &hop_data->right.vir_ang, "Fn\0", &hop_data->right.Fn, "Right_Leg\0", Client_BigKeyLongHandler);
    Client_CreateNewPage(&Client_LeftMotorState, "font\0", &hop_data->left.fount_mot->encoder.current, "back\0", &hop_data->left.back_mot->encoder.current, "wheel\0", &wheel->left_mot->encoder.current, "Left_Motor\0", Client_BigKeyLongHandler);
    Client_CreateNewPage(&Client_RightMotorState, "font\0", &hop_data->right.fount_mot->encoder.current, "back\0", &hop_data->right.back_mot->encoder.current, "wheel\0", &wheel->right_mot->encoder.current, "Right_Motor\0", Client_BigKeyLongHandler);
    Client_CreateNewPage(&Client_LQRState, "x\0", &wheel->state.x, "dx\0", &wheel->state.dx, "ang\0", &wheel->state.phi, "LQR_State\0", Client_BigKeyLongHandler);
    Client_CreateNewPage(&Client_QuadLeftFontState,  "Motor_Cur1\0", &quad->left_font.body_motor->encoder.current, "Motor_Cur2\0", &quad->left_font.crotch_motor->encoder.current, "Motor_Cur3\0", &quad->left_font.knee_motor->encoder.current, "Quad_Left_Font\0", Client_BigKeyLongHandler);
    Client_CreateNewPage(&Client_QuadLeftBackState,  "Motor_Cur1\0", &quad->left_back.body_motor->encoder.current, "Motor_Cur2\0", &quad->left_back.crotch_motor->encoder.current, "Motor_Cur3\0", &quad->left_back.knee_motor->encoder.current, "Quad_Left_Back\0", Client_BigKeyLongHandler);
    Client_CreateNewPage(&Client_QuadRightFontState, "Motor_Cur1\0", &quad->right_font.body_motor->encoder.current, "Motor_Cur2\0", &quad->right_font.crotch_motor->encoder.current, "Motor_Cur3\0", &quad->right_font.knee_motor->encoder.current, "Quad_Right_Font\0", Client_BigKeyLongHandler);
    Client_CreateNewPage(&Client_QuadRightBackState, "Motor_Cur1\0", &quad->right_back.body_motor->encoder.current, "Motor_Cur2\0", &quad->right_back.crotch_motor->encoder.current, "Motor_Cur3\0", &quad->right_back.knee_motor->encoder.current, "Quad_Right_Back\0", Client_BigKeyLongHandler);
}


Client_DataTypeDef* Client_GetClientPtr() {
    return &ClientData;
}


/**
  * @brief          Cartoon interface display
  * @param          NULL
  * @retval         NULL
  */
static void Client_DisplayInterface() {
    Client_DataTypeDef* client = Client_GetClientPtr();

    if (client->interface_flag == CLIENT_NULL) return;
    
    OLED_OperateGram(PEN_CLEAR);
    
    if (client->interface_flag == CLIENT_START) {
        OLED_DisplayGIF(GIF_ROCKET);
    }
    else if (client->interface_flag == CLIENT_ON) {
        OLED_DisplayGIF(GIF_FIRE);
        OLED_Printf(4, 5, "  Polaris\0");
    }
    else if (client->interface_flag == CLIENT_ERROR_1) {
        OLED_DisplayBMG(OLED_BEAR);
    }
    else if (client->interface_flag == CLIENT_ERROR_2) {
        OLED_DisplayBMG(OLED_SUR);
    }
    else if (client->interface_flag == CLIENT_ERROR_3) {
        OLED_DisplayBMG(OLED_SUB);
    }
    else if (client->interface_flag == CLIENT_PAGE) {
        Client_DisplayCurrentPage();
    }
    OLED_RefreshGram();
}


/**
  * @brief          Cartoon interface display
  * @param          NULL
  * @retval         NULL
  */
void Client_ChangeInterface(Client_InterfaceEnum intface) {
    Client_DataTypeDef* client = Client_GetClientPtr();
    if ((client->interface_flag != CLIENT_ERROR_1) || (client->interface_flag != CLIENT_ERROR_2) || (client->interface_flag != CLIENT_ERROR_3))
        client->interface_flag = intface;
}


void Client_ClearErrorFlag() {
    Client_DataTypeDef* client = Client_GetClientPtr();
    client->interface_flag = CLIENT_ON;
    Beeper_SetState(&Beeper_MainBeeper, Beeper_OFF, 0, 0, NULL);
}


/**
  * @brief          Create a new display page
  * @param          NULL
  * @retval         NULL
  */
void Client_CreateNewPage(Client_PageTypeDef *page, const char *val1_name, void *val_1,
                                                    const char *val2_name, void *val_2,
                                                    const char *val3_name, void *val_3,
                                                    const char *page_name,
                                                    Client_KeyEventHandlerTypeDef key_double) {
    Client_DataTypeDef* client = Client_GetClientPtr();

    page->double_press_handler = key_double;  
    memcpy(page->name[0], val1_name, 12);
    memcpy(page->name[1], val2_name, 12);                       
    memcpy(page->name[2], val3_name, 12);
    memcpy(page->page_name, page_name, 15);
    
    page->value[0] = (float *)val_1;
    page->value[1] = (float *)val_2;
    page->value[2] = (float *)val_3;
    List_Rpush(client->Client_ScreenDisplay, List_NodeNew(page));
}


/**
  * @brief          Show a page
  * @param          NULL
  * @retval         NULL
  */
static void Client_DisplayCurrentPage() {
    Client_DataTypeDef* client = Client_GetClientPtr();
    DWT_DataTypeDef* dwt = DWT_GetDWTDataPtr();

    static float page_period = 0;
    if (dwt->SysTime.ms_tick - page_period >= CLIENT_PAGE_REFRESH_PERIOD) {
        page_period = dwt->SysTime.ms_tick;
        Client_PageTypeDef *cur_page;
        if (client->next_page_flag == 1) {
            client->cur_page_node = client->cur_page_node->next == NULL ? client->Client_ScreenDisplay->head : client->cur_page_node->next;
            cur_page = (Client_PageTypeDef *)client->cur_page_node->val;
            client->next_page_flag = 0;
        }
        else if (client->previous_page_flag == 1) {
            client->cur_page_node = client->cur_page_node->prev == NULL ? client->Client_ScreenDisplay->tail : client->cur_page_node->prev;
            cur_page = (Client_PageTypeDef *)client->cur_page_node->val;
            client->previous_page_flag = 0;
        }
        else {
             cur_page = (Client_PageTypeDef *)client->cur_page_node->val;
        }

        if (cur_page == NULL) return;

        for (int i = 0; i < 3 ; i++) {
            OLED_ShowString(i + 1, 1, cur_page->name[i]);
            OLED_Printf(i + 1, 13, ":%0.2f\0", (*cur_page->value[i]));
        }
        OLED_ShowString(4, 1, cur_page->page_name);
    }
}


/**
  * @brief          Back key Short press handling function
  * @param          NULL
  * @retval         NULL
  */
void Client_KeytHandler(uint32_t message) {
    Client_DataTypeDef* client = Client_GetClientPtr();

    uint8_t gpio = (uint8_t)(0xff & message);
    uint8_t press_event = (uint8_t)(0xff & (message >> 8));
    Client_PageTypeDef *cur_page;
    cur_page = (Client_PageTypeDef *)client->cur_page_node->val;
    if (cur_page == NULL) return;

    switch (gpio) {
        case BIG_KEY1_EVENT_ID:
            if (press_event == SHORT_PRESS_EVENT) {
                Client_ChangeInterface(CLIENT_PAGE);
                client->next_page_flag = 1;
            }
            else if (press_event == LONG_PRESS_EVENT) {
                if (cur_page->double_press_handler != NULL)
                    cur_page->double_press_handler();
                    client->previous_page_flag = 1;
            }
            break;
        case BIG_KEY2_EVENT_ID:
            if (press_event == SHORT_PRESS_EVENT) {
                Client_ChangeInterface(CLIENT_PAGE);
                client->previous_page_flag = 1;
            }
            else if (press_event == LONG_PRESS_EVENT) {
                if (cur_page->double_press_handler != NULL)
                    cur_page->double_press_handler();
                    client->next_page_flag = 1;
            }   
            break;     
        default:
            break;
        }
}


void Client_BeeperControl() {
    Client_DataTypeDef* client = Client_GetClientPtr();
    static uint32_t beeper_refresh_flag = 0;
    static uint32_t trick_flag = 0;
    
    if (client->interface_flag == CLIENT_ERROR_1) {
        Beeper_SetState(&Beeper_MainBeeper, Beeper_ON, 600, 0, NULL);
        beeper_refresh_flag = 1;
    }
    else if (client->interface_flag == CLIENT_ERROR_2) {
        Beeper_SetState(&Beeper_MainBeeper, Beeper_ON, 800, 0, NULL);
        beeper_refresh_flag = 1;
    }
    else if (client->interface_flag == CLIENT_ERROR_3) {
        Beeper_SetState(&Beeper_MainBeeper, Beeper_ON_FOUR_TIMES, 600, 0, NULL);
        beeper_refresh_flag = 1;
    }
    else {
        beeper_refresh_flag = 0;
    }

    if (beeper_refresh_flag && !trick_flag) {
        Beeper_RefreshBeeper(&Beeper_MainBeeper);
        trick_flag = 1;
    }
}

/**
  * @brief          Back key Short press handling function
  * @param          NULL
  * @retval         NULL
  */
void Client_BigKeyLongHandler() {
    static uint8_t test_key = 1;
    if (test_key > 0) {
        Beeper_SetState(&Beeper_MainBeeper, Beeper_OFF, 600, 0, NULL);
        test_key = 0;
		return;
    }
    else if (test_key == 0) {
        Beeper_SetState(&Beeper_MainBeeper, Beeper_ON, 600, 0, NULL);
        test_key++;
		return;
    }
}

