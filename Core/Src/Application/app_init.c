/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : app_init.c
 *  Description  : All initialization threads
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:42:52
 *  LastEditTime : 2023-08-24 13:06:08
 */


#include "app_init.h"
#include "sys_softTimer.h"
#include "sys_log.h"
#include "sys_dwt.h"
#include "sys_const.h"
#include "sys_variables.h"
#include "lib_tree.h"
#include "util_adc.h"
#include "util_can.h"
#include "util_usb.h"
#include "util_timer.h"
#include "periph_led.h"
#include "periph_motor.h"
#include "periph_bmi088.h"
#include "periph_bmp280.h"
#include "periph_oled.h"
#include "periph_remote.h"
#include "periph_beeper.h"
#include "module_hop.h"
#include "module_wheel.h"
#include "module_quadruped.h"
#include "module_gimbal.h"
#include "alg_pid.h"
#include "app_ins.h"
#include "app_wheelLeg.h"
#include "app_client.h"
#include "app_remote.h"
#include "app_shell.h"
#include "protocol_common.h"
#include "SEGGER_SYSVIEW.h"



void Init_InitAll() {
    // system init
    DWT_Init(168);
    SEGGER_SYSVIEW_Conf();

    LOG_Init();
    Tree_Init();
    Vari_Registering();

    // util init
    Adc_Init();
    
 	Can_InitFilterAndStart(&hcan1);
	Can_InitFilterAndStart(&hcan2);

    // periph init
    Beeper_InitAllBeepers();
    Beeper_SetState(&Beeper_MainBeeper, Beeper_ON, 500, 0, NULL);
    Beeper_RefreshBeeper(&Beeper_MainBeeper);
    
    LED_InitAllLEDs();
	BMP280_Init();
    BMI088_Init();

    OLED_init();
    Remote_InitRemote();
    Motor_InitAllMotors();

    // Modules init
    Hop_Init();
    Wheel_Init();
    Gimbal_Init();
    Quad_Init();

    // App init
    INS_Init();

    WheelLeg_Init();

    Remote_RemotrControlInit();
    Protocol_InitProtocol();

    Shell_Init();
    Client_Init();
}


void Init_Task(void const * argument) {
	vTaskSuspend(WatchDog_TASKHandle);
    vTaskSuspend(WheelLeg_TASKHandle);
    vTaskSuspend(Quadruped_TASKHandle);
	vTaskSuspend(Comm_TASKHandle);
    vTaskSuspend(Remote_TASKHandle);
    vTaskSuspend(Gimbal_TASKHandle);

	MX_USB_DEVICE_Init();
    SoftTimer_StartAll();
	
	osDelay(3000);
	Init_MotorDetect();
    Beeper_SetState(&Beeper_MainBeeper, Beeper_OFF, 0, 0, NULL);
    Beeper_RefreshBeeper(&Beeper_MainBeeper);

    if (Main_Type == Main_WHEEL) {
        vTaskResume(WheelLeg_TASKHandle);
        vTaskResume(Remote_TASKHandle);
        Client_ChangeInterface(CLIENT_ON);
        Gimbal_SetControlType(Gimbal_Yaw);
    }
    else if (Main_Type == Main_QUADRUPED) {
        vTaskResume(Quadruped_TASKHandle);
        vTaskResume(Remote_TASKHandle);
        Client_ChangeInterface(CLIENT_ON);
        Gimbal_SetControlType(Gimbal_Yaw);
    }
    else if (Main_Type == Main_Head) {
        vTaskSuspend(Shell_TASKHandle);
        Gimbal_SetControlType(Gimbal_Pitch);
    }
    else {
        Client_ChangeInterface(CLIENT_ERROR_1);
        Gimbal_SetControlType(Gimbal_Null);
	}
	
	vTaskResume(WatchDog_TASKHandle);
	vTaskResume(Comm_TASKHandle);
    vTaskResume(Gimbal_TASKHandle);
	
    forever {
        vTaskSuspend(Init_TASKHandle);
      osDelay(3000);
    }
}


void Init_MotorDetect() { 
    for (int i = 0; i < 50; i++) {
	    Motor_SendMotorGroupsOutput();
        DWT_Delayms(20);
    }
	Motor_IsAnyMotorOffline();
    Main_Type = (Main_FuncTypeEnum)Motor_GetConnectingMode();
}
