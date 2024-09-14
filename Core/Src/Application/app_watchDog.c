/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : app_watchDog.c
 *  Description  : Watch Dog Application
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:45:15
 *  LastEditTime : 2023-08-24 13:09:31
 */


#include "app_watchDog.h"
#include "sys_dwt.h"
#include "periph_beeper.h"
#include "periph_bmi088.h"
#include "periph_motor.h"
#include "periph_remote.h"
#include "periph_magnetic.h"
#include "periph_bmp280.h"
#include "module_gimbal.h"
#include "app_ins.h"
#include "app_quadruped.h"
#include "app_wheelLeg.h"
#include "app_client.h"
#include "protocol_common.h"

WatchDog_DataTypeDef WatchData;


WatchDog_FeedEntry WatchDog_Bowls[Const_WatchDog_Bowl_Num] = {
    {&BMI088_IsBMI088Offline        },
    {&MAG_IsMAGOffline              },
    {&BMP280_IsBMP280Offline        },
    {&Motor_IsAnyMotorOffline       }, 
    {&Remote_IsRemoteOffline        },
    {&Protocol_IsProtocolOffline    },
    {&Gimbal_IsGimbalFdbOffline     }
};


void WatchDog_Task(void const * argument) {
    WatchDog_DataTypeDef *watchdog = WatchDog_GetWatchDogDataPtr();
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();
    Quad_QuadrupedDataTypeDef *quad = Quad_GetQuadDataPtr();
    Remote_RemoteDataTypeDef *remote = Remote_GetRemoteDataPtr();
    WheelLeg_DataTypeDef *wheelleg = WheelLeg_GetWheelLegPtr();
    Gimbal_DataTypeDef *gimbal = Gimbal_GetGimbalPtr();
    INS_INSTypeDef* ins = INS_GetINSPtr();
    
    forever {
        watchdog->comm_rx_dt = buscomm->rx_dt;
        watchdog->comm_tx_dt = buscomm->tx_dt;
		watchdog->comm_gim_rx_dt = buscomm->gim_rx_dt;
        watchdog->gimbal_fdb_dt = gimbal->fdb_dt;
        watchdog->gimbal_dt = gimbal->update_dt;
        watchdog->ins_dt = ins->update_dt;
        watchdog->remote_dt = remote->update_dt;
        watchdog->wheelleg_dt = wheelleg->update_dt;
        WatchDog_Feed();
      osDelay(50);
    }
}


WatchDog_DataTypeDef* WatchDog_GetWatchDogDataPtr() {
    return &WatchData;
}

void WatchDog_Feed() {
    WatchDog_DataTypeDef *watchdog = WatchDog_GetWatchDogDataPtr();

    uint32_t bowls = 0;
    static uint32_t motor_lost_count = 0;
    for (int i = 0; i < Const_WatchDog_Bowl_Num; i++) {
        if (WatchDog_Bowls[i].feed != NULL) {
            uint8_t bowl = (WatchDog_Bowls[i].feed());
            bowls |= bowl << i;
        }
    }

    if (Main_Type != (Main_FuncTypeEnum)Motor_GetConnectingMode()) 
        motor_lost_count++;
    else 
        motor_lost_count = 0;
    
    
    if (motor_lost_count > 5) {
        Main_Type = (Main_FuncTypeEnum)Motor_GetConnectingMode();
        WatchDog_StopAllOutput();
        Client_ChangeInterface(CLIENT_ERROR_1);
        Beeper_SetState(&Beeper_MainBeeper, Beeper_ON, 700, 0, NULL);
    }
    
    watchdog->Dog_Bowl = bowls;
}


uint32_t WatchDoge_CheckDogBowl() {
    WatchDog_DataTypeDef *watchdog = WatchDog_GetWatchDogDataPtr();
    return watchdog->Dog_Bowl ;
}

void WatchDog_StopAllOutput() {
    vTaskSuspend(WheelLeg_TASKHandle);
    vTaskSuspend(Quadruped_TASKHandle);
    vTaskSuspend(Gimbal_TASKHandle);
    for (int i = 0; i < MOTOR_GROUP_NUM; i++) {
        for (int j = 0; j < Motor_groupHandle[i]->motor_num; j++) {
            Motor_SetMotorOutput(Motor_groupHandle[i]->motor_handle[j], 0);
        }
        Motor_SendMotorGroupOutput(Motor_groupHandle[i]);
        DWT_Delayms(5);
    }
}
