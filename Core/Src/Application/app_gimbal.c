/*
 *  Project      : Polaris
 * 
 *  file         : app_gimbal.c
 *  Description  : This file contains Gimbal Yaw control function
 *  LastEditors  : Polaris
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2023-08-24 18:13:34
 */


#include "sys_dwt.h"
#include "periph_motor.h"
#include "module_gimbal.h"
#include "app_gimbal.h"
#include "app_ins.h"


/**
  * @brief          Gimbal task
  * @param          NULL
  * @retval         NULL
  */
void Gimbal_Task(void const * argument) {
    Gimbal_DataTypeDef *gimbal = Gimbal_GetGimbalPtr();
    INS_INSTypeDef *ins = INS_GetINSPtr();

    forever {
        if (Main_Type == Main_Head) {
            Gimbal_SetGyroFdb(-ins->Gyro[Y_INS]);
            Gimbal_SetPositionFdb(-ins->Pitch);
        }
        if (DWT_GetDeltaTWithOutUpdate(&gimbal->last_fdb_tick) > 0.6f) {
            Gimbal_SetControlState(0);
        }
        Gimbal_Control();
        Gimbal_Output();
        DWT_Delayms(0.3);
      osDelay(2);
    }
}
