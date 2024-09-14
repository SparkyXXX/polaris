/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : app_wheelLeg.h
 *  Description  : Wheel Leg robot application
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:41:49
 *  LastEditTime : 2023-08-23 17:46:04
 */


#ifndef APP_WHEELLEG_H
#define APP_WHEELLEG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "alg_pid.h"

#define WHEELLEG_WHEEL_R 0.06f

typedef enum {
    Balance_STOP    = 0,
    Balance_STAND   = 1,
    Balance_RUN     = 3,
    Balance_FALL    = 4
} WheelLeg_BalanceModeEnum;

typedef enum {
    Cha_NULL        = 0,
    Cha_Gyro        = 1,
    Cha_Servo       = 2,
} WheelLeg_ChassisModeEnum;

typedef struct {
    uint8_t stable_flag;

    float x_offset;
    float x_ref;

    float len;
    float roll_ang_ref;

    float roll_compensate;
    PID_PIDTypeDef rollAngPID;
    PID_PIDParamTypeDef rollAngPIDParam;
    PID_PIDTypeDef rollSpeedPID;
    PID_PIDParamTypeDef rollSpeedPIDParam;
    WheelLeg_BalanceModeEnum bal_mode;
    WheelLeg_ChassisModeEnum cha_mode;
    
    float yaw_control_tick;
    float update_dt;
    uint32_t last_update_tick;
} WheelLeg_DataTypeDef;

void WheelLeg_Task(void const * argument);
void WheelLeg_Init(void);
WheelLeg_DataTypeDef *WheelLeg_GetWheelLegPtr(void);
void WheelLeg_SetOutput(void);
void WheelLeg_SendOutput(void);
void WheelLeg_SetBalMode(WheelLeg_BalanceModeEnum mode);
void WheelLeg_SetChassisMode(WheelLeg_ChassisModeEnum mode);
void WheelLeg_ClearState(void);
void WheelLeg_SetLQRFdb(void);
void WheelLeg_SetRollAngRef(float ref);
void WheelLeg_SetLenRef(float ref);
void WheelLeg_AddXRef(float add);
void WheelLeg_AddLenRef(float add);
void WheelLeg_RollCompensate(void);
void WheelLeg_ChassisControl(void);
void WheelLeg_FallJudgment(void);


#endif

#ifdef __cplusplus
}

#endif
