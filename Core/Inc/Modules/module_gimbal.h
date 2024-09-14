/*
 *  Project      : Polaris
 * 
 *  file         : module_gimbal.h
 *  Description  : This file contains Gimbal control function
 *  LastEditors  : Polaris
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2023-08-22 00:15:18
 */

#ifndef MODULE_GIMBAL_H
#define MODULE_GIMBAL_H

#ifdef __cplusplus
extern "C" {
#endif 

#include "periph_motor.h"
#include "alg_math.h"
#include "alg_pid.h"

typedef enum {
    Gimbal_Null     = 0,
    Gimbal_Pitch    = 1,
    Gimbal_Yaw      = 2
} Gimbal_ControlTypeEnum;

typedef struct {
    Gimbal_ControlTypeEnum type;
    float ref;                                      // Gimbal angle target value 
    float position_fdb;                             // Gimbal IMU angle feedback value 
    float gyro_fdb;                                 // Gimbal IMU angular velocity feedback value 

    float pitch_dref;

    uint8_t control_state;                          // Whether to enable output 1 Yes 0 No 
    uint8_t pending_state;                          // Gimbal Occupy Lock 1 Yes 0 No 

    PID_PIDTypeDef spdPID;
    PID_PIDParamTypeDef spdPIDParam;
    PID_PIDTypeDef angPID;
    PID_PIDParamTypeDef angPIDParam;

    uint8_t yaw_total_ang_clear_flag;
    float pitch_cha_offset_ang;

    float fdb_dt;
    uint32_t last_fdb_tick;
    float update_dt;
    uint32_t last_update_tick;
} Gimbal_DataTypeDef;


extern Gimbal_DataTypeDef Gimbal_Data;

void Gimbal_Init(void);
Gimbal_DataTypeDef* Gimbal_GetGimbalPtr(void);
uint8_t Gimbal_IsGimbalFdbOffline(void);
void Gimbal_SetControlType(Gimbal_ControlTypeEnum type);
void Gimbal_SetControlState(uint8_t state);
void Gimbal_SetRef(float ref);
void Gimbal_SetPositionFdb(float position_fdb);
void Gimbal_SetGyroFdb(float gyro_fdb);
float Gimbal_LimitYaw(float ref);
float Gimbal_LimitPitch(float ref);
void Gimbal_Control(void);
void Gimbal_Output(void);

#endif

#ifdef __cplusplus
}
#endif
