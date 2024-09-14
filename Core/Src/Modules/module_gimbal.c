/*
 *  Project      : Polaris
 * 
 *  file         : module_gimbal.c
 *  Description  : This file contains Gimbal Yaw control function
 *  LastEditors  : Polaris
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2023-08-23 14:35:50
 */


#include "module_gimbal.h"
#include "sys_const.h"
#include "sys_dwt.h"
#include "app_wheelLeg.h"
#include "app_ins.h"


Gimbal_DataTypeDef Gimbal_Data;

/**
  * @brief      Gimbal yaw control initialization
  * @param      NULL
  * @retval     NULL
  */
void Gimbal_Init() {
    Gimbal_DataTypeDef *gimbal = Gimbal_GetGimbalPtr();

    gimbal->control_state = 0;
    gimbal->ref = 0;
    gimbal->type = Gimbal_Yaw;
    gimbal->update_dt = 0;
	gimbal->last_update_tick = 0;
    gimbal->fdb_dt = 0;
	gimbal->last_fdb_tick = 0;
    PID_InitPIDParam(&gimbal->angPIDParam, Const_GimbalYawParam[0][0][0], Const_GimbalYawParam[0][0][1], Const_GimbalYawParam[0][0][2], Const_GimbalYawParam[0][0][3], 
                    Const_GimbalYawParam[0][0][4], Const_GimbalYawParam[0][1][0], Const_GimbalYawParam[0][1][1], Const_GimbalYawParam[0][2][0], Const_GimbalYawParam[0][2][1], 
                    Const_GimbalYawParam[0][3][0], Const_GimbalYawParam[0][3][1], PID_POSITION);   
    PID_InitPIDParam(&gimbal->spdPIDParam, Const_GimbalYawParam[1][0][0], Const_GimbalYawParam[1][0][1], Const_GimbalYawParam[1][0][2], Const_GimbalYawParam[1][0][3], 
                    Const_GimbalYawParam[1][0][4], Const_GimbalYawParam[1][1][0], Const_GimbalYawParam[1][1][1], Const_GimbalYawParam[1][2][0], Const_GimbalYawParam[1][2][1], 
                    Const_GimbalYawParam[1][3][0], Const_GimbalYawParam[1][3][1], PID_POSITION);                  
}


/**
  * @brief      Get the pointer of gimbal control object
  * @param      NULL
  * @retval     Pointer to gimbal control object
  */
Gimbal_DataTypeDef* Gimbal_GetGimbalPtr() {
    return &Gimbal_Data;
}


uint8_t Gimbal_IsGimbalFdbOffline() {
    Gimbal_DataTypeDef *gimbal = Gimbal_GetGimbalPtr();
    if (((DWT_GetDeltaTWithOutUpdate(&gimbal->last_fdb_tick)) > Const_GimbalFdb_OFFLINE_TIME)) return 1;
    else return 0;
}


void Gimbal_SetControlType(Gimbal_ControlTypeEnum type) {
    Gimbal_DataTypeDef *gimbal = Gimbal_GetGimbalPtr();

    if (type == Gimbal_Null) {
        gimbal->type = type;
        gimbal->control_state = 0;
        PID_ClearPID(&gimbal->angPID);
        PID_ClearPID(&gimbal->spdPID);
        return;
    }

    if (type != gimbal->type) {
        PID_ClearPID(&gimbal->angPID);
        PID_ClearPID(&gimbal->spdPID);
        if (type == Gimbal_Yaw) {
            PID_InitPIDParam(&gimbal->angPIDParam, Const_GimbalYawParam[0][0][0], Const_GimbalYawParam[0][0][1], Const_GimbalYawParam[0][0][2], Const_GimbalYawParam[0][0][3], 
                            Const_GimbalYawParam[0][0][4], Const_GimbalYawParam[0][1][0], Const_GimbalYawParam[0][1][1], Const_GimbalYawParam[0][2][0], Const_GimbalYawParam[0][2][1], 
                            Const_GimbalYawParam[0][3][0], Const_GimbalYawParam[0][3][1], PID_POSITION);   
            PID_InitPIDParam(&gimbal->spdPIDParam, Const_GimbalYawParam[1][0][0], Const_GimbalYawParam[1][0][1], Const_GimbalYawParam[1][0][2], Const_GimbalYawParam[1][0][3], 
                            Const_GimbalYawParam[1][0][4], Const_GimbalYawParam[1][1][0], Const_GimbalYawParam[1][1][1], Const_GimbalYawParam[1][2][0], Const_GimbalYawParam[1][2][1], 
                            Const_GimbalYawParam[1][3][0], Const_GimbalYawParam[1][3][1], PID_POSITION); 
        }
        else if (type == Gimbal_Pitch) {
            PID_InitPIDParam(&gimbal->angPIDParam, Const_GimbalPitchParam[0][0][0], Const_GimbalPitchParam[0][0][1], Const_GimbalPitchParam[0][0][2], Const_GimbalPitchParam[0][0][3], 
                            Const_GimbalPitchParam[0][0][4], Const_GimbalPitchParam[0][1][0], Const_GimbalPitchParam[0][1][1], Const_GimbalPitchParam[0][2][0], Const_GimbalPitchParam[0][2][1], 
                            Const_GimbalPitchParam[0][3][0], Const_GimbalPitchParam[0][3][1], PID_POSITION);   
            PID_InitPIDParam(&gimbal->spdPIDParam, Const_GimbalPitchParam[1][0][0], Const_GimbalPitchParam[1][0][1], Const_GimbalPitchParam[1][0][2], Const_GimbalPitchParam[1][0][3], 
                            Const_GimbalPitchParam[1][0][4], Const_GimbalPitchParam[1][1][0], Const_GimbalPitchParam[1][1][1], Const_GimbalPitchParam[1][2][0], Const_GimbalPitchParam[1][2][1], 
                            Const_GimbalPitchParam[1][3][0], Const_GimbalPitchParam[1][3][1], PID_POSITION);             
        }
        gimbal->type = type;
    }
    return;
}


/**
  * @brief      Set the gimbal control output calculation enabled state
  * @param      state: Enabled, 1 is enabled, 0 is disabled
  * @retval     NULL
  */
void Gimbal_SetControlState(uint8_t state) {
    Gimbal_DataTypeDef *gimbal = Gimbal_GetGimbalPtr();

    gimbal->control_state = gimbal->type == Gimbal_Null ? 0 :state;
}


/**
  * @brief      Set the target value of gimbal yaw
  * @param      yaw_ref: gimbal yaw target value
  * @retval     NULL
  */
void Gimbal_SetRef(float ref) {
    Gimbal_DataTypeDef *gimbal = Gimbal_GetGimbalPtr();
    
    gimbal->ref += ref;
}


/**
  * @brief      Setting IMU yaw position feedback
  * @param      position_fdb: IMU Yaw Position feedback
  * @retval     NULL
  */
void Gimbal_SetPositionFdb(float position_fdb) {
    Gimbal_DataTypeDef *gimbal = Gimbal_GetGimbalPtr();

    gimbal->position_fdb = position_fdb;
    gimbal->fdb_dt = DWT_GetDeltaT(&gimbal->last_fdb_tick);
}


/**
  * @brief      Setting IMU yaw speed feedback
  * @param      gyro_fdb: IMU Yaw Speed feedback
  * @retval     NULL
  */
void Gimbal_SetGyroFdb(float gyro_fdb) {
    Gimbal_DataTypeDef *gimbal = Gimbal_GetGimbalPtr();

    gimbal->gyro_fdb = gyro_fdb;
}


/**
* @brief      Pitch angle limit
* @param      ref: Pitch set ref
* @retval     Limited pitch ref
*/
float Gimbal_LimitPitch(float ref) {
    Gimbal_DataTypeDef *gimbal = Gimbal_GetGimbalPtr();
    
    if (((PID_GetPIDRef(&gimbal->angPID) > (Const_PITCH_UMAXANGLE + gimbal->pitch_cha_offset_ang)) && (ref > 0)) ||
        ((PID_GetPIDRef(&gimbal->angPID) < (Const_PITCH_DMAXANGLE + gimbal->pitch_cha_offset_ang)) && (ref < 0)))
        return 0.0f;
        // Out of depression set maximum ref
    else return ref;
}


/**
* @brief      Yaw angle limit
* @param      ref: Yaw set ref
* @retval     Limited ywa ref
*/
float Gimbal_LimitYaw(float ref) {
    WheelLeg_DataTypeDef *wheel_leg = WheelLeg_GetWheelLegPtr();
    Gimbal_DataTypeDef *gimbal = Gimbal_GetGimbalPtr();
    INS_INSTypeDef *ins = INS_GetINSPtr();

	if (wheel_leg->cha_mode  == Cha_Gyro)
        return ref;
	else if (((Motor_GimbalMotor.encoder.limited_angle - Motor_GimbalMotor.encoder.init_offset > Const_YAW_MAXANGLE) && (ref > 0)) || 
             ((Motor_GimbalMotor.encoder.limited_angle - Motor_GimbalMotor.encoder.init_offset < -Const_YAW_MAXANGLE) && (ref < 0))) 
        return 0.0f;
    else return ref;
}


/**
  * @brief      Control function of gimbal yaw
  * @param      NULL
  * @retval     NULL
  */
void Gimbal_Control() {
    Gimbal_DataTypeDef *gimbal = Gimbal_GetGimbalPtr();

    if (gimbal->control_state != 1)  return;

    gimbal->update_dt = DWT_GetDeltaT(&gimbal->last_update_tick);

    PID_SetPIDRef(&gimbal->angPID, gimbal->ref);
    PID_SetPIDFdb(&gimbal->angPID, gimbal->position_fdb);
    PID_CalcPID(&gimbal->angPID, &gimbal->angPIDParam);

    PID_SetPIDRef(&gimbal->spdPID, PID_GetPIDOutput(&gimbal->angPID));
    PID_SetPIDFdb(&gimbal->spdPID, gimbal->gyro_fdb);
    PID_CalcPID(&gimbal->spdPID, &gimbal->spdPIDParam);   

    Motor_SetMotorOutput(&Motor_GimbalMotor, PID_GetPIDOutput(&gimbal->spdPID));
}


/**
  * @brief      Gimbal yaw output function
  * @param      NULL
  * @retval     NULL
  */
void Gimbal_Output() {
    Gimbal_DataTypeDef *gimbal = Gimbal_GetGimbalPtr();

    if (gimbal->control_state != 1) {
		Motor_SetMotorOutput(&Motor_GimbalMotor, 0);
	}
    Motor_SendMotorGroupOutput(&Motor_GimbalMotors);
}
