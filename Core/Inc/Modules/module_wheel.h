/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : module_wheel.h
 *  Description  : Balance wheel module function
 *  LastEditors  : Polaris
 *  Date         : 2023-02-08 15:36:41
 *  LastEditTime : 2023-08-21 19:40:19
 */


#ifndef MODULE_WHEEL_H
#define MODULE_WHEEL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "periph_motor.h"
#include "alg_pid.h"

typedef struct {
    float theta;
    float dtheta;
    
    float x;
    float dx;

    float phi;
    float dphi;
} Wheel_WheelStateTypeDef;

typedef struct {
    float K[12];
    float yaw_compensate;
    float T, Tp;

    Wheel_WheelStateTypeDef state;

    PID_PIDTypeDef balYawSpdPID;
    PID_PIDParamTypeDef balYawSpdPIDParam;    
    PID_PIDTypeDef balYawAngPID;
    PID_PIDParamTypeDef balYawAngPIDParam;
    
    Motor_MotorTypeDef *left_mot;
    Motor_MotorTypeDef *right_mot;
} Wheel_DataTypeDef;

void Wheel_Init(void);
Wheel_DataTypeDef *Wheel_GetWheelDataPtr(void);
void Wheel_UpdateState(Wheel_DataTypeDef *wheel, Wheel_WheelStateTypeDef state);
void Wheel_LQRUpdate(Wheel_DataTypeDef *wheel, float len, float ref);
void Wheel_YawCompensate(float yaw, float spd, float ang_ref, float spd_ref, uint8_t ring);
void Wheel_AddYawAngRef(float add);

#ifdef __cplusplus
}
#endif

#endif
