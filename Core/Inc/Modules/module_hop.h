/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : module_hop.h
 *  Description  : Bounce structure module function
 *  LastEditors  : Polaris
 *  Date         : 2023-02-08 15:36:41
 *  LastEditTime : 2023-08-23 16:05:53
 */


#ifndef MODULE_HOP_H
#define MODULE_HOP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "alg_pid.h"
#include "periph_motor.h"

typedef struct {
    float theta_1;
    float theta_2;

    float legLen;
    float legLenDiff1;
    float lenLenDiff2;
    float len_err[3];

    float jacobian[4];

    float vir_ang;
    float vir_angDiff1;
    float vir_angDiff2;
    float vir_ang_err[3];

    float zw, zm;
    float Fn;
    float theta_1_offset;
    float theta_2_offset;


    float bal_fon_out;
    float bal_bak_out;

    PID_PIDTypeDef angPID;
    PID_PIDParamTypeDef angPIDParam;
    PID_PIDTypeDef lenPID;
    PID_PIDParamTypeDef lenPIDParam;

    Motor_MotorTypeDef *fount_mot;
    Motor_MotorTypeDef *back_mot;

    float decode_dt;
    uint32_t decode_tick;
} Hop_LegDataTypeDef;

typedef struct {    
    Hop_LegDataTypeDef left;
    Hop_LegDataTypeDef right;
} Hop_DataTypeDef;


void Hop_Init(void);
void Hop_Balance(Hop_LegDataTypeDef *leg, float len);
uint8_t Hop_DecodeLegFunction(Hop_LegDataTypeDef *leg);
Hop_DataTypeDef *Hop_GetHopDataPtr(void);
uint8_t Hop_DecodeSupportFn(Hop_LegDataTypeDef *leg, float zm);
void Hop_CalcDiffData(Hop_LegDataTypeDef *leg);
void Hop_CalcOutput(Hop_LegDataTypeDef *leg, float bal_Tp, float roll_Tp);

#endif

#ifdef __cplusplus
}

#endif
