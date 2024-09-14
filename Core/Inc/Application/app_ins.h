/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : app_ins.h
 *  Description  : Attitude solution
 *  LastEditors  : Polaris
 *  Date         : 2023-01-24 01:43:43
 *  LastEditTime : 2023-08-08 01:28:34
 */


#ifndef INS_TASK_H
#define INS_TASK_H

#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "util_gpio.h"

#define X_INS 0
#define Y_INS 1
#define Z_INS 2

#define INS_TASK_PERIOD 1

#define INS_INIT_SHIFT          0
#define INS_ACCEL_DR_SHIFT      1
#define INS_GYRO_DR_SHIFT       2
#define INS_MAG_DR_SHIFT        3
#define INS_TEMP_DR_SHIFT       4

typedef struct {
    uint8_t flag;

    float scale[3];

    float Yaw;
    float Pitch;
    float Roll;
} INS_DataTypeDef;

typedef struct {
    float q[4]; 
    float Gyro[3];  
    float Accel[3]; 
    float MotionAccel_b[3]; 
    float MotionAccel_n[3]; 

    float AccelLPF; 

    float xn[3];
    float yn[3];
    float zn[3];

    float atanxz;
    float atanyz;

    float Roll;
    float Pitch;
    float Yaw;
    float YawTotalAngle;

    INS_DataTypeDef Param;

    float update_dt;
    uint32_t last_update_tick;
} INS_INSTypeDef;

extern INS_INSTypeDef INS;

void INS_Init(void);
INS_INSTypeDef *INS_GetINSPtr(void);
void Ins_Task(void const * argument);
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);
static void Param_Correction(INS_DataTypeDef *param, float gyro[3], float accel[3]);
void Ins_ClearYawTotalCounter(void);
void Ins_InstallCompAngle(INS_INSTypeDef *ins, const float correc_mat[3][3]);
void Ins_GPIOExitCallback(GPIO_GPIOTypeDef *gpio);

#endif
