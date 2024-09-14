/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : module_quadruped.h
 *  Description  : Quadruped module function
 *  LastEditors  : Polaris
 *  Date         : 2023-03-13 13:22:39
 *  LastEditTime : 2023-03-30 16:59:50
 */


#ifndef MODULE_QUADRUPED_H
#define MODULE_QUADRUPED_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "periph_motor.h"

#define LimitTpByAng(tp, ang, max, min) {       \
        if ((ang >= max) && (tp > 0)) {         \
            tp /= 10.0f;                        \
        }                                       \
        else if ((ang <= min) && (tp < 0)) {    \
            tp /= 10.0f;                        \
        }                                       \
    }


typedef struct {
    float theta[3];
    float theta_offset[3];

    float jacobian[9];
    float x;
    float y;
    float z;
    
    Motor_MotorTypeDef *body_motor;
    Motor_MotorTypeDef *crotch_motor;
    Motor_MotorTypeDef *knee_motor;
} Quad_LegTypeDef;

typedef struct {
    Quad_LegTypeDef left_font;
    Quad_LegTypeDef left_back;
    Quad_LegTypeDef right_font;
    Quad_LegTypeDef right_back;
} Quad_QuadrupedDataTypeDef;

void Quad_Init(void);
Quad_QuadrupedDataTypeDef *Quad_GetQuadDataPtr(void);
void Quad_DecodeTheta(Quad_LegTypeDef *leg);
void Quadruped_SetLegTp(Quad_LegTypeDef *leg, float tp_1, float tp_2, float tp_3);

#endif

#ifdef __cplusplus
}

#endif
