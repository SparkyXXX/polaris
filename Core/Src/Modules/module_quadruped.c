/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : module_quadruped.c
 *  Description  : Quadruped module function
 *  LastEditors  : Polaris
 *  Date         : 2023-03-13 13:22:46
 *  LastEditTime : 2023-08-15 01:30:59
 */


#include "module_quadruped.h"
#include "alg_math.h"
#include "sys_const.h"

Quad_QuadrupedDataTypeDef Quad_Data;

void Quad_Init() {
    Quad_QuadrupedDataTypeDef *quad = Quad_GetQuadDataPtr();
    quad->left_font.theta_offset[0] = Const_QUAD_LEFT_FOUNT_1_OFFSET;
    quad->left_font.theta_offset[1] = Const_QUAD_LEFT_FOUNT_2_OFFSET;
    quad->left_font.theta_offset[2] = Const_QUAD_LEFT_FOUNT_3_OFFSET;
    quad->left_back.theta_offset[0] = Const_QUAD_LEFT_BACK_1_OFFSET;
    quad->left_back.theta_offset[1] = Const_QUAD_LEFT_BACK_2_OFFSET;
    quad->left_back.theta_offset[2] = Const_QUAD_LEFT_BACK_3_OFFSET;
    quad->right_font.theta_offset[0] = Const_QUAD_RIGHT_FOUNT_1_OFFSET;
    quad->right_font.theta_offset[1] = Const_QUAD_RIGHT_FOUNT_2_OFFSET;
    quad->right_font.theta_offset[2] = Const_QUAD_RIGHT_FOUNT_3_OFFSET;
    quad->right_back.theta_offset[0] = Const_QUAD_RIGHT_BACK_1_OFFSET;
    quad->right_back.theta_offset[1] = Const_QUAD_RIGHT_BACK_2_OFFSET;
    quad->right_back.theta_offset[2] = Const_QUAD_RIGHT_BACK_3_OFFSET;

    quad->left_font.body_motor = &Motor_BodyLeftFrontMotor;
    quad->left_font.crotch_motor = &Motor_CrotchLeftFrontMotor;
    quad->left_font.knee_motor = &Motor_KneeLeftFrontMotor;
    quad->left_back.body_motor = &Motor_BodyLeftBackMotor;
    quad->left_back.crotch_motor = &Motor_CrotchLeftBackMotor;
    quad->left_back.knee_motor = &Motor_KneeLeftBackMotor;
    quad->right_font.body_motor = &Motor_BodyRightFrontMotor;
    quad->right_font.crotch_motor = &Motor_CrotchRightFrontMotor;
    quad->right_font.knee_motor = &Motor_KneeRightFrontMotor;
    quad->right_back.body_motor = &Motor_BodyRightBackMotor;
    quad->right_back.crotch_motor = &Motor_CrotchRightBackMotor;
    quad->right_back.knee_motor = &Motor_KneeRightBackMotor;
}


Quad_QuadrupedDataTypeDef *Quad_GetQuadDataPtr() {
    return &Quad_Data;
}


void Quad_DecodeTheta(Quad_LegTypeDef *leg) {
    leg->theta[0] = leg->body_motor->encoder.consequent_angle + leg->theta_offset[0];
    if (leg == &Quad_Data.left_back) {
        leg->theta[1] = -leg->crotch_motor->encoder.consequent_angle + leg->theta_offset[1];
        leg->theta[2] = -leg->knee_motor->encoder.consequent_angle + leg->theta_offset[2];
    }
    else {
        leg->theta[1] = leg->crotch_motor->encoder.consequent_angle + leg->theta_offset[1];
        leg->theta[2] = leg->knee_motor->encoder.consequent_angle + leg->theta_offset[2];
    }
}


void Quadruped_SetLegTp(Quad_LegTypeDef *leg, float tp_1, float tp_2, float tp_3) {
    LimitTpByAng(tp_1, leg->theta[0], 100, 100);
    LimitTpByAng(tp_2, leg->theta[1], 100, 100);
    LimitTpByAng(tp_3, leg->theta[2], 100, 100);

    Motor_SetMotorOutput(leg->body_motor, tp_1);
    Motor_SetMotorOutput(leg->crotch_motor, tp_2);
    Motor_SetMotorOutput(leg->knee_motor, tp_3);
}

