/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : module_hop.c
 *  Description  : Bounce structure module function
 *  LastEditors  : Polaris
 *  Date         : 2023-02-08 15:36:43
 *  LastEditTime : 2023-08-24 00:39:20
 */


#include "module_hop.h"
#include "sys_const.h"
#include "sys_dwt.h"
#include "alg_math.h"

Hop_DataTypeDef Hop_Data;

void Hop_Init() {
    Hop_DataTypeDef *hop_data = Hop_GetHopDataPtr();
    hop_data->left.fount_mot = &Motor_CrotchLeftFrontMotor;
    hop_data->left.back_mot = &Motor_CrotchLeftBackMotor;
    hop_data->left.theta_1_offset = Const_HOP_LEFT_FOUNT_OFFSET;
    hop_data->left.theta_2_offset = Const_HOP_LEFT_BACK_OFFSET;
    hop_data->left.decode_dt = DWT_GetDeltaT(&hop_data->left.decode_tick);

    hop_data->right.fount_mot = &Motor_CrotchRightBackMotor;
    hop_data->right.back_mot = &Motor_CrotchRightFrontMotor;
    hop_data->right.theta_1_offset = Const_HOP_RIGHT_BACK_OFFSET;
    hop_data->right.theta_2_offset = Const_HOP_RIGHT_FOUNT_OFFSET;    
    hop_data->right.decode_dt = DWT_GetDeltaT(&hop_data->right.decode_tick);

    PID_InitPIDParam(&hop_data->left.lenPIDParam, Const_HopLeftLenParam[0][0], Const_HopLeftLenParam[0][1], Const_HopLeftLenParam[0][2], Const_HopLeftLenParam[0][3], 
                    Const_HopLeftLenParam[0][4], Const_HopLeftLenParam[1][0], Const_HopLeftLenParam[1][1], Const_HopLeftLenParam[2][0], Const_HopLeftLenParam[2][1], 
                    Const_HopLeftLenParam[3][0], Const_HopLeftLenParam[3][1], PID_POSITION);
    PID_InitPIDParam(&hop_data->right.lenPIDParam, Const_HopRightLenParam[0][0], Const_HopRightLenParam[0][1], Const_HopRightLenParam[0][2], Const_HopRightLenParam[0][3], 
                    Const_HopRightLenParam[0][4], Const_HopRightLenParam[1][0], Const_HopRightLenParam[1][1], Const_HopRightLenParam[2][0], Const_HopRightLenParam[2][1], 
                    Const_HopRightLenParam[3][0], Const_HopRightLenParam[3][1], PID_POSITION);
    PID_InitPIDParam(&hop_data->left.angPIDParam, Const_HopLeftAngParam[0][0], Const_HopLeftAngParam[0][1], Const_HopLeftAngParam[0][2], Const_HopLeftAngParam[0][3], 
                    Const_HopLeftAngParam[0][4], Const_HopLeftAngParam[1][0], Const_HopLeftAngParam[1][1], Const_HopLeftAngParam[2][0], Const_HopLeftAngParam[2][1], 
                    Const_HopLeftAngParam[3][0], Const_HopLeftAngParam[3][1], PID_POSITION);
    PID_InitPIDParam(&hop_data->right.angPIDParam, Const_HopRightAngParam[0][0], Const_HopRightAngParam[0][1], Const_HopRightAngParam[0][2], Const_HopRightAngParam[0][3], 
                    Const_HopRightAngParam[0][4], Const_HopRightAngParam[1][0], Const_HopRightAngParam[1][1], Const_HopRightAngParam[2][0], Const_HopRightAngParam[2][1], 
                    Const_HopRightAngParam[3][0], Const_HopRightAngParam[3][1], PID_POSITION);
}


Hop_DataTypeDef *Hop_GetHopDataPtr() {
    return &Hop_Data;
}


void Hop_Balance(Hop_LegDataTypeDef *leg, float len) {
    leg->theta_2 = leg->fount_mot->encoder.consequent_angle + leg->theta_2_offset;
    leg->theta_1 = leg->back_mot->encoder.consequent_angle + leg->theta_1_offset;

	Hop_DecodeLegFunction(leg);
    Hop_CalcDiffData(leg);

    PID_SetPIDRef(&leg->lenPID, len);
    PID_SetPIDFdb(&leg->lenPID, leg->legLen);
    PID_CalcPID(&leg->lenPID, &leg->lenPIDParam);

    PID_SetPIDRef(&leg->angPID, 90.0f);
    PID_SetPIDFdb(&leg->angPID, leg->vir_ang);
    PID_CalcPID(&leg->angPID, &leg->angPIDParam);
}


inline uint8_t Hop_DecodeLegFunction(Hop_LegDataTypeDef *leg) {	
    float u1dot = leg->back_mot->encoder.speed;
    float u4dot = leg->fount_mot->encoder.speed;
    float sub_5 = 39.0f * arm_sin_f32(Math_Angle2Rad(leg->theta_2));
    float sub_7 = sub_5 / 500.0f;
    float sub_10 = 39.0f * arm_sin_f32(Math_Angle2Rad(leg->theta_1));
    float sub_11 = sub_10 / 500.0f;
    float sub_14 = 39.0f * arm_cos_f32(Math_Angle2Rad(leg->theta_2));
    float sub_15 = sub_14 / 500.0f;
    float sub_17 = 39.0f * arm_cos_f32(Math_Angle2Rad(leg->theta_1));
    float sub_18 = sub_17 / 500.0f;
    float sub_19 = sub_15 - sub_18;
    float sub_23 = sub_19 + 0.1572f;
    float sub_26 = 13.0f * sub_14 / 39.0f;
    float sub_29 = 13.0f * sub_17 / 39.0f;
    float sub_30 = sub_29 / 100.0f;
    float sub_34 = sub_26 / 100.0f - sub_30 + 0.262f;
    float sub_35 = sub_34 * sub_34;
    float sub_36 = 13.0f * sub_10 / 39.0f;
    float sub_37 = sub_36 / 100.0f;
    float sub_38 = 13.0f * sub_5 / 39.0f;
    float sub_40 = sub_37 - sub_38 / 100.0f;
    float sub_41 = sub_40 * sub_40;
    float sub_42 = sub_35 + sub_41;
    float sub_45 = sub_11 - sub_7;
    float sub_50 = 1.0f / Math_InvSqrt(sub_23 * sub_23 - sub_42 * sub_42 + sub_45 * sub_45);
    float sub_51 = sub_7 - sub_11 + sub_50;
    float sub_54 = sub_19 + sub_35 + sub_41 + 0.1572f;
    float sub_57 = 2.0f * atan(sub_51 / sub_54);
    float sub_58 = arm_cos_f32(sub_57);
    float sub_59 = 3.0f * sub_58;
    float sub_62 = sub_59 / 10.0f + sub_30;
    float sub_65 = sub_62 - 0.131f;
    float sub_66 = sub_65 * sub_65;
    float sub_67 = arm_sin_f32(sub_57);
    float sub_68 = 3.0f * sub_67;
    float sub_70 = sub_68 / 10.0f + sub_37;
    float sub_72 = sub_66 + sub_70 * sub_70;
    float sub_80 = sub_66 - sub_62 * sub_62 + 0.0172f;
    float sub_82 = 1.0f / Math_InvSqrt(sub_72);
    float sub_83 = 131.0f * sub_82;
    float sub_87 = 2.0f * sub_62;
    float sub_90 = sub_36 * u1dot / 100.0f;
    float sub_93 = sub_29 * u1dot / 100.0f;
    float sub_105 = 2.0f * (sub_93 - sub_26 * u4dot / 100.0f) * sub_40 + 2.0f * (sub_90 - sub_38 * u4dot / 100.0f) * sub_34;
    float sub_109 = sub_17 * u1dot / 500.0f;
    float sub_111 = sub_14 * u4dot / 500.0f;
    float sub_117 = sub_10 * u1dot / 500.0f;
    float sub_119 = sub_5 * u4dot / 500.0f;
    float sub_124 = 2.0f * sub_50;
    float sub_132 = sub_54 * sub_54;
    float sub_134 = ((-(2.0f) * sub_105 * sub_42 + 2.0f * (sub_109 - sub_111) * sub_45 + 2.0f * (sub_117 - sub_119) * sub_23) / sub_124 - sub_109 + sub_111) / sub_54 - sub_51 * (sub_105 + sub_117 - sub_119) / sub_132;
    float sub_140 = 5.0f * (sub_51 * sub_51 / sub_132 + 1.0f);
    float sub_142 = sub_90 + sub_68 * sub_134 / sub_140;
    float sub_145 = 2.0f * sub_142 * sub_65;
    float sub_160 = 131.0f / Math_InvSqrt(sub_72 * sub_72 * sub_72);
    float sub_171 = 1.0f / Math_InvSqrt(1.0f - 250000.0f * sub_80 * sub_80 / 17161.0f * sub_72);
    float sub_179 = sub_36 * sub_34 / 50.0f;
    float sub_181 = sub_29 * sub_40 / 50.0f;
    float sub_196 = (sub_18 - (sub_10 * sub_23 / 250.0f - 2.0f * (sub_179 + sub_181) * sub_42 + sub_17 * sub_45 / 250.0f) / sub_124) / sub_54 + (sub_11 + sub_179 + sub_181) * sub_51 / sub_132;
    float sub_204 = sub_37 - sub_68 * sub_196 / sub_140;
    float sub_206 = 2.0f * sub_204 * sub_65;
    float sub_207 = 2.0f * (sub_30 - sub_59 * sub_196 / sub_140) * sub_70 - sub_206;
    float sub_208 = 2.0f * sub_82;
    float sub_225 = 6.0f * sub_67;
    float sub_229 = sub_38 * sub_34 / 50.0f;
    float sub_231 = sub_26 * sub_40 / 50.0f;
    float sub_246 = (sub_15 - (sub_5 * sub_23 / 250.0f - 2.0f * (sub_229 + sub_231) * sub_42 + sub_14 * sub_45 / 250.0f) / sub_124) / sub_54 + (sub_7 + sub_229 + sub_231) * sub_51 / sub_132;
    float sub_249 = sub_225 * sub_246 * sub_65 / sub_140;
    float sub_254 = sub_249 - 6.0f * sub_58 * sub_70 * sub_246 / sub_140;
    leg->legLen = 1.0f / Math_InvSqrt(sub_72);
    leg->vir_ang = 180.0f - Math_Rad2Angle(acos(500.0f * sub_80 / sub_83));
    leg->vir_angDiff1 = (-(500.0f * (sub_87 * sub_142 - sub_145) / sub_83 - 250.0f * (2.0f * (sub_93 + sub_59 * sub_134 / sub_140) * sub_70 - sub_145) * sub_80 / sub_160) / sub_171);
    leg->jacobian[0] = (sub_207 / sub_208);
    leg->jacobian[1] = (-(500.0f * (sub_87 * sub_204 - sub_206) / sub_83 - 250.0f * sub_207 * sub_80 / sub_160) / sub_171);
    leg->jacobian[2] = (-(sub_254) / sub_208);
    leg->jacobian[3] = (-(500.0f * (sub_225 * sub_62 * sub_246 / sub_140 - sub_249) / sub_83 + 250.0f * sub_254 * sub_80 / sub_160) / sub_171);
    return 0;
}


void Hop_CalcDiffData(Hop_LegDataTypeDef *leg) {
    float dt = DWT_GetDeltaT(&leg->decode_tick);
    leg->decode_dt = (dt == 0.0f ? 0.002f : dt);
    
    leg->len_err[2] = leg->len_err[1];
    leg->len_err[1] = leg->len_err[0];
    leg->len_err[0] = leg->legLen;
    
    leg->legLenDiff1  = Math_Differential(leg->len_err, 1, leg->decode_dt);
    leg->lenLenDiff2 = Math_Differential(leg->len_err, 2, leg->decode_dt);

    leg->vir_ang_err[2] = leg->vir_ang_err[1];
    leg->vir_ang_err[1] = leg->vir_ang_err[0];
    leg->vir_ang_err[0] = leg->vir_ang;

    leg->vir_angDiff2 = Math_Differential(leg->vir_ang_err, 2, leg->decode_dt);
}


uint8_t Hop_DecodeSupportFn(Hop_LegDataTypeDef *leg, float zm) {
    float F, Tp;
    float mw = 1;
    float Jinv[4], theta_rad[3];
    theta_rad[0] = Math_Angle2Rad(leg->vir_ang);
    theta_rad[1] = Math_Angle2Rad(leg->vir_angDiff1);
    theta_rad[2] = Math_Angle2Rad(leg->vir_angDiff2);
    float rank = leg->jacobian[0] * leg->jacobian[3] - leg->jacobian[1] * leg->jacobian[2];
    if (rank == 0.0f) return 1;
    rank = 1.0f / rank;
    Jinv[0] = rank * (leg->jacobian[3]);
    Jinv[1] = rank * (-leg->jacobian[1]);
    Jinv[2] = rank * (-leg->jacobian[2]);
    Jinv[3] = rank * (leg->jacobian[0]);
    
    F  = -(Jinv[0] * leg->fount_mot->encoder.current * 1.4f + Jinv[1] * leg->back_mot->encoder.current * 1.4f);
    Tp = -(Jinv[2] * leg->fount_mot->encoder.current * 1.4f + Jinv[3] * leg->back_mot->encoder.current * 1.4f);
    leg->zm = zm;
    leg->zw = zm - leg->lenLenDiff2 * arm_cos_f32(theta_rad[0]) + 
                2.0f * leg->legLenDiff1 * theta_rad[1] * arm_sin_f32(theta_rad[0]) +
                leg->legLen * theta_rad[2] * arm_sin_f32(theta_rad[0]) +
                leg->legLen * theta_rad[1] * theta_rad[1] * arm_cos_f32(theta_rad[0]);
    if (leg->legLen == 0) return 0;

    leg->Fn = F * arm_cos_f32(theta_rad[0]) + Tp * arm_sin_f32(theta_rad[0]) / leg->legLen +  mw * (Gravity + leg->zw);
		return 0;
}


void Hop_CalcOutput(Hop_LegDataTypeDef *leg, float bal_Tp, float roll_Tp) {
    LimitMax(bal_Tp, 8);
    LimitMax(roll_Tp, 30);
    leg->bal_fon_out = -(leg->jacobian[0] * (PID_GetPIDOutput(&leg->lenPID) + roll_Tp) + leg->jacobian[1] * (PID_GetPIDOutput(&leg->angPID) + bal_Tp));
    leg->bal_bak_out = -(leg->jacobian[2] * (PID_GetPIDOutput(&leg->lenPID) + roll_Tp) + leg->jacobian[3] * (PID_GetPIDOutput(&leg->angPID) + bal_Tp));
}
