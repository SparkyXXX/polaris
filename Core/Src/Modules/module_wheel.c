/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : module_wheel.c
 *  Description  : Balance wheel module function
 *  LastEditors  : Polaris
 *  Date         : 2023-02-08 15:36:43
 *  LastEditTime : 2023-08-23 17:46:42
 */


#include "module_wheel.h"
#include "alg_math.h"
#include "sys_const.h"

Wheel_DataTypeDef WheelData;

void Wheel_Init() {
    Wheel_DataTypeDef *wheel = Wheel_GetWheelDataPtr();
    PID_InitPIDParam(&wheel->balYawAngPIDParam, Const_BalYawParam[0][0][0], Const_BalYawParam[0][0][1], Const_BalYawParam[0][0][2], Const_BalYawParam[0][0][3], 
                    Const_BalYawParam[0][0][4], Const_BalYawParam[0][1][0], Const_BalYawParam[0][1][1], Const_BalYawParam[0][2][0], Const_BalYawParam[0][2][1], 
                    Const_BalYawParam[0][3][0], Const_BalYawParam[0][3][1], PID_POSITION);
    PID_InitPIDParam(&wheel->balYawSpdPIDParam, Const_BalYawParam[1][0][0], Const_BalYawParam[1][0][1], Const_BalYawParam[1][0][2], Const_BalYawParam[1][0][3], 
                    Const_BalYawParam[1][0][4], Const_BalYawParam[1][1][0], Const_BalYawParam[1][1][1], Const_BalYawParam[1][2][0], Const_BalYawParam[1][2][1], 
                    Const_BalYawParam[1][3][0], Const_BalYawParam[1][3][1], PID_POSITION);
    wheel->left_mot = &Motor_LeftWheelMotor;
    wheel->right_mot = &Motor_RightWheelMotor;
}


Wheel_DataTypeDef *Wheel_GetWheelDataPtr() {
    return &WheelData;
}


void Wheel_YawCompensate(float yaw, float spd, float ang_ref, float spd_ref, uint8_t ring) {
    Wheel_DataTypeDef *wheel = Wheel_GetWheelDataPtr();
    PID_SetPIDRef(&wheel->balYawAngPID, ang_ref);
    PID_SetPIDFdb(&wheel->balYawAngPID, yaw);
    PID_CalcPID(&wheel->balYawAngPID, &wheel->balYawAngPIDParam);

    PID_SetPIDRef(&wheel->balYawSpdPID, ring == 1 ? spd_ref : PID_GetPIDOutput(&wheel->balYawAngPID));
    PID_SetPIDFdb(&wheel->balYawSpdPID, spd);
    PID_CalcPID(&wheel->balYawSpdPID, &wheel->balYawSpdPIDParam);

    wheel->yaw_compensate = PID_GetPIDOutput(&wheel->balYawSpdPID);
}


void Wheel_UpdateState(Wheel_DataTypeDef *wheel, Wheel_WheelStateTypeDef state) {
    wheel->state = state;
}


void Wheel_LQRUpdate(Wheel_DataTypeDef *wheel, float len, float ref) {
    float len_3 = len * len * len; 
    float len_2 = len * len;
    for (int i = 0; i < 12; i++) {
        wheel->K[i] = LQR_PolyfitK[0][i] * len_3 + LQR_PolyfitK[1][i] * len_2 + LQR_PolyfitK[2][i] * len + LQR_PolyfitK[3][i];
        
    }
    wheel->T  = wheel->K[0] * (0 - wheel->state.theta) + wheel->K[1] * (0 - wheel->state.dtheta) + wheel->K[2] * (ref - wheel->state.x) + 
                wheel->K[3] * (0 - wheel->state.dx) + wheel->K[4] * (0 - wheel->state.phi) + wheel->K[5] * (0 - wheel->state.dphi); 
    wheel->Tp = wheel->K[6] * (0 - wheel->state.theta) + wheel->K[7] * (0 - wheel->state.dtheta) + wheel->K[8] * (ref - wheel->state.x) + 
                wheel->K[9] * (0 - wheel->state.dx) + wheel->K[10] * (0 - wheel->state.phi) + wheel->K[11] * (0 - wheel->state.dphi); 
}
