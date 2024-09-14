/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : app_wheelLeg.c
 *  Description  : Wheel Leg robot application
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:41:33
 *  LastEditTime : 2023-08-24 17:50:49
 */


#include "app_wheelLeg.h"
#include "stdio.h"
#include "app_ins.h"
#include "module_gimbal.h"
#include "module_hop.h"
#include "module_wheel.h"
#include "alg_math.h"
#include "sys_const.h"
#include "sys_dwt.h"

WheelLeg_DataTypeDef WheelLegData;


char syj_buff[128];

void WheelLeg_Task(void const * argument) {
    Hop_DataTypeDef *hop_data = Hop_GetHopDataPtr();
    Wheel_DataTypeDef *wheel = Wheel_GetWheelDataPtr();
    WheelLeg_DataTypeDef *wheelleg = WheelLeg_GetWheelLegPtr();
    INS_INSTypeDef *ins = INS_GetINSPtr();
    DWT_DataTypeDef* dwt = DWT_GetDWTDataPtr();

    forever {
        Hop_Balance(&hop_data->left, wheelleg->len);
	    Hop_Balance(&hop_data->right, wheelleg->len);

        WheelLeg_SetLQRFdb();
        WheelLeg_RollCompensate();

        Hop_DecodeSupportFn(&hop_data->left, ins->MotionAccel_b[Z_INS]);
        Hop_DecodeSupportFn(&hop_data->right, ins->MotionAccel_b[Z_INS]);

        Hop_CalcOutput(&hop_data->left, (wheelleg->bal_mode == Balance_RUN ? (Const_WHEEELEG_LQR_LEG_TP_GAIN * wheel->Tp) : 0), -wheelleg->roll_compensate);
        Hop_CalcOutput(&hop_data->right, (wheelleg->bal_mode == Balance_RUN ? (-Const_WHEEELEG_LQR_LEG_TP_GAIN * wheel->Tp) : 0), wheelleg->roll_compensate);
        WheelLeg_ChassisControl();
        
        WheelLeg_SetOutput();
        WheelLeg_SendOutput();

        wheelleg->update_dt = DWT_GetDeltaT(&wheelleg->last_update_tick);
      osDelay(2);
    }
}


void WheelLeg_Init() {
    WheelLeg_DataTypeDef *wheelleg = WheelLeg_GetWheelLegPtr();
    wheelleg->stable_flag = 0;
    wheelleg->x_ref = 0;
    wheelleg->roll_ang_ref = 0.0f;
    wheelleg->len = Const_WHEELLEG_DEFAULT_LEG_LEN;
    wheelleg->update_dt = 0;
	wheelleg->last_update_tick = 0;
    PID_InitPIDParam(&wheelleg->rollAngPIDParam, Const_BalRollParam[0][0][0], Const_BalRollParam[0][0][1], Const_BalRollParam[0][0][2], Const_BalRollParam[0][0][3], 
                    Const_BalRollParam[0][0][4], Const_BalRollParam[0][1][0], Const_BalRollParam[0][1][1], Const_BalRollParam[0][2][0], Const_BalRollParam[0][2][1], 
                    Const_BalRollParam[0][3][0], Const_BalRollParam[0][3][1], PID_POSITION);
    PID_InitPIDParam(&wheelleg->rollSpeedPIDParam, Const_BalRollParam[1][0][0], Const_BalRollParam[1][0][1], Const_BalRollParam[1][0][2], Const_BalRollParam[1][0][3], 
                    Const_BalRollParam[1][0][4], Const_BalRollParam[1][1][0], Const_BalRollParam[1][1][1], Const_BalRollParam[1][2][0], Const_BalRollParam[1][2][1], 
                    Const_BalRollParam[1][3][0], Const_BalRollParam[1][3][1], PID_POSITION);
}


WheelLeg_DataTypeDef *WheelLeg_GetWheelLegPtr() {
    return &WheelLegData;
}


void WheelLeg_ClearState() {
    WheelLeg_DataTypeDef *wheelleg = WheelLeg_GetWheelLegPtr();
    Wheel_DataTypeDef *wheel = Wheel_GetWheelDataPtr();
    wheel->left_mot->encoder.round_count = 0;
    wheel->right_mot->encoder.round_count = 0;
    wheelleg->x_ref = 0;
    wheelleg->len = Const_WHEELLEG_DEFAULT_LEG_LEN;
    Ins_ClearYawTotalCounter();
    PID_ClearPID(&wheel->balYawAngPID);
    PID_ClearPID(&wheel->balYawSpdPID);
    PID_ClearPID(&wheelleg->rollAngPID);
    PID_ClearPID(&wheelleg->rollSpeedPID);
}


void WheelLeg_SetRollAngRef(float ref) {
    WheelLeg_DataTypeDef *wheelleg = WheelLeg_GetWheelLegPtr();
    LimitMax(ref, 15.0f);
    wheelleg->roll_ang_ref = ref;
}


void WheelLeg_AddXRef(float add) {
    WheelLeg_DataTypeDef *wheelleg = WheelLeg_GetWheelLegPtr();
    LimitMax(add, 0.02f);
    wheelleg->x_ref += add;
}


void WheelLeg_SetLenRef(float ref) {
    WheelLeg_DataTypeDef *wheelleg = WheelLeg_GetWheelLegPtr();
    wheelleg->len = ref;  
    LimitMaxMin(wheelleg->len, 0.38f, 0.20f);  
}


void WheelLeg_AddLenRef(float add) {
    WheelLeg_DataTypeDef *wheelleg = WheelLeg_GetWheelLegPtr();
    LimitMax(add, 0.00045f);
    wheelleg->len += add;  
    LimitMaxMin(wheelleg->len, 0.38f, 0.20f);  
}


void WheelLeg_SetBalMode(WheelLeg_BalanceModeEnum mode) {
    WheelLeg_DataTypeDef *wheelleg = WheelLeg_GetWheelLegPtr();
    DWT_DataTypeDef* dwt = DWT_GetDWTDataPtr();
    
    static float bal_tick;
    if ((wheelleg->bal_mode != mode) && 
        (dwt->SysTime.ms_tick - bal_tick > Const_WHEELLEG_FALL_CONTINUE_TIME)) {
        if (mode == Balance_RUN) {
            wheelleg->yaw_control_tick = dwt->SysTime.ms_tick;
            WheelLeg_ClearState();
        }
        if ((mode == Balance_FALL) && (wheelleg->stable_flag)) {
            bal_tick = dwt->SysTime.ms_tick;
            wheelleg->len = Const_WHEELLEG_DEFAULT_LEG_LEN;
            wheelleg->stable_flag = 0;
        }
        wheelleg->bal_mode = mode;
    }
}


void WheelLeg_SetChassisMode(WheelLeg_ChassisModeEnum mode) {
    WheelLeg_DataTypeDef *wheelleg = WheelLeg_GetWheelLegPtr();
    static WheelLeg_ChassisModeEnum last_mode;
    if (wheelleg->bal_mode == Balance_RUN) {
        last_mode = wheelleg->cha_mode;
        wheelleg->cha_mode = mode;
    } else {
        wheelleg->cha_mode = Cha_NULL;
    }

    if ((last_mode == Cha_Gyro) && (mode != Cha_Gyro)) {
        Motor_GimbalMotor.encoder.round_count = 0;
    }
}


void WheelLeg_FallJudgment() {
    Wheel_DataTypeDef *wheel = Wheel_GetWheelDataPtr();
    WheelLeg_DataTypeDef *wheelleg = WheelLeg_GetWheelLegPtr();
    INS_INSTypeDef *ins = INS_GetINSPtr();
    DWT_DataTypeDef* dwt = DWT_GetDWTDataPtr();

    static float bal_tick;
    static uint32_t count_speed_fall = 0, count_ang_fall = 0;

    // Judge is stable ?
    if ((wheelleg->bal_mode == Balance_RUN) && (fabs(ins->Pitch) > Const_WHEELLEG_STABLE_PITCH_ANGLE))
        bal_tick = dwt->SysTime.ms_tick;

    if ((wheelleg->bal_mode == Balance_RUN) && (dwt->SysTime.ms_tick - bal_tick > Const_WHEELLEG_STABLE_TIME)) 
        wheelleg->stable_flag = 1;

    // Judge Speed over
    if (wheelleg->stable_flag && (fabs(wheel->state.dx) > Const_WHEELLEG_FALL_SPEED_THRESHOLD) && (wheelleg->bal_mode == Balance_RUN))
        count_speed_fall++;
    else
        count_speed_fall = 0;

    if (count_speed_fall > Const_WHEELLEG_SPEED_FALL_TICK)
        WheelLeg_SetBalMode(Balance_FALL);

    // Judge Ang over
    if (wheelleg->stable_flag && (fabs(Math_Rad2Angle(wheel->state.phi)) > Const_WHEELLEG_FALL_PHI_ANGLE) && (wheelleg->bal_mode == Balance_RUN))
        count_ang_fall++;
    else
        count_ang_fall = 0;

    if (count_ang_fall > Const_WHEELLEG_PHI_ANG_FALL_TICK) 
        WheelLeg_SetBalMode(Balance_FALL);


    if (wheelleg->bal_mode != Balance_RUN) 
        wheelleg->cha_mode = Cha_NULL;

    if ((wheelleg->bal_mode != Balance_RUN) && (wheelleg->bal_mode != Balance_STAND))
        WheelLeg_ClearState();
}



void WheelLeg_RollCompensate() {
    WheelLeg_DataTypeDef *wheelleg = WheelLeg_GetWheelLegPtr();
    INS_INSTypeDef *ins = INS_GetINSPtr();

    PID_SetPIDRef(&wheelleg->rollAngPID, wheelleg->roll_ang_ref);
    PID_SetPIDFdb(&wheelleg->rollAngPID, ins->Roll);
    PID_SetPIDRef(&wheelleg->rollSpeedPID, 0);
    PID_SetPIDFdb(&wheelleg->rollSpeedPID, ins->Gyro[X_INS]);

    PID_CalcPID(&wheelleg->rollAngPID, &wheelleg->rollAngPIDParam);
    PID_CalcPID(&wheelleg->rollSpeedPID, &wheelleg->rollSpeedPIDParam);

    wheelleg->roll_compensate = PID_GetPIDOutput(&wheelleg->rollAngPID) + PID_GetPIDOutput(&wheelleg->rollSpeedPID);
}


void WheelLeg_ChassisControl() {
    WheelLeg_DataTypeDef *wheelleg = WheelLeg_GetWheelLegPtr();
    INS_INSTypeDef *ins = INS_GetINSPtr();
    DWT_DataTypeDef* dwt = DWT_GetDWTDataPtr();
	static uint8_t leg_len_flag = 0;
        
    if (dwt->SysTime.ms_tick - wheelleg->yaw_control_tick < Const_WHEELLEG_STABLE_TIME) {
	    Gimbal_SetControlState(0);
        Wheel_YawCompensate(0, ins->Gyro[Z_INS], 0, 0, 1);
		leg_len_flag = 1;
    }
    else {
		if (leg_len_flag == 1) {
			wheelleg->len = Const_WHEELLEG_NORMAL_LEG_LEN;
			leg_len_flag = 0;
		}
	           
        if (wheelleg->cha_mode == Cha_NULL) {
            Wheel_YawCompensate(0, ins->Gyro[Z_INS], 0, 0, 1);
			Gimbal_SetControlState(0); 
        }
        else if (wheelleg->cha_mode == Cha_Gyro) {
            Wheel_YawCompensate(0, ins->Gyro[Z_INS], 0, Const_Chassis_Gyrp_Spd, 1);
			Gimbal_SetControlState(1); 
        }
        else if (wheelleg->cha_mode == Cha_Servo) {
            Wheel_YawCompensate(-Motor_GimbalMotor.encoder.limited_angle + Motor_GimbalMotor.encoder.init_offset, ins->Gyro[Z_INS], 0, 0, 2);
			Gimbal_SetControlState(1); 
        }
    }
}


void WheelLeg_SetLQRFdb() {
    WheelLeg_DataTypeDef *wheelleg = WheelLeg_GetWheelLegPtr();
    Hop_DataTypeDef *hop_data = Hop_GetHopDataPtr();
    Wheel_DataTypeDef *wheel = Wheel_GetWheelDataPtr();
    INS_INSTypeDef *ins = INS_GetINSPtr();
    Wheel_WheelStateTypeDef state;

    float x_avr = Math_Angle2Rad(wheel->left_mot->encoder.consequent_angle - wheel->right_mot->encoder.consequent_angle) / 2.0f * WHEELLEG_WHEEL_R;
    state.dx = Math_Angle2Rad(wheel->left_mot->encoder.speed - wheel->right_mot->encoder.speed) / 2.0f * WHEELLEG_WHEEL_R;
    state.phi = Math_Angle2Rad(ins->Pitch);
    state.dphi = ins->Gyro[1];
    state.theta = (Math_Angle2Rad(hop_data->left.vir_ang - hop_data->right.vir_ang) / 2.0f) - Math_Angle2Rad(ins->Pitch);
    state.dtheta = -(hop_data->left.vir_angDiff1 - hop_data->right.vir_angDiff1) / 2.0f;


    if (wheelleg->bal_mode == Balance_RUN) {
        state.x = x_avr - wheelleg->x_offset;
    }
    else {
        wheelleg->x_offset = x_avr;
    }

    Wheel_UpdateState(wheel, state);

    Wheel_LQRUpdate(wheel, ((hop_data->left.legLen + hop_data->right.legLen) / 2.0f), wheelleg->x_ref);
}


void WheelLeg_SetOutput() {
    WheelLeg_DataTypeDef *wheelleg = WheelLeg_GetWheelLegPtr();
    Hop_DataTypeDef *hop_data = Hop_GetHopDataPtr();
    Wheel_DataTypeDef *wheel = Wheel_GetWheelDataPtr();
    Gimbal_DataTypeDef *gimbal = Gimbal_GetGimbalPtr();
    
    if (wheelleg->bal_mode == Balance_RUN) {
        gimbal->yaw_total_ang_clear_flag = 0;

        Motor_SetMotorOutput(wheel->left_mot, Const_WHEELLEG_LQR_LEG_T_GAIN * wheel->T - wheel->yaw_compensate);
        Motor_SetMotorOutput(wheel->right_mot, -Const_WHEELLEG_LQR_LEG_T_GAIN * wheel->T - wheel->yaw_compensate);
        
	    Motor_SetMotorOutput(hop_data->left.fount_mot, hop_data->left.bal_fon_out);
	    Motor_SetMotorOutput(hop_data->left.back_mot, hop_data->left.bal_bak_out);
	    Motor_SetMotorOutput(hop_data->right.fount_mot, hop_data->right.bal_fon_out);
	    Motor_SetMotorOutput(hop_data->right.back_mot, hop_data->right.bal_bak_out);    
   
    }
    else if (wheelleg->bal_mode == Balance_STAND) {
        gimbal->yaw_total_ang_clear_flag = 1;

        Motor_SetMotorOutput(wheel->left_mot, 0);
        Motor_SetMotorOutput(wheel->right_mot, 0);

	    Motor_SetMotorOutput(hop_data->left.fount_mot, hop_data->left.bal_fon_out);
	    Motor_SetMotorOutput(hop_data->left.back_mot, hop_data->left.bal_bak_out);
	    Motor_SetMotorOutput(hop_data->right.fount_mot, hop_data->right.bal_fon_out);
	    Motor_SetMotorOutput(hop_data->right.back_mot, hop_data->right.bal_bak_out);
    }
    else {
        gimbal->yaw_total_ang_clear_flag = 1;
        
        Motor_SetMotorOutput(wheel->left_mot, 0);
        Motor_SetMotorOutput(wheel->right_mot, 0);

	    Motor_SetMotorOutput(hop_data->left.fount_mot, 0);
	    Motor_SetMotorOutput(hop_data->left.back_mot, 0);
	    Motor_SetMotorOutput(hop_data->right.fount_mot, 0);
	    Motor_SetMotorOutput(hop_data->right.back_mot, 0);        
    }
    
}


void WheelLeg_SendOutput() {
	Motor_SendMotorGroupOutput(&Motor_WheelMotors);
	
    Motor_SendMotorGroupOutput(&Motor_BackLegMotors);
	Motor_SendMotorGroupOutput(&Motor_FontLegMotors);
}
