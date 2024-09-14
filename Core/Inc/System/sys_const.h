/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : sys_const.h
 *  Description  : This file include all required constants
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:21:16
 *  LastEditTime : 2023-08-23 15:59:25
 */


#ifndef SYS_CONST_H
#define SYS_CONST_H

#ifdef __cplusplus
extern "C" {
#endif 

#include "main.h"
#include "util_spi.h"
#include "util_i2c.h"
#include "util_uart.h"
#include "util_timer.h"

extern const float Const_SERVO_INIT_OFFSET;

extern const float Const_LEFT_WHEEL_MOTOR_INIT_OFFSET;
extern const float Const_RIGHT_WHEEL_MOTOR_INIT_OFFSET;
extern const float Const_BODY_LEFT_FRONT_MOTOR_INIT_OFFSET;
extern const float Const_BODY_RIGHT_FRONT_MOTOR_INIT_OFFSET;
extern const float Const_BODY_LEFT_BACK_MOTOR_INIT_OFFSET;
extern const float Const_BODY_RIGHT_BACK_MOTOR_INIT_OFFSET;
extern const float Const_CRO_LEFT_FRONT_MOTOR_INIT_OFFSET;
extern const float Const_CRO_RIGHT_FRONT_MOTOR_INIT_OFFSET;
extern const float Const_CRO_LEFT_BACK_MOTOR_INIT_OFFSET;
extern const float Const_CRO_RIGHT_BACK_MOTOR_INIT_OFFSET;
extern const float Const_KNEE_LEFT_FRONT_MOTOR_INIT_OFFSET;
extern const float Const_KNEE_RIGHT_FRONT_MOTOR_INIT_OFFSET;
extern const float Const_KNEE_LEFT_BACK_MOTOR_INIT_OFFSET;
extern const float Const_KNEE_RIGHT_BACK_MOTOR_INIT_OFFSET;
extern const float Const_GIMBAL_YAW_MOTOR_INIT_OFFSET;   
extern const float Const_GIMBAL_PITCH_MOTOR_INIT_OFFSET; 

extern const float Const_HOP_LEFT_FOUNT_OFFSET;  
extern const float Const_HOP_LEFT_BACK_OFFSET;  
extern const float Const_HOP_RIGHT_FOUNT_OFFSET; 
extern const float Const_HOP_RIGHT_BACK_OFFSET;  

extern const float Const_QUAD_LEFT_FOUNT_1_OFFSET;
extern const float Const_QUAD_LEFT_FOUNT_2_OFFSET;
extern const float Const_QUAD_LEFT_FOUNT_3_OFFSET;

extern const float Const_QUAD_LEFT_BACK_1_OFFSET;
extern const float Const_QUAD_LEFT_BACK_2_OFFSET;
extern const float Const_QUAD_LEFT_BACK_3_OFFSET;

extern const float Const_QUAD_RIGHT_FOUNT_1_OFFSET;
extern const float Const_QUAD_RIGHT_FOUNT_2_OFFSET;
extern const float Const_QUAD_RIGHT_FOUNT_3_OFFSET;

extern const float Const_QUAD_RIGHT_BACK_1_OFFSET;
extern const float Const_QUAD_RIGHT_BACK_2_OFFSET;
extern const float Const_QUAD_RIGHT_BACK_3_OFFSET;

extern const float Const_WHEELLEG_REMOTE_YAW_GAIN;
extern float Const_WHEELLEG_REMOTE_X_GAIN;
extern const float Const_WHEELLEG_REMOTE_LEN_GAIN;

extern const float Const_WHEELLEG_DEFAULT_LEG_LEN;   
extern const float Const_WHEELLEG_NORMAL_LEG_LEN;
extern const float Const_WHEELLEG_STABLE_TIME;       
extern const float Const_WHEELLEG_FALL_CONTINUE_TIME;

extern const float Const_WHEELLEG_FALL_SPEED_THRESHOLD;
extern const float Const_WHEELLEG_STABLE_PITCH_ANGLE;
extern const float Const_WHEELLEG_FALL_PHI_ANGLE;
extern const uint32_t Const_WHEELLEG_PHI_ANG_FALL_TICK;
extern const uint32_t Const_WHEELLEG_SPEED_FALL_TICK;

extern float Const_WHEELLEG_LQR_LEG_T_GAIN;
extern float Const_WHEEELEG_LQR_LEG_TP_GAIN;

extern const float REMOTE_YAW_ANGLE_TO_REF;
extern const float REMOTE_PITCH_ANGLE_TO_REF;
extern float Const_PITCH_UMAXANGLE;
extern float Const_PITCH_DMAXANGLE;
extern float Const_YAW_MAXANGLE;
extern float Const_Chassis_Gyrp_Spd;

extern const float Const_HopLeftLenParam[4][5];
extern const float Const_HopRightLenParam[4][5];
extern const float Const_HopLeftAngParam[4][5];
extern const float Const_HopRightAngParam[4][5];
extern const float Const_BalRollParam[2][4][5];
extern const float Const_BalYawParam[2][4][5];
extern const float Const_WheelMotorCurParam[4][5];
extern const float Const_GimbalYawParam[2][4][5];
extern const float Const_GimbalPitchParam[2][4][5];

extern const float QuaternionEKF_F[36];                                   
extern float QuaternionEKF_P[36];
extern float LQR_PolyfitK[4][12];

extern const char* Shell_WelComMessage;
extern const char* Shell_EndOfMessage;
extern const char* Shell_Password;
extern const char* Shell_LoginSuccess;
extern const char* Shell_LoginFailed;
extern const char* Shell_PolarRoot;

extern SPI_HandleTypeDef* Const_BMI088_SPI_HANDLER;
extern const float Const_BMI088_OFFLINE_TIME;
extern I2C_HandleTypeDef* Const_BMP280_I2C_HANDLER;
extern const float Const_BMP280_OFFLINE_TIME;
extern I2C_HandleTypeDef* Const_MAGNETIC_I2C_HANDLER;
extern const float Const_MAG_MAG_OFFLINE_TIME;       
extern const float Const_Motor_MOTOR_OFFLINE_TIME;
extern I2C_HandleTypeDef* Const_OLED_I2C_HANDLER;
extern UART_HandleTypeDef* Const_Remote_UART_HANDLER;
extern const float Const_Remote_REMOTE_OFFLINE_TIME;
extern const float Const_Protocol_OFFLINE_TIME;
extern UART_HandleTypeDef* Const_LOG_UART_HANDLER;
extern const float Const_GimbalFdb_OFFLINE_TIME;
extern UART_HandleTypeDef* Const_Shell_UART_HANDLER;
extern TIM_HandleTypeDef* Const_FreeRTOS_RUNTIME_TIMER_HANDLER;
extern UART_HandleTypeDef* Const_Gimbal_UART_HANDLER;

#endif

#ifdef __cplusplus
}
#endif
