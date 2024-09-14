/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : periph_motor.h
 *  Description  : This file contains motor control function
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 00:44:03
 *  LastEditTime : 2023-08-24 17:51:33
 */
#ifndef MOTOR_PERIPH_H
#define MOTOR_PERIPH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "util_can.h"
#include "alg_math.h"
#include "alg_filter.h"

#define MOTOR_GROUP_NUM 5
#define Const_Motor_MOTOR_TX_EXTID  0x01
#define Const_Motor_MOTOR_TX_DLC    8
#define Const_Motor_MOTOR_RX_DLC    8

typedef enum {      
    Motor_TYPE_NOT_CONNECTED    = 0,
    Motor_TYPE_RMD9015          = 1,
    Motor_TYPE_ECA4310          = 2,
    Motor_TYPE_RM2006           = 3,
    Motor_TYPE_RM6020           = 4
} Motor_MotorTypeEnum;  // Motor drive type

typedef enum {
    Motor_NO_CONNECTED  = 0,
    Motor_WHEEL         = 1,
    Motor_QUADRUPED     = 2,
    Motor_BASIC         = 3,
    Motor_Head          = 4,
} Motor_FuncMotorTypeEnum;

typedef struct {
    // Motor feedback
    float angle;
    float speed;
    float current;
    float temp;               // ℃
    uint8_t error_code;

    // Used to calculate continuous angles
    float last_angle;
    int32_t round_count;
    float init_offset;
    float limited_angle;
    float consequent_angle;
} Motor_EncoderTypeDef;

typedef struct _motor_type {
    Motor_EncoderTypeDef encoder;
    Motor_MotorTypeEnum type;
    Motor_FuncMotorTypeEnum func;

    float output;
    uint32_t id;
    uint8_t init;
    void (*callback)(struct _motor_type*, uint8_t rxbuff[], uint32_t len);
    Filter_LowPassParamTypeDef fdb_fil_param;
    Filter_LowPassTypeDef fdb_fil;

    float update_dt;
    uint32_t last_update_tick;
} Motor_MotorTypeDef;

typedef struct {
    uint8_t motor_num;
    Motor_MotorTypeDef* motor_handle[4];
    Motor_MotorTypeEnum type;
    CAN_HandleTypeDef* can_handle;
    CAN_TxHeaderTypeDef can_header;
} Motor_MotorGroupTypeDef;

typedef void (*Motor_EncoderCallbackFuncTypeDef)(Motor_MotorTypeDef*, uint8_t[], uint32_t);

extern Motor_MotorGroupTypeDef *Motor_groupHandle[MOTOR_GROUP_NUM];
extern Motor_MotorGroupTypeDef Motor_WheelMotors;      // can2 * 2
extern Motor_MotorGroupTypeDef Motor_BodyMotors;       // can1 * 4
extern Motor_MotorGroupTypeDef Motor_FontLegMotors;    // can2 * 4
extern Motor_MotorGroupTypeDef Motor_BackLegMotors;    // can1 * 4
extern Motor_MotorGroupTypeDef Motor_GimbalMotors;     // can2 * 1

extern Motor_MotorTypeDef Motor_LeftWheelMotor;
extern Motor_MotorTypeDef Motor_RightWheelMotor;
extern Motor_MotorTypeDef Motor_BodyLeftFrontMotor;
extern Motor_MotorTypeDef Motor_BodyLeftBackMotor;
extern Motor_MotorTypeDef Motor_BodyRightFrontMotor;
extern Motor_MotorTypeDef Motor_BodyRightBackMotor;
extern Motor_MotorTypeDef Motor_CrotchLeftFrontMotor;
extern Motor_MotorTypeDef Motor_CrotchLeftBackMotor;
extern Motor_MotorTypeDef Motor_KneeLeftFrontMotor;
extern Motor_MotorTypeDef Motor_KneeLeftBackMotor;
extern Motor_MotorTypeDef Motor_CrotchRightFrontMotor;
extern Motor_MotorTypeDef Motor_CrotchRightBackMotor;
extern Motor_MotorTypeDef Motor_KneeRightFrontMotor;
extern Motor_MotorTypeDef Motor_KneeRightBackMotor;
extern Motor_MotorTypeDef Motor_GimbalMotor;

/********** VOLATILE USER CODE END **********/
extern Motor_FuncMotorTypeEnum Motor_ConnectingState;

void Motor_EncoderDecodeCallback(CAN_HandleTypeDef* phcan, uint32_t stdid, uint8_t rxdata[], uint32_t len);
void Motor_InitAllMotors(void);
void Motor_InitMotor(Motor_MotorTypeDef* pmotor, Motor_MotorTypeEnum type, uint16_t id, float fil_param, 
                        Motor_FuncMotorTypeEnum func, Motor_EncoderCallbackFuncTypeDef callback, float motor_offset);
void Motor_InitMotorGroup(Motor_MotorGroupTypeDef* pgroup, uint8_t motor_num, CAN_HandleTypeDef* phcan, uint16_t stdid);
void Motor_SetMotorOutput(Motor_MotorTypeDef* pmotor, float output);
uint16_t Motor_GetMotorOutput(Motor_MotorTypeDef* pmotor);
void Motor_SendMotorGroupOutput(Motor_MotorGroupTypeDef *pgroup);
Motor_FuncMotorTypeEnum Motor_GetConnectingMode(void);
void Motor_SendMotorGroupsOutput(void);
uint8_t Motor_IsAnyMotorOffline(void);
uint8_t Motor_IsMotorOffline(Motor_MotorTypeDef* pmotor);
void rmd9015_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len);
void rm2006_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len);
void eca4310_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len);
void gm6020_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len);

#endif

#ifdef __cplusplus
}
#endif
