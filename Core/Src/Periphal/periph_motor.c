/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : periph_motor.c
 *  Description  : This file contains motor control function
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 00:43:48
 *  LastEditTime : 2023-08-24 17:51:48
 */


#include "periph_motor.h"
#include "stdio.h"
#include "stdlib.h"
#include "sys_const.h"
#include "sys_dwt.h"
#include "alg_math.h"

/********** VOLATILE USER CODE **********/
Motor_MotorGroupTypeDef *Motor_groupHandle[MOTOR_GROUP_NUM];

Motor_MotorGroupTypeDef Motor_WheelMotors;      // can2 * 2
Motor_MotorGroupTypeDef Motor_BodyMotors;       // can1 * 4
Motor_MotorGroupTypeDef Motor_FontLegMotors;    // can2 * 4
Motor_MotorGroupTypeDef Motor_BackLegMotors;    // can1 * 4
Motor_MotorGroupTypeDef Motor_GimbalMotors;     // can2 * 1

Motor_MotorTypeDef Motor_LeftWheelMotor;
Motor_MotorTypeDef Motor_RightWheelMotor;

Motor_MotorTypeDef Motor_BodyLeftFrontMotor;
Motor_MotorTypeDef Motor_BodyLeftBackMotor;
Motor_MotorTypeDef Motor_BodyRightFrontMotor;
Motor_MotorTypeDef Motor_BodyRightBackMotor;

Motor_MotorTypeDef Motor_CrotchLeftFrontMotor;
Motor_MotorTypeDef Motor_CrotchLeftBackMotor;
Motor_MotorTypeDef Motor_KneeLeftFrontMotor;
Motor_MotorTypeDef Motor_KneeLeftBackMotor;

Motor_MotorTypeDef Motor_CrotchRightFrontMotor;
Motor_MotorTypeDef Motor_CrotchRightBackMotor;
Motor_MotorTypeDef Motor_KneeRightFrontMotor;
Motor_MotorTypeDef Motor_KneeRightBackMotor;

Motor_MotorTypeDef Motor_GimbalMotor;

Motor_FuncMotorTypeEnum Motor_ConnectingState;

/**
  * @brief      Motor encoder decoding callback function
  * @param      canid: CAN Handle number
  * @param      stdid: CAN identifier
  * @param      rxdata: CAN rx data buff
  * @retval     NULL
  */
void Motor_EncoderDecodeCallback(CAN_HandleTypeDef* phcan, uint32_t stdid, uint8_t rxdata[], uint32_t len) {
    for (int i = 0; i < MOTOR_GROUP_NUM; i++) {
        for (int j = 0; j < Motor_groupHandle[i]->motor_num; j++) {
            if ((phcan == Motor_groupHandle[i]->can_handle) 
                  && (stdid == Motor_groupHandle[i]->motor_handle[j]->id)) {
                Motor_groupHandle[i]->motor_handle[j]->callback(Motor_groupHandle[i]->motor_handle[j], rxdata, len);
            }
        }
    }
}


/********** VOLATILE USER CODE END **********/


/**
  * @brief      Initialize all motors
  * @param      NULL
  * @retval     NULL
  */
void Motor_InitAllMotors() {
    
    Motor_groupHandle[0] = &Motor_WheelMotors;
    Motor_InitMotorGroup(&Motor_WheelMotors, 2, &hcan2, 0x280);
    Motor_InitMotor(&Motor_RightWheelMotor, Motor_TYPE_RMD9015, 0x141, 0.1, Motor_WHEEL, rmd9015_encoder_callback, Const_RIGHT_WHEEL_MOTOR_INIT_OFFSET);
    Motor_InitMotor(&Motor_LeftWheelMotor, Motor_TYPE_RMD9015, 0x142, 0.1, Motor_WHEEL, rmd9015_encoder_callback, Const_LEFT_WHEEL_MOTOR_INIT_OFFSET);
    Motor_WheelMotors.motor_handle[0] = &Motor_RightWheelMotor;
    Motor_WheelMotors.motor_handle[1] = &Motor_LeftWheelMotor;

    Motor_groupHandle[1] = &Motor_BodyMotors;
    Motor_InitMotorGroup(&Motor_BodyMotors, 4, &hcan1, 0x1FF);
    Motor_InitMotor(&Motor_BodyRightFrontMotor, Motor_TYPE_ECA4310, 0x205, 0.1, Motor_QUADRUPED, eca4310_encoder_callback, Const_BODY_RIGHT_FRONT_MOTOR_INIT_OFFSET);
    Motor_InitMotor(&Motor_BodyLeftFrontMotor, Motor_TYPE_ECA4310, 0x206, 0.1, Motor_QUADRUPED, eca4310_encoder_callback, Const_BODY_LEFT_FRONT_MOTOR_INIT_OFFSET);
    Motor_InitMotor(&Motor_BodyRightBackMotor, Motor_TYPE_ECA4310, 0x207, 0.1, Motor_QUADRUPED, eca4310_encoder_callback, Const_BODY_RIGHT_BACK_MOTOR_INIT_OFFSET);
    Motor_InitMotor(&Motor_BodyLeftBackMotor, Motor_TYPE_ECA4310, 0x208, 0.1, Motor_QUADRUPED, eca4310_encoder_callback, Const_BODY_LEFT_BACK_MOTOR_INIT_OFFSET);
    Motor_BodyMotors.motor_handle[0] = &Motor_BodyRightFrontMotor;
    Motor_BodyMotors.motor_handle[1] = &Motor_BodyLeftFrontMotor;
    Motor_BodyMotors.motor_handle[2] = &Motor_BodyRightBackMotor;
    Motor_BodyMotors.motor_handle[3] = &Motor_BodyLeftBackMotor;

    Motor_groupHandle[2] = &Motor_FontLegMotors;
    Motor_InitMotorGroup(&Motor_FontLegMotors, 4, &hcan2, 0x1FF);
    Motor_InitMotor(&Motor_CrotchRightFrontMotor, Motor_TYPE_ECA4310, 0x205, 0.1, Motor_BASIC, eca4310_encoder_callback, Const_CRO_RIGHT_FRONT_MOTOR_INIT_OFFSET);
    Motor_InitMotor(&Motor_CrotchLeftFrontMotor, Motor_TYPE_ECA4310, 0x206, 0.1, Motor_BASIC, eca4310_encoder_callback, Const_CRO_LEFT_FRONT_MOTOR_INIT_OFFSET);
    Motor_InitMotor(&Motor_KneeRightFrontMotor, Motor_TYPE_ECA4310, 0x207, 0.1, Motor_QUADRUPED, eca4310_encoder_callback, Const_KNEE_RIGHT_FRONT_MOTOR_INIT_OFFSET);
    Motor_InitMotor(&Motor_KneeLeftFrontMotor, Motor_TYPE_ECA4310, 0x208, 0.1, Motor_QUADRUPED, eca4310_encoder_callback, Const_KNEE_LEFT_FRONT_MOTOR_INIT_OFFSET);
    Motor_FontLegMotors.motor_handle[0] = &Motor_CrotchRightFrontMotor;
    Motor_FontLegMotors.motor_handle[1] = &Motor_CrotchLeftFrontMotor;
    Motor_FontLegMotors.motor_handle[2] = &Motor_KneeRightFrontMotor;
    Motor_FontLegMotors.motor_handle[3] = &Motor_KneeLeftFrontMotor;

    Motor_groupHandle[3] = &Motor_BackLegMotors;
    Motor_InitMotorGroup(&Motor_BackLegMotors, 4, &hcan1, 0x2FF);
    Motor_InitMotor(&Motor_CrotchRightBackMotor, Motor_TYPE_ECA4310, 0x209, 0.1, Motor_BASIC, eca4310_encoder_callback, Const_CRO_RIGHT_BACK_MOTOR_INIT_OFFSET);
    Motor_InitMotor(&Motor_CrotchLeftBackMotor, Motor_TYPE_ECA4310, 0x20A, 0.1, Motor_BASIC, eca4310_encoder_callback, Const_CRO_LEFT_BACK_MOTOR_INIT_OFFSET);
    Motor_InitMotor(&Motor_KneeRightBackMotor, Motor_TYPE_ECA4310, 0x20B, 0.1, Motor_QUADRUPED, eca4310_encoder_callback, Const_KNEE_RIGHT_BACK_MOTOR_INIT_OFFSET);
    Motor_InitMotor(&Motor_KneeLeftBackMotor, Motor_TYPE_ECA4310, 0x20C, 0.1, Motor_QUADRUPED, eca4310_encoder_callback, Const_KNEE_LEFT_BACK_MOTOR_INIT_OFFSET);
    Motor_BackLegMotors.motor_handle[0] = &Motor_CrotchRightBackMotor;
    Motor_BackLegMotors.motor_handle[1] = &Motor_CrotchLeftBackMotor;
    Motor_BackLegMotors.motor_handle[2] = &Motor_KneeRightBackMotor;
    Motor_BackLegMotors.motor_handle[3] = &Motor_KneeLeftBackMotor;

    Motor_groupHandle[4] = &Motor_GimbalMotors;
    Motor_InitMotorGroup(&Motor_GimbalMotors, 3, &hcan2, 0x2FF);
    Motor_InitMotor(&Motor_GimbalMotor, Motor_TYPE_RM6020, 0x20B, 0.1, Motor_Head, gm6020_encoder_callback, Const_GIMBAL_YAW_MOTOR_INIT_OFFSET);
    Motor_GimbalMotors.motor_handle[2] = &Motor_GimbalMotor;
}


/**
  * @brief      Initialize the motor
  * @param      pmotor: Pointer to motor object
  * @param      type: Type of motor (pwm or can)
  * @param      callback: Motor callback function
  * @retval     NULL
  */
void Motor_InitMotor(Motor_MotorTypeDef* pmotor, Motor_MotorTypeEnum type, uint16_t id, float fil_param, 
                        Motor_FuncMotorTypeEnum func, Motor_EncoderCallbackFuncTypeDef callback, float motor_offset) {
    if (pmotor == NULL) return;
    pmotor->update_dt = 0;
    pmotor->last_update_tick = 0;
    pmotor->type = type;
    pmotor->func = func;
    pmotor->id = id;			
    pmotor->init = 0;			
    pmotor->output = 0;
    pmotor->callback = callback;
    pmotor->encoder.init_offset = motor_offset;
    Filter_LowPassInit(fil_param, &pmotor->fdb_fil_param);
}


/**
  * @brief      Initialization of motor group
  * @param      pgroup: Pointer to motor group
  * @param      type: Type of motor (pwm or can)
  * @param      motor_num: Number of motor group
  * @param      phcan: Pointer of can handle
  * @param      stdid: Motor id
  * @retval     NULL
  */
void Motor_InitMotorGroup(Motor_MotorGroupTypeDef* pgroup, uint8_t motor_num, CAN_HandleTypeDef* phcan, uint16_t stdid) {
    if (pgroup == NULL) return;
    pgroup->motor_num = motor_num;

    if (phcan == NULL) return;
    pgroup->can_handle = phcan;
    Can_InitTxHeader(&(pgroup->can_header), stdid, Const_Motor_MOTOR_TX_EXTID, Const_Motor_MOTOR_TX_DLC);

    for (int i = 0; i < 4; ++i) 
        pgroup->motor_handle[i] = NULL;

}


/**
  * @brief      Set output
  * @param      pmotor: Pointer to motor object
  * @param      pparam: Pointer to motor parameter object
  * @retval     NULL
  */
void Motor_SetMotorOutput(Motor_MotorTypeDef* pmotor, float output) {
    pmotor->output = output;
}


/**
  * @brief      Get motor output value
  * @param      pmotor: Pointer to motor object
  * @retval     Output value
  */
uint16_t Motor_GetMotorOutput(Motor_MotorTypeDef* pmotor) {
    if (pmotor == NULL) return 0;
    if (pmotor->type == Motor_TYPE_NOT_CONNECTED) return 0;

    int16_t ret = 0;
    if (pmotor->type == Motor_TYPE_RMD9015) {
        ret = pmotor->output * 31.25f;
        LimitMax(ret, 1900);
        return ((uint16_t)(ret & 0xff) << 8) | ((uint16_t)(ret >> 8) & 0xff);
    }
    if (pmotor->type == Motor_TYPE_ECA4310) {
        ret = (int16_t)(pmotor->output * 100.0f * 1.4f);               // output / 327.67f * 32767.0f
        return (uint16_t)ret;
    }
    if ((pmotor->type == Motor_TYPE_RM2006) || (pmotor->type == Motor_TYPE_RM6020)) {
        ret = (int16_t)(pmotor->output * 1000.0f);               // output / 10.0f * 10000.0f
        return (uint16_t)ret;
    }
    return 0;
}


/**
  * @brief      Transmitter output
  * @param      pgroup: Pointer to the motor group to send
  * @retval     NULL
  */
void Motor_SendMotorGroupOutput(Motor_MotorGroupTypeDef *pgroup) {

    if (pgroup == NULL) return;
    uint8_t txdata[8];

    txdata[0] = (uint8_t)(Motor_GetMotorOutput(pgroup->motor_handle[0]) >> 8);
    txdata[1] = (uint8_t)Motor_GetMotorOutput(pgroup->motor_handle[0]);
    txdata[2] = (uint8_t)(Motor_GetMotorOutput(pgroup->motor_handle[1]) >> 8);
    txdata[3] = (uint8_t)Motor_GetMotorOutput(pgroup->motor_handle[1]);
    txdata[4] = (uint8_t)(Motor_GetMotorOutput(pgroup->motor_handle[2]) >> 8);
    txdata[5] = (uint8_t)Motor_GetMotorOutput(pgroup->motor_handle[2]);
    txdata[6] = (uint8_t)(Motor_GetMotorOutput(pgroup->motor_handle[3]) >> 8);
    txdata[7] = (uint8_t)Motor_GetMotorOutput(pgroup->motor_handle[3]);

    Can_SendMessage(pgroup->can_handle, &(pgroup->can_header), txdata);
    
}


void Motor_SendMotorGroupsOutput() {
    for (int i = 0; i < MOTOR_GROUP_NUM; i++) {
        Motor_SendMotorGroupOutput(Motor_groupHandle[i]);
    }
}


/** 
  * @brief         Get the motor connected state
  * @retval        Motor_FuncMotorTypeEnum
 */
Motor_FuncMotorTypeEnum Motor_GetConnectingMode() {
    return Motor_ConnectingState;
}


/**
  * @brief      Judge whether any motor is offline
  * @param      NULL
  * @retval     Offline or not (1 is yes, 0 is no)
  */
uint8_t Motor_IsAnyMotorOffline() {
    uint8_t bas = 0, wheel = 0, quad = 0, head = 0;
    for (int i = 0; i < MOTOR_GROUP_NUM; ++i) {
        for (int j = 0; j < 4; ++j) {
            if (Motor_IsMotorOffline(Motor_groupHandle[i]->motor_handle[j])) {
                if (Motor_groupHandle[i]->motor_handle[j]->func == Motor_WHEEL) {
                    wheel++;
                }
                else if (Motor_groupHandle[i]->motor_handle[j]->func == Motor_QUADRUPED) {
                    quad++;
                }
                else if (Motor_groupHandle[i]->motor_handle[j]->func == Motor_BASIC) {
                    bas++;
                }
                else if (Motor_groupHandle[i]->motor_handle[j]->func == Motor_Head) {
                    head++;
                }
            }
        }
    }
    if ((bas == 0) && (wheel == 0) && (quad == 0) && (head == 1)) {
        Motor_ConnectingState = Motor_Head;
        Motor_GimbalMotor.encoder.init_offset = Const_GIMBAL_PITCH_MOTOR_INIT_OFFSET;
        return 0;
    }
    Motor_GimbalMotor.encoder.init_offset = Const_GIMBAL_YAW_MOTOR_INIT_OFFSET;

    if (bas != 4) {
        Motor_ConnectingState = Motor_NO_CONNECTED;
        return 1;
    }
    
    if ((bas == 4) && (wheel == 2)) {
        Motor_ConnectingState = Motor_WHEEL;
        return 0;
    }
    else if ((bas == 4) && (quad == 8)) {
        Motor_ConnectingState = Motor_QUADRUPED;
        return 0;
    }
    else {
        Motor_ConnectingState = Motor_BASIC;
        return 1;
    }
}


/**
  * @brief      Judge whether the motor is offline
  * @param      pmotor: Pointer to motor object
  * @retval     Offline or not (1 is yes, 0 is no)
  */
uint8_t Motor_IsMotorOffline(Motor_MotorTypeDef* pmotor) {
    if (pmotor == NULL) return 0;
    if (pmotor->type == Motor_TYPE_NOT_CONNECTED) return 0;
    if (DWT_GetDeltaTWithOutUpdate(&pmotor->last_update_tick) > Const_Motor_MOTOR_OFFLINE_TIME) 
        pmotor->type = Motor_TYPE_NOT_CONNECTED;

    return (pmotor->type != Motor_TYPE_NOT_CONNECTED);
}


/********** ENCODER CALLBACK FUNCTION **********/

/**
  * @brief      rmd9015 motor encoder callback
  * @param      pmotor: Pointer to motor object
  * @retval     NULL
  */
void rmd9015_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len) {
    
    if (pmotor == NULL) return;
    if (len != 8) return;
    if (rxbuff[0] != 0XA1) return;                  // Torque loop control command
    pmotor->encoder.last_angle = pmotor->encoder.angle;
    pmotor->encoder.temp    = (float)((int16_t)rxbuff[1]);               
    pmotor->encoder.current = (float)((int16_t)(rxbuff[3] << 8 | rxbuff[2])) / 2048.0f * 66.0f;
    pmotor->encoder.speed   = (float)((int16_t)(rxbuff[5] << 8 | rxbuff[4]));   // deg per sec
    pmotor->encoder.angle   = rxbuff[7] << 8 | rxbuff[6];
    int diff = pmotor->encoder.angle - pmotor->encoder.last_angle;      //The increase of mechanical angle is positive
    if (diff < -8192)           // Make a positive turn
        pmotor->encoder.round_count++;
    else if (diff > 8192)       // Turn around in the opposite direction
        pmotor->encoder.round_count--;
    // Calculate continuous angle
    pmotor->encoder.consequent_angle = (float)pmotor->encoder.round_count * 360.0f + 
                                       (float)pmotor->encoder.angle / 16384.0f * 360.0f +
                                        pmotor->encoder.init_offset;
    pmotor->encoder.limited_angle = (float)pmotor->encoder.angle / 16384.0f * 360.0f + 
                                        pmotor->encoder.init_offset;
    // For yaw axis processing, the small gyroscope is rotated to the same position as the PTZ according to the nearest distance after stopping
    if (pmotor->encoder.limited_angle < pmotor->encoder.init_offset - 180 && pmotor->encoder.init_offset >= 180)
        pmotor->encoder.limited_angle += 360;
    else if (pmotor->encoder.limited_angle > pmotor->encoder.init_offset + 180 && pmotor->encoder.init_offset < 180)
        pmotor->encoder.limited_angle -= 360;
    
    pmotor->type = Motor_TYPE_RMD9015;
    pmotor->update_dt = DWT_GetDeltaT(&pmotor->last_update_tick);
    
}


/**
  * @brief      Gimbal motor encoder callback
  * @param      pmotor: Pointer to motor object
  * @retval     NULL
  */
void gm6020_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len) {
    // Calculate angle difference and number of cycles
    int diff = pmotor->encoder.angle - pmotor->encoder.last_angle;      //The increase of mechanical angle is positive
    if (diff < -5500)           // Make a positive turn
        pmotor->encoder.round_count++;
    else if (diff > 5500)       // Turn around in the opposite direction
        pmotor->encoder.round_count--;

    pmotor->encoder.last_angle = pmotor->init == 1 ? pmotor->encoder.angle : (rxbuff[0] << 8 | rxbuff[1]);
    pmotor->encoder.angle   = (float)((uint16_t)((uint16_t)rxbuff[0] << 8 | (uint16_t)rxbuff[1]));
    pmotor->encoder.speed   = (float)((int16_t)((uint16_t)rxbuff[2] << 8 | (uint16_t)rxbuff[3]));
    pmotor->encoder.current = (float)((int16_t)((uint16_t)rxbuff[4] << 8 | (uint16_t)rxbuff[5]));
    pmotor->encoder.temp = (float)(rxbuff[6]); 
    pmotor->init = 1; 

    // Calculate continuous angle
    pmotor->encoder.consequent_angle = (float)pmotor->encoder.round_count * 360.0f + 
                                       (float)pmotor->encoder.angle / 8192.0f * 360.0f;
    pmotor->encoder.limited_angle = (float)pmotor->encoder.angle / 8192.0f * 360.0f;
    
    if (pmotor->encoder.limited_angle < pmotor->encoder.init_offset - 180 && pmotor->encoder.init_offset >= 180)
        pmotor->encoder.limited_angle += 360;
    else if (pmotor->encoder.limited_angle > pmotor->encoder.init_offset + 180 && pmotor->encoder.init_offset < 180)
        pmotor->encoder.limited_angle -= 360;
    
    pmotor->type = Motor_TYPE_RM6020;
    pmotor->update_dt = DWT_GetDeltaT(&pmotor->last_update_tick); 
    
}


/**
  * @brief      rm2006 motor encoder callback
  * @param      pmotor: Pointer to motor object
  * @retval     NULL
  */
void rm2006_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len) {
    
    if (pmotor == NULL) return;
    if (len != 8) return;

    pmotor->encoder.last_angle = pmotor->init == 1 ? pmotor->encoder.angle : (rxbuff[0] << 8 | rxbuff[1]);
    pmotor->encoder.angle   = rxbuff[0] << 8 | rxbuff[1];
    pmotor->encoder.speed   = (float)(rxbuff[2] << 8 | rxbuff[3]);
    pmotor->encoder.current = (float)(rxbuff[4] << 8 | rxbuff[5]);
    pmotor->encoder.temp = 0; 
    pmotor->init = 1; 
    
    int diff = pmotor->encoder.angle - pmotor->encoder.last_angle;      
    if (diff < -4096)           
        pmotor->encoder.round_count++;
    else if (diff > 4096)       
        pmotor->encoder.round_count--;
    // Calculate the shaft angle because the reduction ratio needs to be divided by 36
    pmotor->encoder.consequent_angle = (float)pmotor->encoder.round_count * 10.0f + 
                                       (float)pmotor->encoder.angle / 8192.0f * 10.0f;
    if (pmotor->encoder.round_count > 10000) {
        pmotor->encoder.consequent_angle -= 10 * pmotor->encoder.round_count; 
        pmotor->encoder.round_count = 0;
    }
    if (pmotor->encoder.round_count < -10000) {
        pmotor->encoder.consequent_angle += 10 * pmotor->encoder.round_count; 
        pmotor->encoder.round_count = 0;
    }
    if (pmotor->encoder.limited_angle < pmotor->encoder.init_offset - 180 && pmotor->encoder.init_offset >= 180)
        pmotor->encoder.limited_angle += 360;
    else if (pmotor->encoder.limited_angle > pmotor->encoder.init_offset + 180 && pmotor->encoder.init_offset < 180)
        pmotor->encoder.limited_angle -= 360;

    pmotor->type = Motor_TYPE_RM2006;
    pmotor->update_dt = DWT_GetDeltaT(&pmotor->last_update_tick);
    
}


/**
  * @brief      eca4310 motor encoder callback
  * @param      pmotor: Pointer to motor object
  * @retval     NULL
  */
void eca4310_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len) {
    
    if (pmotor == NULL) return;
    if (len != 8) return;

    pmotor->encoder.last_angle = (!pmotor->init) ? 
                                (float)((int16_t)(rxbuff[0] << 8 | rxbuff[1])) / 100.0f + 180.0f : 
                                pmotor->encoder.angle;
    pmotor->encoder.angle   = (float)((int16_t)(rxbuff[0] << 8 | rxbuff[1])) / 100.0f + 180.0f;
    pmotor->encoder.speed   = (float)((int16_t)(rxbuff[2] << 8 | rxbuff[3])) * 0.010472f;
    pmotor->encoder.current = (float)((int16_t)(rxbuff[4] << 8 | rxbuff[5])) / 100.0f;
    pmotor->encoder.temp    = ((float)((int16_t)(rxbuff[6])) -  50.0f) / 2.0f; 
    pmotor->encoder.error_code = rxbuff[7];
    if ((!pmotor->init) && (pmotor->encoder.angle >= 180.0f)) {
        pmotor->encoder.init_offset = pmotor->encoder.init_offset - 360.0f;
    }
    pmotor->init = 1;

    if ((pmotor->encoder.last_angle > 358.0f) && (pmotor->encoder.angle < 2.0f)) 
        pmotor->encoder.round_count++;
    else if ((pmotor->encoder.last_angle < 2.0f) && (pmotor->encoder.angle > 358.0f)) 
        pmotor->encoder.round_count--;

    // Calculate continuous angle
    pmotor->encoder.limited_angle = pmotor->encoder.angle + pmotor->encoder.init_offset;
    pmotor->encoder.consequent_angle = (float)(pmotor->encoder.round_count) * 360.0f + 
                                        pmotor->encoder.limited_angle;

    pmotor->type = Motor_TYPE_ECA4310;
    pmotor->update_dt = DWT_GetDeltaT(&pmotor->last_update_tick);
    
}
