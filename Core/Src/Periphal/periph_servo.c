/*
 *  Project      : Polaris Robot
 * 
 *  FilePath     : periph_servo.c
 *  Description  : This document contains the relevant auxiliary functions of steering gear control
 *  LastEditors  : Polaris
 *  Date         : 2022-04-16 22:53:07
 *  LastEditTime : 2023-01-24 02:22:57
 */


#include "periph_servo.h"
#include "sys_const.h"

/**
  * @brief      Initialize all steering gear
  * @param      NULL
  * @retval     NULL
  */
void Servo_InitAllServos() {
//    Servo_InitServo(&Servo_ammoContainerCapServo, &htim1, TIM_CHANNEL_1);
}


/**
  * @brief      Return to steering gear status
  * @param      servo: The pointer points to the actuator object
  * @retval     Steering gear status
  */
Servo_ServoStateEnum Servo_GetServoState(Servo_ServoTypeDef* servo) {
    return servo->state;
}


/**
  * @brief      Start the steering gear
  * @param      servo: The pointer points to the actuator object
  * @retval     NULL
  */
void Servo_StartServo(Servo_ServoTypeDef* servo) {
    servo->state = Servo_ON;
    PWM_StartPWM(&(servo->pwm));
}


/**
  * @brief      Stop the servo
  * @param      servo: The pointer points to the actuator object
  * @retval     NULL
  */
void Servo_StopServo(Servo_ServoTypeDef* servo) {
    servo->state = Servo_OFF;
    PWM_StopPWM(&(servo->pwm));
}


/**
  * @brief      Return to steering angle
  * @param      servo: The pointer points to the actuator object
  * @retval     Steering gear angle
  */
float Servo_GetServoAngle(Servo_ServoTypeDef* servo) {
    return servo->angle;
}


/**
  * @brief      Set steering angle
  * @param      servo: The pointer points to the actuator object
  * @param      angle: Steering gear angle
  * @retval     NULL
  */
void Servo_SetServoAngle(Servo_ServoTypeDef* servo, float angle) {
    servo->angle = angle;
    PWM_SetPWMDuty(&(servo->pwm), angle / 275.0f * 0.1f + Const_SERVO_INIT_OFFSET);
}


/**
  * @brief      Initialize the steering gear
  * @param      servo: Pointer to steering gear object
  * @param      htim: Timer handle
  * @param      ch: PWM channel number
  * @retval     NULL
  */
void Servo_InitServo(Servo_ServoTypeDef* servo, TIM_HandleTypeDef* htim, uint32_t ch) {
    servo->state = Servo_OFF;
    
    PWM_InitPWM(&(servo->pwm), htim, ch);
    PWM_SetPWMFreq(&(servo->pwm), 50);
    Servo_StartServo(servo);
    Servo_SetServoAngle(servo, 200.0f);
}
