/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : app_quadruped.c
 *  Description  : Quadruped robot application
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:42:20
 *  LastEditTime : 2023-08-23 14:20:27
 */


#include "app_quadruped.h"
#include "periph_motor.h"
#include "sys_dwt.h"
#include "module_gimbal.h"

void Quadruped_Task(void const * argument) {
    Quad_QuadrupedDataTypeDef *quad = Quad_GetQuadDataPtr();

    forever {
		
        Quad_DecodeTheta(&quad->left_back);
        Quad_DecodeTheta(&quad->left_font);
        Quad_DecodeTheta(&quad->right_back);
        Quad_DecodeTheta(&quad->right_font);

        Motor_SendMotorGroupOutput(&Motor_BodyMotors);
        Motor_SendMotorGroupOutput(&Motor_FontLegMotors);
        Motor_SendMotorGroupOutput(&Motor_BackLegMotors);;
		Gimbal_SetControlState(1);
		
      osDelay(2);
    }
}
