/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : protocol_transmit.c
 *  Description  : This file is for transmit communication
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:18:41
 *  LastEditTime : 2023-08-22 10:07:35
 */


#include "protocol_common.h"
#include "protocol_transmit.h"
#include "stdlib.h"
#include "sys_dwt.h"
#include "sys_const.h"
#include "sys_softTimer.h"
#include "lib_buff.h"
#include "periph_bmp280.h"
#include "periph_beeper.h"
#include "periph_led.h"
#include "periph_remote.h"
#include "module_hop.h"
#include "module_gimbal.h"
#include "module_wheel.h"
#include "module_quadruped.h"
#include "app_ins.h"

static uint32_t _send_RUNTick_data(uint8_t *buff);
static uint32_t _send_IMU_data(uint8_t *buff);
static uint32_t _send_Barometer_data(uint8_t *buff);
static uint32_t _send_BasicPeriphState_data(uint8_t *buff);
static uint32_t _send_Remote_data(uint8_t *buff);
static uint32_t _send_MCUData_data(uint8_t *buff);
static uint32_t _send_WheelLeg_data(uint8_t *buff);
static uint32_t _send_Quadruped_data(uint8_t *buff);
static uint32_t _send_GimbalPitchData(uint8_t *buff);

Protocol_SendEntry ProtocolCmd_Send[Const_Protocol_Transmit_BUFF_SIZE] = {
    {&_send_RUNTick_data            },
    {&_send_IMU_data                },
    {&_send_Barometer_data          },
    {&_send_BasicPeriphState_data   },
    {&_send_Remote_data             },
    {&_send_MCUData_data            },
    {&_send_WheelLeg_data           },
    {&_send_Quadruped_data          },
    {&_send_GimbalPitchData         }
};

static uint32_t _send_RUNTick_data(uint8_t *buff) {
    DWT_DataTypeDef* dwt = DWT_GetDWTDataPtr();
    
    float2buff(dwt->SysTime.ms_tick, buff);
    return 4;
}

static uint32_t _send_IMU_data(uint8_t *buff) {
    INS_INSTypeDef *ins = INS_GetINSPtr();
    float2buff(ins->Pitch, buff);
    float2buff(ins->Roll, buff + 4);
    float2buff(ins->YawTotalAngle, buff + 8);
    float2buff(ins->Gyro[X_INS], buff + 12);
    float2buff(ins->Gyro[Y_INS], buff + 16);
    float2buff(ins->Gyro[Z_INS], buff + 20);
    float2buff(ins->Accel[X_INS], buff + 24);
    float2buff(ins->Accel[Y_INS], buff + 28);
    float2buff(ins->Accel[Z_INS], buff + 32);
    float2buff(ins->MotionAccel_b[X_INS], buff + 36);
    float2buff(ins->MotionAccel_b[Y_INS], buff + 40);
    float2buff(ins->MotionAccel_b[Z_INS], buff + 44);
    float2buff(ins->MotionAccel_n[X_INS], buff + 48);
    float2buff(ins->MotionAccel_n[Y_INS], buff + 52);
    float2buff(ins->MotionAccel_n[Z_INS], buff + 56);
    return 60;
}

static uint32_t _send_Barometer_data(uint8_t *buff) {
    BMP280_BMP280TypeDef *bmp280 = BMP280_GetBMP280DataPtr();
    float2buff((float)bmp280->pressure, buff);
    float2buff(bmp280->altitude, buff + 4);
    float2buff((float)bmp280->temperature, buff + 8);
    return 12;
}

static uint32_t _send_BasicPeriphState_data(uint8_t *buff) {
    buff[0] = (uint8_t)Beeper_MainBeeper.state;
    buff[1] = (uint8_t)LED_GetLEDState(&LED_GREEN) | ((uint8_t)LED_GetLEDState(&LED_RED) << 4); 
    buff[2] = (uint8_t)Main_Type;
    return 3;
}

static uint32_t _send_Remote_data(uint8_t *buff) {
    Remote_RemoteDataTypeDef *rc = Remote_GetRemoteDataPtr();
    buff[0] = rc->key.a | (rc->key.b << 1) | (rc->key.c << 2) | (rc->key.d << 3) |
                (rc->key.e << 4) | (rc->key.f << 5) | (rc->key.g << 6) | (rc->key.q << 7);
    buff[1] = rc->key.r | (rc->key.s << 1) | (rc->key.v << 2) | (rc->key.w << 3) |
                (rc->key.x << 4) | (rc->key.z << 5) | (rc->key.ctrl << 6) | (rc->key.shift << 7);
    buff[2] = rc->mouse.l | (rc->mouse.r << 1);
    buff[3] = (uint8_t)(rc->mouse.x >> 8);
    buff[4] = (uint8_t)(rc->mouse.x);
    buff[5] = (uint8_t)(rc->mouse.y >> 8);
    buff[6] = (uint8_t)(rc->mouse.y);
    buff[7] = (uint8_t)(rc->remote.s[0]) | ((uint8_t)(rc->remote.s[1]) << 4);
    buff[8] = (uint8_t)(rc->remote.ch[0] >> 8);
    buff[9] = (uint8_t)(rc->remote.ch[0]);
    buff[10] = (uint8_t)(rc->remote.ch[1] >> 8);
    buff[11] = (uint8_t)(rc->remote.ch[1]);
    buff[12] = (uint8_t)(rc->remote.ch[2] >> 8);
    buff[13] = (uint8_t)(rc->remote.ch[2]);
    buff[14] = (uint8_t)(rc->remote.ch[3] >> 8);
    buff[15] = (uint8_t)(rc->remote.ch[3]);
    buff[16] = (uint8_t)(rc->remote.ch[4] >> 8);
    buff[17] = (uint8_t)(rc->remote.ch[4]);
    return 18;
}

static uint32_t _send_MCUData_data(uint8_t *buff) {
    float2buff(vbat_adc, buff);
    float2buff(temp_adc, buff + 4);
    return 8;
}

static uint32_t _send_WheelLeg_data(uint8_t *buff) {
    Hop_DataTypeDef *hop_data = Hop_GetHopDataPtr();
    Wheel_DataTypeDef *wheel = Wheel_GetWheelDataPtr();

    float2buff(hop_data->left.theta_1, buff);
    float2buff(hop_data->left.theta_2, buff + 4);
    float2buff(hop_data->left.legLen, buff + 8);
    float2buff(hop_data->left.vir_ang, buff + 12);
    float2buff(hop_data->left.Fn, buff + 16);

    float2buff(hop_data->right.theta_1, buff + 20);
    float2buff(hop_data->right.theta_2, buff + 24);
    float2buff(hop_data->right.legLen, buff + 28);
    float2buff(hop_data->right.vir_ang, buff + 32);
    float2buff(hop_data->right.Fn, buff + 36);

    float2buff(wheel->state.dtheta, buff + 40);
    float2buff(wheel->state.theta, buff + 44);
    float2buff(wheel->state.dx, buff + 48);
    float2buff(wheel->state.x, buff + 52);
    float2buff(wheel->state.dphi, buff + 56);
    float2buff(wheel->state.phi, buff + 60);

    return 64;
}

static uint32_t _send_Quadruped_data(uint8_t *buff) {
    Quad_QuadrupedDataTypeDef *quad = Quad_GetQuadDataPtr();  
     
    float2buff(quad->left_back.theta[0], buff + 0);
    float2buff(quad->left_back.theta[1], buff + 4);
    float2buff(quad->left_back.theta[2], buff + 8);

    float2buff(quad->left_font.theta[0], buff + 12);
    float2buff(quad->left_font.theta[1], buff + 16);
    float2buff(quad->left_font.theta[2], buff + 20);
    
    float2buff(quad->right_back.theta[0], buff + 24);
    float2buff(quad->right_back.theta[1], buff + 28);
    float2buff(quad->right_back.theta[2], buff + 32);

    float2buff(quad->right_font.theta[0], buff + 36);
    float2buff(quad->right_font.theta[1], buff + 40);
    float2buff(quad->right_font.theta[2], buff + 44);   
    return 48;
}

static uint32_t _send_GimbalPitchData(uint8_t *buff) {
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();

    float2buff(buscomm->gimbal.pitch, buff);
    float2buff(buscomm->gimbal.yaw, buff + 4);
    float2buff(buscomm->gimbal.yaw_total, buff + 8);
    float2buff(buscomm->gimbal.temperature, buff + 12);
    return 16;
}
