/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : protocol_gim_transmit.c
 *  Description  : This file is for transmit communication
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:18:41
 *  LastEditTime : 2023-08-22 10:04:55
 */


#include "protocol_common.h"
#include "protocol_gim_transmit.h"
#include "stdlib.h"
#include "sys_dwt.h"
#include "sys_softTimer.h"
#include "lib_buff.h"
#include "periph_bmp280.h"
#include "periph_beeper.h"
#include "periph_motor.h"
#include "periph_led.h"
#include "app_ins.h"

static uint32_t _send_GimInsData_data(uint8_t *buff);
static uint32_t _send_GimBarometer_data(uint8_t *buff);
static uint32_t _send_GimBasicPeriphState_data(uint8_t *buff);

Protocol_GimSendEntry ProtocolCmd_GimSend[Const_Protocol_Gim_Transmit_BUFF_SIZE] = {
    {&_send_GimInsData_data            },
    {&_send_GimBarometer_data          },
    {&_send_GimBasicPeriphState_data   }
};

static uint32_t _send_GimInsData_data(uint8_t *buff) {
    INS_INSTypeDef *ins = INS_GetINSPtr();

    float2buff(ins->Pitch, buff);
    float2buff(ins->Yaw, buff + 4);
    float2buff(ins->YawTotalAngle, buff + 8);
    float2buff(ins->Gyro[Z_INS], buff + 12);
    return 16;
}

static uint32_t _send_GimBarometer_data(uint8_t *buff) {
    BMP280_BMP280TypeDef *bmp280 = BMP280_GetBMP280DataPtr();
    float2buff((float)bmp280->temperature, buff);
    return 4;
}

static uint32_t _send_GimBasicPeriphState_data(uint8_t *buff) {
    buff[0] = (uint8_t)Main_Type;
    return 1;
}
