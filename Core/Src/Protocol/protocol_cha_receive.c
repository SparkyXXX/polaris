/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : protocol_cha_receive.c
 *  Description  : This file is for receive communication
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:10:30
 *  LastEditTime : 2023-08-22 10:06:15
 */


#include "protocol_common.h"
#include "protocol_cha_receive.h"
#include "lib_buff.h"
#include "periph_motor.h"
#include "module_gimbal.h"

static uint32_t _set_ChaInsData_(uint8_t *buff);
static uint32_t _set_ChaBarometer_(uint8_t *buff);
static uint32_t _set_ChaBasicPeriphState_(uint8_t *buff);

Protocol_ChaReceiveEntry ProtocolCmd_ChaReceive[Const_Protocol_Cha_Receive_BUFF_SIZE] = {
    {&_set_ChaInsData_          },
    {&_set_ChaBarometer_        },
    {&_set_ChaBasicPeriphState_ },
};


static uint32_t _set_ChaInsData_(uint8_t *buff) {
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();
    buscomm->gimbal.pitch = buff2float(buff);
    buscomm->gimbal.yaw = buff2float(buff + 4);
    buscomm->gimbal.yaw_total = buff2float(buff + 8);
    buscomm->gimbal.gyro_z = buff2float(buff + 12);

    Gimbal_SetGyroFdb(buscomm->gimbal.gyro_z);
    Gimbal_SetPositionFdb(buscomm->gimbal.yaw_total);

    return 16;
}

static uint32_t _set_ChaBarometer_(uint8_t *buff) {
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();
    buscomm->gimbal.temperature = buff2float(buff);
    return 4;
}

static uint32_t _set_ChaBasicPeriphState_(uint8_t *buff) {
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();
    buscomm->gimbal.motor_mode = buff[0];
    return 1;
}
