/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : protocol_gim_receive.c
 *  Description  : This file is for receive communication
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:10:30
 *  LastEditTime : 2023-08-23 14:58:09
 */


#include "protocol_common.h"
#include "protocol_gim_receive.h"
#include "lib_buff.h"
#include "module_gimbal.h"
#include "app_ins.h"

static uint32_t _set_GimPitchRef(uint8_t *buff);


Protocol_GimReceiveEntry ProtocolCmd_GimReceive[Const_Protocol_Gim_Receive_BUFF_SIZE] = {
    {&_set_GimPitchRef         }
};

static uint32_t _set_GimPitchRef(uint8_t *buff) {
    Gimbal_DataTypeDef *gimbal = Gimbal_GetGimbalPtr();
    Gimbal_SetControlState(buff[0]);
    if (buff[3] == 1) {
        Ins_ClearYawTotalCounter();
        gimbal->ref = 0;
    }
    Gimbal_SetRef(Gimbal_LimitPitch(buff2float(buff + 4)));
    gimbal->pitch_cha_offset_ang = buff2float(buff + 8);
    return 12;
}
