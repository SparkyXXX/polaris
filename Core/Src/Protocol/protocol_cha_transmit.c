/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : protocol_cha_transmit.c
 *  Description  : This file is for transmit communication
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:18:41
 *  LastEditTime : 2023-08-22 00:20:44
 */


#include "protocol_common.h"
#include "protocol_cha_transmit.h"
#include "sys_const.h"
#include "lib_buff.h"
#include "periph_remote.h"
#include "module_gimbal.h"
#include "app_ins.h"

static uint32_t _send_PitchData(uint8_t *buff);

Protocol_ChaSendEntry ProtocolCmd_ChaSend[Const_Protocol_Cha_Transmit_BUFF_SIZE] = {
    {&_send_PitchData            }
};

static uint32_t _send_PitchData(uint8_t *buff) {
    Gimbal_DataTypeDef *gimbal = Gimbal_GetGimbalPtr();
    Remote_RemoteDataTypeDef *remote = Remote_GetRemoteDataPtr();
    INS_INSTypeDef *ins = INS_GetINSPtr();

    buff[0] = gimbal->control_state;
    buff[1] = 0;
    buff[2] = 0;
    buff[3] = gimbal->yaw_total_ang_clear_flag;
    float2buff(gimbal->pitch_dref, buff + 4);
    float2buff(ins->Pitch, buff + 8);
    return 12;
}
