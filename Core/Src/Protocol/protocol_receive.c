/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : protocol_receive.c
 *  Description  : This file is for receive communication
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:10:30
 *  LastEditTime : 2023-08-23 16:15:20
 */


#include "protocol_common.h"
#include "protocol_receive.h"
#include "stdlib.h"
#include "math.h"
#include "sys_log.h"
#include "lib_buff.h"
#include "periph_led.h"
#include "periph_beeper.h"
#include "periph_remote.h"
#include "module_gimbal.h"
#include "module_quadruped.h"
#include "app_wheelLeg.h"


static uint32_t _set_UnixTick_(uint8_t *buff);
static uint32_t _set_BasicPeriph_(uint8_t *buff);
static uint32_t _set_WheelLegData_(uint8_t *buff);
static uint32_t _set_QuadrupedData(uint8_t *buff);
static uint32_t _set_GimPitchData(uint8_t * buff);

Protocol_ReceiveEntry ProtocolCmd_Receive[Const_Protocol_Receive_BUFF_SIZE] = {
    {&_set_UnixTick_        },
    {&_set_BasicPeriph_     },
    {&_set_WheelLegData_    },
    {&_set_QuadrupedData    },
	{&_set_GimPitchData		},
};


static uint32_t _set_UnixTick_(uint8_t *buff) {
    uint32_t unix_time = 0;
	for (int8_t i = 3; i >= 0; i--) {
		unix_time <<= 8;
		unix_time |= buff[i];
	} 
    if ((unix_time - Time_To_Unix()) > 3600000) {
//        Unix_To_Time(unix_time);
    }
    return 4;
}


static uint32_t _set_BasicPeriph_(uint8_t *buff) {
    Remote_RemoteDataTypeDef *data = Remote_GetRemoteDataPtr();
    if (data->remote.ch[0] == Remote_SWITCH_DOWN) {
        LED_SetLEDState(&LED_GREEN, (LED_LEDStateEnum)(buff[0]));
        LED_SetLEDState(&LED_RED, (LED_LEDStateEnum)(buff[1]));
        Beeper_SetState(&Beeper_MainBeeper, (Beeper_BeeperStateEnum)(buff[2]), (buff[3] * 5), 0, NULL);
    }
    return 4;
}


static uint32_t _set_WheelLegData_(uint8_t *buff) {
    Remote_RemoteDataTypeDef *data = Remote_GetRemoteDataPtr();    
    if ((data->remote.ch[0] == Remote_SWITCH_DOWN) && (Main_Type == Main_WHEEL)) {
        WheelLeg_SetBalMode((WheelLeg_BalanceModeEnum)(buff[0]));
        WheelLeg_AddXRef(buff2float(buff + 1));
        WheelLeg_SetLenRef(buff2float(buff + 5));
        WheelLeg_SetRollAngRef(buff2float(buff + 9));
    }
    return 13;
}


static uint32_t _set_QuadrupedData(uint8_t *buff) {
    Remote_RemoteDataTypeDef *data = Remote_GetRemoteDataPtr();    
    Quad_QuadrupedDataTypeDef *quad = Quad_GetQuadDataPtr();
    if ((data->remote.ch[0] == Remote_SWITCH_DOWN) && (Main_Type == Main_QUADRUPED)) {    
        Quadruped_SetLegTp(&quad->left_font, buff2float(buff), buff2float(buff + 4), buff2float(buff + 8));
        Quadruped_SetLegTp(&quad->left_back, buff2float(buff + 12), buff2float(buff + 16), buff2float(buff + 20));
        Quadruped_SetLegTp(&quad->right_font, buff2float(buff + 24), buff2float(buff + 28), buff2float(buff + 32));
        Quadruped_SetLegTp(&quad->right_back, buff2float(buff + 36), buff2float(buff + 40), buff2float(buff + 44));
    }
    return 48;
}


static uint32_t _set_GimPitchData(uint8_t * buff) {
    Remote_RemoteDataTypeDef *data = Remote_GetRemoteDataPtr();
    Gimbal_DataTypeDef *gimbal = Gimbal_GetGimbalPtr();
    if (data->remote.ch[0] == Remote_SWITCH_DOWN) {
        gimbal->pitch_dref = buff2float(buff);
        gimbal->ref = buff2float(buff);

    }
    return 8;
}
