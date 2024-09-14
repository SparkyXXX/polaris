/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : sys_variables.c
 *  Description  : Registering System Variables
 *  LastEditors  : Polaris
 *  Date         : 2023-08-12 13:21:03
 *  LastEditTime : 2023-08-21 15:56:39
 */


#include "sys_variables.h"
#include "sys_log.h"
#include "sys_softTimer.h"
#include "sys_dwt.h"
#include "lib_tree.h"
#include "periph_beeper.h"
#include "periph_bmi088.h"
#include "periph_bmp280.h"
#include "periph_led.h"
#include "periph_magnetic.h"
#include "periph_motor.h"
#include "periph_remote.h"
#include "module_hop.h"
#include "module_wheel.h"
#include "module_quadruped.h"
#include "module_gimbal.h"
#include "app_client.h"
#include "app_watchDog.h"
#include "app_ins.h"
#include "app_wheelLeg.h"
#include "protocol_common.h"

void Vari_Registering() {
    Tree_NodeTypeDef* util_dir = Tree_CreateDirNode(DataTreeRoot, "util");
    Tree_NodeTypeDef* periph_dir = Tree_CreateDirNode(DataTreeRoot, "periph");
    Tree_NodeTypeDef* module_dir = Tree_CreateDirNode(DataTreeRoot, "module");
    Tree_NodeTypeDef* sys_dir = Tree_CreateDirNode(DataTreeRoot, "sys");
    Tree_NodeTypeDef* protocol_dir = Tree_CreateDirNode(DataTreeRoot, "protocol");
    Tree_NodeTypeDef* app_dir = Tree_CreateDirNode(DataTreeRoot, "app");

    // util

    //****************** periph *************************
    Tree_NodeTypeDef* beeper_dir = Tree_CreateDirNode(periph_dir, "beeper");
    Tree_CreateObjNode(beeper_dir, "state", (*(uint8_t *)(&Beeper_MainBeeper.state)));
    Tree_CreateObjNode(beeper_dir, "freq", Beeper_MainBeeper.buff[0]);

    BMI088_DataTypeDef *bmi088 = BMI088_GetBMI088DataPtr();
    Tree_NodeTypeDef* bmi088_dir = Tree_CreateDirNode(periph_dir, "bmi088");
    Tree_CreateObjNode(bmi088_dir, "state", (*(uint8_t *)(&bmi088->state)));
    Tree_CreateObjNode(bmi088_dir, "temp", bmi088->temperature);
    Tree_CreateObjNode(bmi088_dir, "dt", bmi088->update_dt);
    
    BMP280_BMP280TypeDef *bmp280 = BMP280_GetBMP280DataPtr();
    Tree_NodeTypeDef* bmp280_dir = Tree_CreateDirNode(periph_dir, "bmp280");
    Tree_CreateObjNode(bmp280_dir, "state", (*(uint8_t *)(&bmp280->state)));
    Tree_CreateObjNode(bmp280_dir, "altitude", bmp280->altitude);
    Tree_CreateObjNode(bmp280_dir, "pressure", bmp280->pressure);
    Tree_CreateObjNode(bmp280_dir, "temperature", bmp280->temperature);
    Tree_CreateObjNode(bmp280_dir, "dt", bmp280->update_dt);

    Tree_NodeTypeDef* led_dir = Tree_CreateDirNode(periph_dir, "led");
    Tree_CreateObjNode(led_dir, "red_state", (*(uint8_t *)(&LED_RED.state)));
    Tree_CreateObjNode(led_dir, "green_state", (*(uint8_t *)(&LED_GREEN.state)));

    MAG_MAGDataTypeDef *mag = MAG_GetMAGDataPtr();
    Tree_NodeTypeDef* mag_dir = Tree_CreateDirNode(periph_dir, "mag");
    Tree_CreateObjNode(mag_dir, "state", (*(uint8_t *)(&mag->state)));
    Tree_CreateObjNode(mag_dir, "dt", mag->update_dt);
    Tree_CreateObjNode(mag_dir, "x_mag", mag->mag_x);
    Tree_CreateObjNode(mag_dir, "y_mag", mag->mag_y);    
    Tree_CreateObjNode(mag_dir, "z_mag", mag->mag_z);

    Tree_NodeTypeDef* motor_dir = Tree_CreateDirNode(periph_dir, "motor");
    Tree_CreateObjNode(motor_dir, "motorConnect", (*(uint8_t *)(&Motor_ConnectingState)));

    Remote_RemoteDataTypeDef *rc = Remote_GetRemoteDataPtr();
    Tree_NodeTypeDef* rc_dir = Tree_CreateDirNode(periph_dir, "remote");
    Tree_CreateObjNode(rc_dir, "state", (*(uint8_t *)(&rc->state)));
    Tree_CreateObjNode(rc_dir, "dt", rc->update_dt);

    Tree_NodeTypeDef* servo_dir = Tree_CreateDirNode(periph_dir, "servo");

    //****************** module *************************
    Hop_DataTypeDef *hop = Hop_GetHopDataPtr();
    Tree_NodeTypeDef* hop_dir = Tree_CreateDirNode(module_dir, "hop");
        Tree_NodeTypeDef* hop_left_dir = Tree_CreateDirNode(hop_dir, "left");
        Tree_CreateObjNode(hop_left_dir, "vir_ang", hop->left.vir_ang);
        Tree_CreateObjNode(hop_left_dir, "len", hop->left.legLen);
        Tree_CreateObjNode(hop_left_dir, "decode_dt", hop->left.decode_dt);
        Tree_CreateObjNode(hop_left_dir, "fn", hop->left.Fn);

        Tree_NodeTypeDef* hop_right_dir = Tree_CreateDirNode(hop_dir, "right");
        Tree_CreateObjNode(hop_right_dir, "vir_ang", hop->right.vir_ang);
        Tree_CreateObjNode(hop_right_dir, "len", hop->right.legLen);
        Tree_CreateObjNode(hop_right_dir, "decode_dt", hop->right.decode_dt);
        Tree_CreateObjNode(hop_right_dir, "fn", hop->right.Fn);  

    Wheel_DataTypeDef *wheel = Wheel_GetWheelDataPtr();
    Tree_NodeTypeDef* wheel_dir = Tree_CreateDirNode(module_dir, "wheel");
    Tree_CreateObjNode(wheel_dir, "yaw_comp", wheel->yaw_compensate);
    Tree_CreateObjNode(wheel_dir, "T", wheel->T);
    Tree_CreateObjNode(wheel_dir, "Tp", wheel->Tp); 


    Quad_QuadrupedDataTypeDef *quad = Quad_GetQuadDataPtr();
    Tree_NodeTypeDef* quad_dir = Tree_CreateDirNode(module_dir, "quad");
    
    Gimbal_DataTypeDef *gimbal = Gimbal_GetGimbalPtr();
    Tree_NodeTypeDef* gimbal_dir = Tree_CreateDirNode(module_dir, "gimbal");
    Tree_CreateObjNode(gimbal_dir, "ref", gimbal->ref);
    Tree_CreateObjNode(gimbal_dir, "gyro_dfb", gimbal->gyro_fdb);
    Tree_CreateObjNode(gimbal_dir, "ang_dfb", gimbal->position_fdb);
    Tree_CreateObjNode(gimbal_dir, "con_state", gimbal->control_state);
    Tree_CreateObjNode(gimbal_dir, "fdb_dt", gimbal->fdb_dt);
    Tree_CreateObjNode(gimbal_dir, "dt", gimbal->update_dt);
    Tree_CreateObjNode(gimbal_dir, "type", (*(uint8_t *)(&gimbal->type)));

    //****************** system *************************
    Log_DataTypeDef* log = Log_GetLogDataPtr();
    Tree_NodeTypeDef* log_dir = Tree_CreateDirNode(sys_dir, "log");
    Tree_CreateObjNode(log_dir, "save_type", (*(uint8_t *)(&log->saveType)));

    Tree_NodeTypeDef* sysadc_dir = Tree_CreateDirNode(sys_dir, "adc");
    Tree_CreateObjNode(sysadc_dir, "temp", temp_adc);
    Tree_CreateObjNode(sysadc_dir, "vrefint", vrefint_adc);
    Tree_CreateObjNode(sysadc_dir, "vbat", vbat_adc);

    DWT_DataTypeDef* dwt = DWT_GetDWTDataPtr();
    Tree_NodeTypeDef* dwt_dir = Tree_CreateDirNode(sys_dir, "dwt");
    Tree_CreateObjNode(dwt_dir, "s", dwt->SysTime.s);
    Tree_CreateObjNode(dwt_dir, "ms", dwt->SysTime.ms);
    Tree_CreateObjNode(dwt_dir, "us", dwt->SysTime.us);    

    //****************** protocol *************************
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();
    Tree_NodeTypeDef* prot_common_dir = Tree_CreateDirNode(protocol_dir, "common");
    Tree_CreateObjNode(prot_common_dir, "rx_dt", buscomm->rx_dt);
    Tree_CreateObjNode(prot_common_dir, "tx_dt", buscomm->tx_dt);

    //****************** application *************************
    Client_DataTypeDef* client = Client_GetClientPtr();
    Tree_NodeTypeDef* client_dir = Tree_CreateDirNode(app_dir, "client");
    Tree_CreateObjNode(client_dir, "interface", (*(uint8_t *)(&client->interface_flag)));

    INS_INSTypeDef* ins = INS_GetINSPtr();
    Tree_NodeTypeDef* ins_dir = Tree_CreateDirNode(app_dir, "ins");
    Tree_CreateObjNode(ins_dir, "dt", ins->update_dt);
        Tree_NodeTypeDef* ang_dir = Tree_CreateDirNode(ins_dir, "ang");
        Tree_CreateObjNode(ang_dir, "pitch", ins->Pitch);
        Tree_CreateObjNode(ang_dir, "roll", ins->Roll);
        Tree_CreateObjNode(ang_dir, "yaw", ins->Yaw);
        Tree_CreateObjNode(ang_dir, "total_yaw", ins->YawTotalAngle);

        Tree_NodeTypeDef* gyro_dir = Tree_CreateDirNode(ins_dir, "gyro");
        Tree_CreateObjNode(gyro_dir, "x", ins->Gyro[X_INS]);
        Tree_CreateObjNode(gyro_dir, "y", ins->Gyro[Y_INS]);
        Tree_CreateObjNode(gyro_dir, "z", ins->Gyro[Z_INS]);

        Tree_NodeTypeDef* accel_dir = Tree_CreateDirNode(ins_dir, "accel");
        Tree_CreateObjNode(accel_dir, "x", ins->Accel[X_INS]);
        Tree_CreateObjNode(accel_dir, "y", ins->Accel[Y_INS]);
        Tree_CreateObjNode(accel_dir, "z", ins->Accel[Z_INS]);

        Tree_NodeTypeDef* accel_b_dir = Tree_CreateDirNode(ins_dir, "accel_b");
        Tree_CreateObjNode(accel_b_dir, "x", ins->MotionAccel_b[X_INS]);
        Tree_CreateObjNode(accel_b_dir, "y", ins->MotionAccel_b[Y_INS]);
        Tree_CreateObjNode(accel_b_dir, "z", ins->MotionAccel_b[Z_INS]);

        Tree_NodeTypeDef* accel_n_dir = Tree_CreateDirNode(ins_dir, "accel_n");
        Tree_CreateObjNode(accel_n_dir, "x", ins->MotionAccel_n[X_INS]);
        Tree_CreateObjNode(accel_n_dir, "y", ins->MotionAccel_n[Y_INS]);
        Tree_CreateObjNode(accel_n_dir, "z", ins->MotionAccel_n[Z_INS]);
        
    WatchDog_DataTypeDef *watchdog = WatchDog_GetWatchDogDataPtr();
    Tree_NodeTypeDef* wtd_dir = Tree_CreateDirNode(app_dir, "wtd");
    Tree_CreateObjNode(wtd_dir, "bowl", watchdog->Dog_Bowl);
        
    WheelLeg_DataTypeDef *wheelleg = WheelLeg_GetWheelLegPtr();
    Tree_NodeTypeDef* wheelleg_dir = Tree_CreateDirNode(app_dir, "wtd");
    Tree_CreateObjNode(wheelleg_dir, "bal_mode", (*(uint8_t *)(&wheelleg->bal_mode)));
    Tree_CreateObjNode(wheelleg_dir, "cha_mode", (*(uint8_t *)(&wheelleg->cha_mode)));
    Tree_CreateObjNode(wheelleg_dir, "roll_ref", wheelleg->roll_ang_ref);
    Tree_CreateObjNode(wheelleg_dir, "dt", wheelleg->update_dt);
}
