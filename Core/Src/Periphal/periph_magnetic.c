/*
 *  Project      : Polaris Robot
 * 
 *  FilePath     : periph_magnetic.c
 *  Description  : This file contains the magnetic type of ist8310
 *  LastEditors  : Polaris
 *  Date         : 2022-04-16 22:53:07
 *  LastEditTime : 2023-08-12 03:25:26
 */


#include "periph_magnetic.h"
#include "sys_const.h"
#include "sys_dwt.h"

MAG_MAGDataTypeDef MAG_MAGData;

/**
  * @brief          Magnetic init
  * @param          NULL
  * @retval         Magnetic init state
  */
void MAG_Init() {
    MAG_MAGDataTypeDef *mag_data = MAG_GetMAGDataPtr();
    
    static const uint8_t wait_time = 1;
    static const uint8_t sleep_time = 50;
    uint8_t res = 0;
    uint8_t write_num = 0;
    uint8_t MAG_WriteRegDataError[IST8310_WRITE_REG_NUM][3] = {
        {0x0B, 0x08, 0x01},
        {0x41, 0x09, 0x02},
        {0x42, 0xC0, 0x03},
        {0x0A, 0x0B, 0x04}};

    MAG_ResetMAGData();
    
    GPIO_Reset(IST8310_RST);
    DWT_Delayms(sleep_time);
    GPIO_Set(IST8310_RST);
    DWT_Delayms(sleep_time);

    I2c_ReadSingleReg(Const_MAGNETIC_I2C_HANDLER, IST8310_IIC_ADDRESS, IST8310_CHIP_ID, &res);
    if (res != IST8310_CHIP_ID_VALUE) {
        mag_data->state = MAG_STATE_LOST;
        mag_data->error = IST8310_NO_SENSOR;
        MAG_ResetMAGData();
        return;
    }

    DWT_Delayms(wait_time);

    for (write_num = 0; write_num < IST8310_WRITE_REG_NUM; write_num++) {
        I2c_WriteSingleReg(Const_MAGNETIC_I2C_HANDLER, IST8310_IIC_ADDRESS, MAG_WriteRegDataError[write_num][0], MAG_WriteRegDataError[write_num][1]);
        DWT_Delayms(wait_time);
        I2c_ReadSingleReg(Const_MAGNETIC_I2C_HANDLER, IST8310_IIC_ADDRESS, MAG_WriteRegDataError[write_num][0], &res);
        DWT_Delayms(wait_time);
        if (res != MAG_WriteRegDataError[write_num][1]) {
            mag_data->state = MAG_STATE_ERROR;
            mag_data->error = MAG_WriteRegDataError[write_num][2];
            return;
        }
    }
    
    mag_data->state = MAG_STATE_LOST;
    mag_data->error = IST8310_NO_SENSOR;
}


/**
  * @brief      Reset MAG data object
  * @param      NUULL
  * @retval     NUL
  */
void MAG_ResetMAGData() {
    MAG_MAGDataTypeDef *mag = MAG_GetMAGDataPtr();
    
    mag->update_dt = 0;
	mag->last_update_tick = 0;
    mag->mag_x = 0;
    mag->mag_y = 0;
    mag->mag_z = 0;
}


/**
  * @brief      Get pinter to the MAG data object
  * @param      NULL
  * @retval     Pointer to MAG data object
  */
MAG_MAGDataTypeDef* MAG_GetMAGDataPtr() {
    return &MAG_MAGData;
}


/**
  * @brief      Judge Magnetic offline
  * @param      NULL
  * @retval     Offline or not��1 is offline��0 is not��
  */
uint8_t MAG_IsMAGOffline() {
    MAG_MAGDataTypeDef *mag_data = MAG_GetMAGDataPtr();

    if ((DWT_GetDeltaTWithOutUpdate(&mag_data->last_update_tick)) > Const_MAG_MAG_OFFLINE_TIME)
        mag_data->state = MAG_STATE_LOST;
    return mag_data->state != MAG_STATE_CONNECTED;
}


/**
  * @brief          Magnetic init
  * @param          NULL
  * @retval         Magnetic Update
  */
void MAG_MAGUpdate() {
    MAG_MAGDataTypeDef *mag_data = MAG_GetMAGDataPtr();

    uint8_t buff[6];
    mag_data->update_dt = DWT_GetDeltaT(&mag_data->last_update_tick);
    I2c_ReadMuliReg(Const_MAGNETIC_I2C_HANDLER, IST8310_IIC_ADDRESS, 0x03, 6, buff);

    mag_data->state = MAG_STATE_PENDING;
    mag_data->mag_x = ((int16_t)((buff[1] << 8) | buff[0])) * MAG_SEN;
    mag_data->mag_y = ((int16_t)((buff[3] << 8) | buff[2])) * MAG_SEN;
    mag_data->mag_z = ((int16_t)((buff[5] << 8) | buff[4])) * MAG_SEN;
    mag_data->state = MAG_STATE_CONNECTED;
}
