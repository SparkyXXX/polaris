/*
 *  Project      : Polaris Robot
 * 
 *  FilePath     : periph_magnetic.h
 *  Description  : This file contains the magnetic type of ist8310
 *  LastEditors  : Polaris
 *  Date         : 2022-04-16 22:53:30
 *  LastEditTime : 2023-01-24 02:14:14
 */


#ifndef MAG_PERIPH_H
#define MAG_PERIPH_H

#ifdef __cplusplus
extern "C" {
#endif


#include "util_i2c.h"
#include "util_gpio.h"

#define MAG_SEN 0.3f                            // change to uT
#define IST8310_CHIP_ID 0x00                   // IST8310 who am I 
#define IST8310_CHIP_ID_VALUE 0x10             // ID
#define IST8310_WRITE_REG_NUM 4 
#define IST8310_IIC_ADDRESS (0x0E << 1)         
#define IST8310_IIC_READ_MSB (0x80)             
#define IST8310_DATA_READY_BIT 2
#define IST8310_NO_ERROR 0x00
#define IST8310_NO_SENSOR 0x40

typedef enum {
    MAG_STATE_NULL      = 0,
    MAG_STATE_CONNECTED = 1,
    MAG_STATE_LOST      = 2,
    MAG_STATE_ERROR     = 3,
    MAG_STATE_PENDING   = 4
} MAG_MAGStateEnum;

typedef struct {
  MAG_MAGStateEnum state;

  int error;
  float mag_x,
        mag_y,
        mag_z;
    float update_dt;
    uint32_t last_update_tick;
} MAG_MAGDataTypeDef;

void MAG_Init(void);
void MAG_ResetMAGData(void);
MAG_MAGDataTypeDef* MAG_GetMAGDataPtr(void);
uint8_t MAG_IsMAGOffline(void);
void MAG_MAGUpdate(void);

#endif

#ifdef __cplusplus
}
#endif
