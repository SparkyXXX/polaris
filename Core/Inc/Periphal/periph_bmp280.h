/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : periph_bmp280.h
 *  Description  : This files contains the bmp280 driver
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 00:46:10
 *  LastEditTime : 2023-08-12 00:10:29
 */


#ifndef BMP280_PERIPH_H
#define BMP280_PERIPH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "reg_bmp280.h"

typedef enum {
    BMP280_STATE_NULL      = 0,
    BMP280_STATE_CONNECTED = 1,
    BMP280_STATE_LOST      = 2
} BMP_BMP280StateEnum;

typedef struct {
    BMP_BMP280StateEnum state;
    uint8_t id;
    float altitude;
    double pressure;
    double temperature;
    
    BMP280_Offset offset;
    BMP280_S32_t t_fine;
    float update_dt;
    uint32_t last_update_tick;
} BMP280_BMP280TypeDef;


void BMP280_Init(void);
BMP280_BMP280TypeDef* BMP280_GetBMP280DataPtr(void);
uint8_t BMP280_IsBMP280Offline(void);
uint8_t BMP280_ReadID(void);
void BMP280_DecodeData(void);
uint8_t BMP280_GetStatus(uint8_t status_flag);
double BMP280_Get_Pressure(void);
double BMP280_Get_Temperature(void);
float BMP280_GetAltitude(void);
static void BMP280_Set_TemOversamp(BMP_OVERSAMPLE_MODE *Oversample_Mode);
static void BMP280_Set_Standby_FILTER(BMP_CONFIG *BMP_Config);
static double bmp280_compensate_T_double(BMP280_S32_t adc_T);
static double bmp280_compensate_P_double(BMP280_S32_t adc_P);


#endif

#ifdef __cplusplus
}
#endif
