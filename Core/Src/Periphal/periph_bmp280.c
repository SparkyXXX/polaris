/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : periph_bmp280.c
 *  Description  : This files contains the bmp280 driver
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 00:45:59
 *  LastEditTime : 2023-08-21 15:41:12
 */


#include "periph_bmp280.h"
#include "sys_const.h"
#include "sys_dwt.h"
#include "alg_math.h"
#include "util_i2c.h"

BMP280_BMP280TypeDef BMP280_BMP280Data;


/**             
  * @brief         BMI280 Initial
  * @retval        NULL
 */
void BMP280_Init() {
    BMP280_BMP280TypeDef *bmp280 = BMP280_GetBMP280DataPtr();
	uint8_t Lsb,Msb;
	
    BMP280_ReadID();

	I2c_ReadSingleReg(Const_BMP280_I2C_HANDLER, BMP280_ADDRESS << 1, BMP280_DIG_T1_LSB_REG, &Lsb);
	I2c_ReadSingleReg(Const_BMP280_I2C_HANDLER, BMP280_ADDRESS << 1, BMP280_DIG_T1_MSB_REG, &Msb);
	bmp280->offset.T1 = (((uint8_t)Msb) << 8) + Lsb;			
	I2c_ReadSingleReg(Const_BMP280_I2C_HANDLER, BMP280_ADDRESS << 1, BMP280_DIG_T2_LSB_REG, &Lsb);
	I2c_ReadSingleReg(Const_BMP280_I2C_HANDLER, BMP280_ADDRESS << 1, BMP280_DIG_T2_MSB_REG, &Msb);
	bmp280->offset.T2 = (((uint8_t)Msb) << 8) + Lsb;		
	I2c_ReadSingleReg(Const_BMP280_I2C_HANDLER, BMP280_ADDRESS << 1, BMP280_DIG_T3_LSB_REG, &Lsb);
	I2c_ReadSingleReg(Const_BMP280_I2C_HANDLER, BMP280_ADDRESS << 1, BMP280_DIG_T3_MSB_REG, &Msb);
	bmp280->offset.T3 = (((uint8_t)Msb) << 8) + Lsb;		
	
	I2c_ReadSingleReg(Const_BMP280_I2C_HANDLER, BMP280_ADDRESS << 1, BMP280_DIG_P1_LSB_REG, &Lsb);
	I2c_ReadSingleReg(Const_BMP280_I2C_HANDLER, BMP280_ADDRESS << 1, BMP280_DIG_P1_MSB_REG, &Msb);
	bmp280->offset.P1 = (((uint8_t)Msb) << 8) + Lsb;		
	I2c_ReadSingleReg(Const_BMP280_I2C_HANDLER, BMP280_ADDRESS << 1, BMP280_DIG_P2_LSB_REG, &Lsb);
	I2c_ReadSingleReg(Const_BMP280_I2C_HANDLER, BMP280_ADDRESS << 1, BMP280_DIG_P2_MSB_REG, &Msb);
	bmp280->offset.P2 = (((uint8_t)Msb) << 8) + Lsb;	
	I2c_ReadSingleReg(Const_BMP280_I2C_HANDLER, BMP280_ADDRESS << 1, BMP280_DIG_P3_LSB_REG, &Lsb);
	I2c_ReadSingleReg(Const_BMP280_I2C_HANDLER, BMP280_ADDRESS << 1, BMP280_DIG_P3_MSB_REG, &Msb);
	bmp280->offset.P3 = (((uint8_t)Msb) << 8) + Lsb;	
	I2c_ReadSingleReg(Const_BMP280_I2C_HANDLER, BMP280_ADDRESS << 1, BMP280_DIG_P4_LSB_REG, &Lsb);
	I2c_ReadSingleReg(Const_BMP280_I2C_HANDLER, BMP280_ADDRESS << 1, BMP280_DIG_P4_MSB_REG, &Msb);
	bmp280->offset.P4 = (((uint8_t)Msb) << 8) + Lsb;	
	I2c_ReadSingleReg(Const_BMP280_I2C_HANDLER, BMP280_ADDRESS << 1, BMP280_DIG_P5_LSB_REG, &Lsb);
	I2c_ReadSingleReg(Const_BMP280_I2C_HANDLER, BMP280_ADDRESS << 1, BMP280_DIG_P5_MSB_REG, &Msb);
	bmp280->offset.P5 = (((uint8_t)Msb) << 8) + Lsb;	
	I2c_ReadSingleReg(Const_BMP280_I2C_HANDLER, BMP280_ADDRESS << 1, BMP280_DIG_P6_LSB_REG, &Lsb);
	I2c_ReadSingleReg(Const_BMP280_I2C_HANDLER, BMP280_ADDRESS << 1, BMP280_DIG_P6_MSB_REG, &Msb);
	bmp280->offset.P6 = (((uint8_t)Msb) << 8) + Lsb;	
	I2c_ReadSingleReg(Const_BMP280_I2C_HANDLER, BMP280_ADDRESS << 1, BMP280_DIG_P7_LSB_REG, &Lsb);
	I2c_ReadSingleReg(Const_BMP280_I2C_HANDLER, BMP280_ADDRESS << 1, BMP280_DIG_P7_MSB_REG, &Msb);
	bmp280->offset.P7 = (((uint8_t)Msb) << 8) + Lsb;	
	I2c_ReadSingleReg(Const_BMP280_I2C_HANDLER, BMP280_ADDRESS << 1, BMP280_DIG_P8_LSB_REG, &Lsb);
	I2c_ReadSingleReg(Const_BMP280_I2C_HANDLER, BMP280_ADDRESS << 1, BMP280_DIG_P8_MSB_REG, &Msb);
	bmp280->offset.P8 = (((uint8_t)Msb) << 8) + Lsb;	
	I2c_ReadSingleReg(Const_BMP280_I2C_HANDLER, BMP280_ADDRESS << 1, BMP280_DIG_P9_LSB_REG, &Lsb);
	I2c_ReadSingleReg(Const_BMP280_I2C_HANDLER, BMP280_ADDRESS << 1, BMP280_DIG_P9_MSB_REG, &Msb);
	bmp280->offset.P9 = (((uint8_t)Msb) << 8) + Lsb;	

	I2c_WriteSingleReg(Const_BMP280_I2C_HANDLER, BMP280_ADDRESS << 1, BMP280_RESET_REG,BMP280_RESET_VALUE);
	
	BMP_OVERSAMPLE_MODE BMP_OVERSAMPLE_MODEStructure;
	BMP_OVERSAMPLE_MODEStructure.P_Osample = BMP280_P_MODE_3;
	BMP_OVERSAMPLE_MODEStructure.T_Osample = BMP280_T_MODE_1;
	BMP_OVERSAMPLE_MODEStructure.WORKMODE  = BMP280_NORMAL_MODE;
	BMP280_Set_TemOversamp(&BMP_OVERSAMPLE_MODEStructure);
	
	BMP_CONFIG BMP_CONFIGStructure;
	BMP_CONFIGStructure.T_SB = BMP280_T_SB1;
	BMP_CONFIGStructure.FILTER_COEFFICIENT = BMP280_FILTER_MODE_4;
	BMP_CONFIGStructure.SPI_EN = DISABLE;
	
	BMP280_Set_Standby_FILTER(&BMP_CONFIGStructure);
    BMP280_ReadID();
    HAL_Delay(10);
    bmp280->state = (BMP280_ReadID() != 0) ? BMP280_STATE_CONNECTED : BMP280_STATE_LOST;

    bmp280->update_dt = 0;
	bmp280->last_update_tick = 0;
}


/**
  * @brief      Get pinter to the bmp280 data object
  * @param      NULL
  * @retval     Pointer to bmp280 data object
  */
BMP280_BMP280TypeDef* BMP280_GetBMP280DataPtr() {
    return &BMP280_BMP280Data;
}


uint8_t BMP280_IsBMP280Offline() {
    BMP280_BMP280TypeDef *bmp280 = BMP280_GetBMP280DataPtr();

    if ((DWT_GetDeltaTWithOutUpdate(&bmp280->last_update_tick)) > Const_BMP280_OFFLINE_TIME)
        bmp280->state = BMP280_STATE_LOST;
    return bmp280->state != BMP280_STATE_CONNECTED;
}


/** 
  * @brief         Get the Ic Id
  * @retval        IC ID
 */
uint8_t BMP280_ReadID() {
    BMP280_BMP280TypeDef *bmp280 = BMP280_GetBMP280DataPtr();
    uint8_t res = 0;
	I2c_ReadSingleReg(Const_BMP280_I2C_HANDLER, BMP280_ADDRESS << 1, BMP280_CHIPID_REG, &res);
    bmp280->id = res;
    return res;
}


/** 
  * @brief         Decode the temperature and pressure data
  * @retval        NULL
 */
void BMP280_DecodeData() {
    BMP280_BMP280TypeDef *bmp280 = BMP280_GetBMP280DataPtr();
    if (BMP280_GetStatus(BMP280_MEASURING | BMP280_IM_UPDATE) == RESET) {
        bmp280->update_dt = DWT_GetDeltaT(&bmp280->last_update_tick);
        BMP280_Get_Temperature();
        BMP280_Get_Pressure();
    }
}


/** 
  * @brief         Get the decode status
  * @param         status_flag: status flag address
  * @retval        status
 */
uint8_t BMP280_GetStatus(uint8_t status_flag) {
	uint8_t flag;
    I2c_ReadSingleReg(Const_BMP280_I2C_HANDLER, BMP280_ADDRESS << 1, BMP280_STATUS_REG, &flag);
    return ((flag & status_flag) == status_flag) ? SET : RESET;
}


/** 
  * @brief         Get the Pressure
  * @retval        Pressure
 */
double BMP280_Get_Pressure() {
    BMP280_BMP280TypeDef *bmp280 = BMP280_GetBMP280DataPtr();
	uint8_t XLsb,Lsb, Msb;
	long signed Bit32;
	double pressure;
	I2c_ReadSingleReg(Const_BMP280_I2C_HANDLER, BMP280_ADDRESS << 1, BMP280_PRESSURE_XLSB_REG, &XLsb);
	I2c_ReadSingleReg(Const_BMP280_I2C_HANDLER, BMP280_ADDRESS << 1, BMP280_PRESSURE_LSB_REG, &Lsb);
	I2c_ReadSingleReg(Const_BMP280_I2C_HANDLER, BMP280_ADDRESS << 1, BMP280_PRESSURE_MSB_REG, &Msb);
	Bit32 = ((long)(Msb << 12)) | ((long)(Lsb << 4)) | (XLsb >> 4);
	pressure = bmp280_compensate_P_double(Bit32);
    bmp280->pressure = pressure;
    // hyposometric
    bmp280->altitude = ((bmp280->temperature + 273.15f) * (pow((101325.0f / pressure), 1.0f / 5.256f) - 1)) / 0.0065f;
	return pressure;
}


/** 
  * @brief         Get the temperature
  * @retval        temperature
 */
double BMP280_Get_Temperature() {
    BMP280_BMP280TypeDef *bmp280 = BMP280_GetBMP280DataPtr();
	uint8_t XLsb, Lsb, Msb;
	long signed Bit32;
	double temperature;
	I2c_ReadSingleReg(Const_BMP280_I2C_HANDLER, BMP280_ADDRESS << 1, BMP280_TEMPERATURE_XLSB_REG, &XLsb);
	I2c_ReadSingleReg(Const_BMP280_I2C_HANDLER, BMP280_ADDRESS << 1, BMP280_TEMPERATURE_LSB_REG, &Lsb);
	I2c_ReadSingleReg(Const_BMP280_I2C_HANDLER, BMP280_ADDRESS << 1, BMP280_TEMPERATURE_MSB_REG, &Msb);
	Bit32 = ((long)(Msb << 12)) | ((long)(Lsb << 4)) | (XLsb >> 4);	
	temperature = bmp280_compensate_T_double(Bit32);
    bmp280->temperature = temperature;
	return temperature;
}


/** 
  * @brief         Get the altitude
  * @retval        altitude
 */
float BMP280_GetAltitude() {
    BMP280_BMP280TypeDef *bmp280 = BMP280_GetBMP280DataPtr();
    return bmp280->altitude;
}


/** 
  * @brief         Set BMP oversampling factor MODE
  * @param         *Oversample_Mode : over sampling mode
  * @retval        NULL
 */
static void BMP280_Set_TemOversamp(BMP_OVERSAMPLE_MODE *Oversample_Mode) {
	uint8_t regtmp;
	regtmp = ((Oversample_Mode->T_Osample) << 5) |
			 ((Oversample_Mode->P_Osample) << 2) |
			 ((Oversample_Mode)->WORKMODE);
	
	I2c_WriteSingleReg(Const_BMP280_I2C_HANDLER, BMP280_ADDRESS << 1, BMP280_CTRLMEAS_REG, regtmp);
}


/** 
  * @brief         Set the holding time and filter frequency division factor
  * @param         *BMP_Config : BMP config factor
  * @retval        NULL
 */
static void BMP280_Set_Standby_FILTER(BMP_CONFIG *BMP_Config) {
	uint8_t regtmp;
	regtmp = ((BMP_Config->T_SB) << 5) |
			 ((BMP_Config->FILTER_COEFFICIENT) << 2) |
			 ((BMP_Config->SPI_EN));
	
	I2c_WriteSingleReg(Const_BMP280_I2C_HANDLER, BMP280_ADDRESS << 1, BMP280_CONFIG_REG, regtmp);
}


/** 
  * @brief         double compensate T factor
  * @param         adc_T T factor struct
  * @retval        T
 */
static double bmp280_compensate_T_double(BMP280_S32_t adc_T) {
    BMP280_BMP280TypeDef *bmp280 = BMP280_GetBMP280DataPtr();
	double var1, var2, T;
	var1 = (((double)adc_T) / 16384.0 - ((double)dig_T1) / 1024.0) * ((double)dig_T2);
	var2 = ((((double)adc_T) / 131072.0 - ((double)dig_T1) / 8192.0) *
	        (((double)adc_T) / 131072.0 - ((double) dig_T1) / 8192.0)) * ((double)dig_T3);
	bmp280->t_fine = (BMP280_S32_t)(var1 + var2);
	T = (var1 + var2) / 5120.0;
	return T;
}


/** 
  * @brief         double compensate P factor
  * @param         adc_P P factor struct
  * @retval        P
 */
static double bmp280_compensate_P_double(BMP280_S32_t adc_P) {
    BMP280_BMP280TypeDef *bmp280 = BMP280_GetBMP280DataPtr();
	double var1, var2, p;
	var1 = ((double)(bmp280->t_fine) / 2.0) - 64000.0;
	var2 = var1 * var1 * ((double)dig_P6) / 32768.0;
	var2 = var2 + var1 * ((double)dig_P5) * 2.0;
	var2 = (var2 / 4.0) + (((double)dig_P4) * 65536.0);
	var1 = (((double)dig_P3) * var1 * var1 / 524288.0 + ((double)dig_P2) * var1) / 524288.0;
	var1 = (1.0 + var1 / 32768.0) * ((double)dig_P1);
	if (var1 == 0.0) {
	    return 0; // avoid exception caused by division by zero
	}
	p = 1048576.0 - (double)adc_P;
	p = (p - (var2 / 4096.0)) * 6250.0 / var1;
	var1 = ((double)dig_P9) * p * p / 2147483648.0;
	var2 = p * ((double)dig_P8) / 32768.0;
	p = p + (var1 + var2 + ((double)dig_P7)) / 16.0;
	return p;
}
