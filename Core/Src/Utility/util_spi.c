/*
 *  Project      : Polaris Robot
 * 
 *  FilePath     : util_spi.c
 *  Description  : This file contains the functions of SPI
 *  LastEditors  : Polaris
 *  Date         : 2022-04-16 22:53:07
 *  LastEditTime : 2023-08-24 00:14:52
 */


#include "util_spi.h"


/**
  * @brief          Initnation SPI
  * @param          hspi: The spi handle
  * @retval         NULL
  */
void Spi_Init(SPI_HandleTypeDef *hspi) {
    hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    uint32_t ret;

    if (HAL_SPI_Init(hspi) != HAL_OK)
        Spi_ErrorHandler(ret);

}


/**
  * @brief          Receive data or command to spi address(For DMA)
  * @param          hspi: The spi handle
  * @param          pData: To be received data
  * @param          len: The data length
  * @retval         NULL
  */
void Spi_ReceiveDataDMA(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t len) {

    if ((hspi == NULL) || (pData == NULL)) 
        Spi_ErrorHandler(HAL_ERROR);

    uint32_t ret = HAL_SPI_Receive_DMA(hspi, pData, len);
    if (ret != HAL_OK) 
        Spi_ErrorHandler(ret);
    
}


/**
  * @brief          Receive data or command to spi address
  * @param          hspi: The spi handle
  * @param          pData: To be received data
  * @param          len: The data length
  * @retval         NULL
  */
void Spi_ReceiveData(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t len) {

    if ((hspi == NULL) || (pData == NULL)) 
        Spi_ErrorHandler(HAL_ERROR);

    uint32_t ret = HAL_SPI_Receive(hspi, pData, len, 10);
    if (ret != HAL_OK) 
        Spi_ErrorHandler(ret);
    
}


/**
  * @brief          Send data or command to spi address(For DMA)
  * @param          hspi: The spi handle
  * @param          pData: To be transmit data
  * @param          len: The data length
  * @retval         NULL
  */
void Spi_TransmitDataDMA(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t len) {

    if ((hspi == NULL) || (pData == NULL)) 
        Spi_ErrorHandler(HAL_ERROR);

    uint32_t ret = HAL_SPI_Transmit_DMA(hspi, pData, len);
    if (ret != HAL_OK)
        Spi_ErrorHandler(ret);
    
}


/**
  * @brief          Send data or command to spi address
  * @param          hspi: The spi handle
  * @param          pData: To be transmit data
  * @param          len: The data length
  * @retval         NULL
  */
void Spi_TransmitData(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t len) {

    if ((hspi == NULL) || (pData == NULL))
        Spi_ErrorHandler(HAL_ERROR);

    uint32_t ret = HAL_SPI_Transmit(hspi, pData, len, 10);
    if (ret != HAL_OK)
        Spi_ErrorHandler(ret);

}


/**
  * @brief          Swap a data or command to spi address
  * @param          hspi: The spi handle
  * @param          pData: To be transmit data
  * @param          len: The data length
  * @retval         NULL
  */
uint8_t Spi_SwapAbyteData(SPI_HandleTypeDef *hspi, uint8_t txdata) {
    
    uint8_t rx_data;
    uint32_t ret = HAL_SPI_TransmitReceive(hspi, &txdata, &rx_data, 1, 100);
    if (ret != HAL_OK) 
        Spi_ErrorHandler(ret);

    return rx_data;
    
}


/**
  * @brief          Swap muli data or command to spi address
  * @param          hspi: The spi handle
  * @param          pData: To be transmit data
  * @param          len: The data length
  * @retval         NULL
  */
void Spi_ReadMuliReg(SPI_HandleTypeDef *hspi, uint8_t *rx_data, uint8_t len) {
    
    while (len != 0) { 
        *rx_data = Spi_SwapAbyteData(hspi, 0x55);
        rx_data++;
        len--;
    }
}


/**
  * @brief          Swap data or command to spi address(For DMA)
  * @param          hspi: The spi handle
  * @param          pData: To be transmit data
  * @param          len: The data length
  * @retval         NULL
  */
void Spi_SwapDataDMA(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t len) {

    if ((hspi == NULL) || (pTxData == NULL) || (pRxData == NULL)) 
        Spi_ErrorHandler(HAL_ERROR);

    uint32_t ret = HAL_SPI_TransmitReceive_DMA(hspi, pTxData, pRxData, len);
    if (ret != HAL_OK)
        Spi_ErrorHandler(ret);
    
}


/**
  * @brief          Swap data or command to spi address
  * @param          hspi: The spi handle
  * @param          pData: To be transmit data
  * @param          len: The data length
  * @retval         NULL
  */
void Spi_SwapData(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t len) {

    if ((hspi == NULL) || (pTxData == NULL) || (pRxData == NULL))
        Spi_ErrorHandler(HAL_ERROR);

    uint32_t ret = HAL_SPI_TransmitReceive(hspi, pTxData, pRxData, len, 10);
    if (ret != HAL_OK) 
        Spi_ErrorHandler(ret);
    
}


/**
  * @brief      Spi error handler
  * @param      ret: error data
  * @retval     NULL
  */
void Spi_ErrorHandler(uint32_t ret) {
    while (1) {
        return;
    }
}
