/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : protocol_common.c
 *  Description  : This file contains Bus communication control function
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:02:53
 *  LastEditTime : 2023-08-23 15:58:41
 */


#include "protocol_common.h"
#include "protocol_receive.h"
#include "protocol_transmit.h"
#include "protocol_gim_receive.h"
#include "protocol_gim_transmit.h"
#include "protocol_cha_receive.h"
#include "protocol_cha_transmit.h"
#include "lib_buff.h"
#include "stdlib.h"
#include "sys_dwt.h"
#include "sys_const.h"
#include "alg_crc.h"
#include "util_uart.h"
#include "util_usb.h"
#include "periph_motor.h"

Protocol_DataTypeDef Protocol_Data;

/**
  * @brief      Inter bus communication initialization
  * @param      NULL
  * @retval     NULL
  */
void Protocol_InitProtocol() {
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();
    Protocol_ResetProtocolData();
    Uart_InitUartDMA(Const_Gimbal_UART_HANDLER);
    Uart_ReceiveDMA(Const_Gimbal_UART_HANDLER, buscomm->uart_watchBuff, PROTOCOL_RECEIVE_BUFF_LEN);
}


/**
  * @brief      Gets the pointer to the bus communication data object
  * @param      NULL
  * @retval     Pointer to bus communication data object
  */
Protocol_DataTypeDef* Protocol_GetBusDataPtr() {
    return &Protocol_Data;
}


/**
  * @brief      Check whether the dual bus communication is offline
  * @param      NULL
  * @retval     NULL
  */
uint8_t Protocol_IsProtocolOffline() {
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();
    if (DWT_GetDeltaTWithOutUpdate(&buscomm->last_rx_tick) > Const_Protocol_OFFLINE_TIME) {
        buscomm->state = Protocol_STATE_LOST;
    }
    return buscomm->state != Protocol_STATE_CONNECTED;
}


/**
  * @brief      Protocol send block error handler
  * @param      NULL
  * @retval     NULL
  */
void Protocol_SendBlockError() {

}


/**
  * @brief      Reset inter bus communication data object
  * @param      NULL
  * @retval     NULL
  */
void Protocol_ResetProtocolData() {
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();
    buscomm->rx_dt = DWT_GetDeltaT(&buscomm->last_rx_tick);
    buscomm->tx_dt = DWT_GetDeltaT(&buscomm->last_tx_tick);
}


/**
  * @brief      Data sending function of serial port in inter bus communication
  * @param      NULL
  * @retval     NULL
  */
void Protocol_SendProtocolData() {
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();
    buscomm->state = Protocol_STATE_PENDING;
    buscomm->tx_dt = DWT_GetDeltaT(&buscomm->last_tx_tick);
    uint32_t len;   

    // to nuc
    len = 4;
    if (Main_Type != Main_Head) {
        buscomm->usb_sendBuff[0] = 0X5A;
        buscomm->usb_sendBuff[1] = 0XA5;
        for (int i = 0; i < Const_Protocol_Transmit_BUFF_SIZE; i++) {
            if (ProtocolCmd_Send[i].bus_func != NULL) {
                len += ProtocolCmd_Send[i].bus_func(buscomm->usb_sendBuff + len);
            }
        }
        buscomm->usb_sendBuff[2] = (uint8_t)(len & 0xff);
        buscomm->usb_sendBuff[3] = (uint8_t)((len & 0xff00) >> 8);
        buscomm->usb_sendBuff[len] = 0X7A;
        buscomm->usb_sendBuff[len + 1 ] = 0XA7;
        Usb_SendBuff(buscomm->usb_sendBuff, 256);
    }

    // for two stm32 communicate
    buscomm->uart_sendBuff[0] = 0X5A;
    buscomm->uart_sendBuff[1] = 0XA5;
    len = 4;
    if (Main_Type == Main_Head) {
        for (int i = 0; i < Const_Protocol_Gim_Transmit_BUFF_SIZE; i++) {
            if (ProtocolCmd_GimSend[i].bus_func != NULL) {
                len += ProtocolCmd_GimSend[i].bus_func(buscomm->uart_sendBuff + len);
            }
        }
    }
    else {
        for (int i = 0; i < Const_Protocol_Cha_Transmit_BUFF_SIZE; i++) {
            if (ProtocolCmd_ChaSend[i].bus_func != NULL) {
                len += ProtocolCmd_ChaSend[i].bus_func(buscomm->uart_sendBuff + len);
            }
        }
    }
    buscomm->uart_sendBuff[2] = (uint8_t)(len & 0xff);
    buscomm->uart_sendBuff[3] = (uint8_t)((len & 0xff00) >> 8);
    buscomm->uart_sendBuff[len] = 0x7A;
    buscomm->uart_sendBuff[len + 1] = 0xA7;
    Uart_SendMessageDMA(Const_Gimbal_UART_HANDLER, buscomm->uart_sendBuff, len + 2);

    buscomm->state = Protocol_STATE_CONNECTED;
}


static uint8_t Protocol_MergeAndverify(uint8_t buff[], uint32_t rxdatalen) {
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();
    static uint64_t pack_flag = 0;
    
    if (rxdatalen != 64) return 0;
    if ((buff[0] == 0x5C) && (buff[1] == 0xC5)) {
        pack_flag = ((uint64_t)buff[4] << 24) | ((uint64_t)buff[5] << 16) | ((uint64_t)buff[6] << 8) | ((uint64_t)buff[7]);
        
        memcpy(buscomm->usb_watchBuff, buff, rxdatalen);
        return 0;
    }
    if ((buff[62] == 0x7C) && (buff[63] == 0xC7)) {
        if (pack_flag == (((uint64_t)buff[58] << 24) | ((uint64_t)buff[59] << 16) | ((uint64_t)buff[60] << 8) | ((uint64_t)buff[61]))) {
            memcpy(buscomm->usb_watchBuff + rxdatalen, buff, rxdatalen);
            return 1;
        }
        else {
            return 0;
        }
    } 
	return 0;
}

static uint8_t Protocol_UartVerify(uint8_t buff[], uint32_t rxdatalen) {
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();
    
    if (rxdatalen < 6) return 0;
    if (rxdatalen > 30) return 0;
    if ((buff[0] != 0x5A) || (buff[1] != 0xA5)) return 0;
    uint32_t len = ((uint32_t)buff[2] | (((uint32_t)buff[3] << 8) & 0xff00));
    if (len + 3 > PROTOCOL_RECEIVE_BUFF_LEN) return 0;
    if ((len + 2) != rxdatalen) return 0;
    if ((buff[len] != 0x7A) || (buff[len + 1] != 0xA7)) return 0;

    return 1;
}



/**
  * @brief      Data decoding function of serial port in inter bus communication
  * @param      buff: Data buffer
  * @param      rxdatalen: data length
  * @retval     NULL
  */
void Protocol_DecodeData(uint8_t buff[], uint32_t rxdatalen) {
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();

    if (Protocol_MergeAndverify(buff, rxdatalen) == 0) return; 
    buscomm->rx_dt = DWT_GetDeltaT(&buscomm->last_rx_tick);
    uint32_t len = 8;

    if (Main_Type != Main_Head) {
        for (int i = 0; i < Const_Protocol_Receive_BUFF_SIZE; i++) {
            if (ProtocolCmd_Receive[i].bus_func != NULL) {
                len += ProtocolCmd_Receive[i].bus_func(buscomm->usb_watchBuff + len);
            }
        }
    }
}

void Protocol_UartDecode(uint8_t buff[], uint32_t rxdatalen) {
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();

    if (Protocol_UartVerify(buff, rxdatalen) == 0) return;
    buscomm->gim_rx_dt = DWT_GetDeltaT(&buscomm->last_gim_rx_tick);
    uint32_t len = 4;

    if (Main_Type == Main_Head) {
        for (int i = 0; i < Const_Protocol_Gim_Receive_BUFF_SIZE; i++) {
            if (ProtocolCmd_GimReceive[i].bus_func != NULL) {
                len += ProtocolCmd_GimReceive[i].bus_func(buscomm->uart_watchBuff + len);
            }
        }
    }
    else {
        for (int i = 0; i < Const_Protocol_Cha_Receive_BUFF_SIZE; i++) {
            if (ProtocolCmd_ChaReceive[i].bus_func != NULL) {
                len += ProtocolCmd_ChaReceive[i].bus_func(buscomm->uart_watchBuff + len);
            }
        }
    }
}

void Protocol_UartCallBack(UART_HandleTypeDef* huart) {
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();
    /* clear DMA transfer complete flag */
    __HAL_DMA_DISABLE(huart->hdmarx);

    /* handle uart data from DMA */
    int rxdatalen = PROTOCOL_RECEIVE_BUFF_LEN - Uart_DMACurrentDataCounter(huart->hdmarx->Instance);
    Protocol_UartDecode(buscomm->uart_watchBuff, rxdatalen);

    /* restart dma transmission */
    __HAL_DMA_SET_COUNTER(huart->hdmarx, PROTOCOL_RECEIVE_BUFF_LEN);
    __HAL_DMA_ENABLE(huart->hdmarx);    
}
