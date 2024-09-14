/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : sys_log.c
 *  Description  : This files contains the log
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:22:21
 *  LastEditTime : 2023-08-12 03:34:16
 */


#include "sys_log.h"
#include "sys_const.h"
#include "rtc.h"
#include "stdlib.h"
#include "stdarg.h"
#include "util_uart.h"
#include "lib_queue.h"

Log_DataTypeDef LogData;

/**
  * @brief      Initialize log control
  * @param      NULL
  * @retval     NULL
  */
void LOG_Init() {
    Log_DataTypeDef* log = Log_GetLogDataPtr();
    FRESULT f_res;     
    Uart_InitUartDMA(Const_LOG_UART_HANDLER);
    Uart_ReceiveDMA(Const_LOG_UART_HANDLER, log->LOG_RxData, Const_LOG_BUFF_LEN);
    log->saveType = LOG_UART;
    log->log_name = "0:/polaris.txt";
	f_res = f_mount(&SDFatFS, "0:", 1);
    
    if (f_res != FR_NO_FILESYSTEM) {
        BYTE work[_MAX_SS];
        f_res = f_mkfs("0:", FM_FAT, 0, work, sizeof(work));
        if(f_res == FR_OK) {
            f_res = f_mount(NULL, "0:", 1);
            f_res = f_mount(&SDFatFS, "0:", 1);
            log->saveType = f_res == FR_OK ? LOG_ALLTYPE : LOG_UART;
        }
    }
    else if(f_res == FR_OK) {
        log->saveType = LOG_ALLTYPE;
    }
}


Log_DataTypeDef* Log_GetLogDataPtr() {
    return &LogData;
}


/** 
  * @brief       Direct printing of log records
  * @param *fmt  
  * @retval      NULL  
 */
void LOG_Printf(Log_SaveModeEnum mode, const char *fmt,...) {
    Log_DataTypeDef* log = Log_GetLogDataPtr();
    FRESULT f_res;
    UINT fnum;
    static va_list ap;
    uint16_t len = 0;
    char *buf;
    
    va_start(ap, fmt);

    RTC_DateTypeDef GetData;  
    RTC_TimeTypeDef GetTime;  
    HAL_RTC_GetTime(&hrtc, &GetTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &GetData, RTC_FORMAT_BIN);
    buf = malloc(sizeof(char) * 256);
    len = sprintf(buf, "%02d/%02d/%02d  ", 2000 + GetData.Year, GetData.Month, GetData.Date);
    len += sprintf(buf + len, "%02d:%02d:%02d  ", GetTime.Hours, GetTime.Minutes, GetTime.Seconds);

    len += vsprintf(buf + len, fmt, ap);

    len += sprintf(buf + len, "\r\n ");

    va_end(ap);    
        
    if (((log->saveType == LOG_ALLTYPE) || (log->saveType == LOG_UART)) && 
        ((mode == LOG_ALLTYPE) || (mode == LOG_UART))) {
        Uart_SendMessage(Const_LOG_UART_HANDLER, (uint8_t *)buf, (uint16_t)len, 50);
    }

    if (((log->saveType == LOG_ALLTYPE) || (log->saveType == LOG_SDCARD)) &&
        ((mode == LOG_ALLTYPE) || (mode == LOG_SDCARD))) {
        f_res = f_open(&log->file, log->log_name, FA_OPEN_ALWAYS | FA_WRITE);
			
        if (f_res == FR_OK) {
            f_res = f_lseek(&log->file, f_size(&log->file));
            f_res = f_write(&log->file, buf, len, &fnum);
        }     
        f_res = f_close(&log->file);
    }
    
    free(buf);
}


/**
  * @brief      Log control receiving callback function
  * @param      huart: Pointer to UART handle
  * @retval     NULL
  */
void LOG_RXCallback(UART_HandleTypeDef* huart) {
    /* clear DMA transfer complete flag */
    __HAL_DMA_DISABLE(huart->hdmarx);

    /* handle uart data from DMA */
    int rxdatalen = Const_LOG_BUFF_LEN - Uart_DMACurrentDataCounter(huart->hdmarx->Instance);


    /* restart dma transmission */
    __HAL_DMA_SET_COUNTER(huart->hdmarx, Const_LOG_BUFF_LEN);
    __HAL_DMA_ENABLE(huart->hdmarx);
}

uint64_t Time_To_Unix() {  
    struct tm stm;  
    RTC_DateTypeDef sdatestructureget;
    RTC_TimeTypeDef stimestructureget;

    HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);
    HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);
 
    stm.tm_year = sdatestructureget.Year + 100;  
    stm.tm_mon = sdatestructureget.Month - 1;  
    stm.tm_mday = sdatestructureget.Date;  
    stm.tm_hour = stimestructureget.Hours - 8;//北京时间-8=UTC  
    stm.tm_min = stimestructureget.Minutes;  
    stm.tm_sec = stimestructureget.Seconds; 

    return (uint32_t)self_mktime(stm);  
}
void Unix_To_Time(uint32_t unix_tick) {
	struct tm *stmU;
	RTC_DateTypeDef sdate;
	RTC_TimeTypeDef stime;
		          
	self_gmtime(stmU, unix_tick);
	sdate.Year = stmU->tm_year - 100;
	sdate.Month = stmU->tm_mon + 1;
	sdate.Date = stmU->tm_mday;
	stime.Hours = stmU->tm_hour + 8;
	stime.Minutes = stmU->tm_min;
	stime.Seconds = stmU->tm_sec;
	
	HAL_RTC_SetDate(&hrtc,&sdate,RTC_FORMAT_BIN);
	HAL_RTC_SetTime(&hrtc,&stime,RTC_FORMAT_BIN); 
}

uint32_t self_mktime(struct tm tm_now) {
    uint32_t year0 = tm_now.tm_year + 1900;
    uint32_t mon0 = tm_now.tm_mon + 1;
    uint32_t day = tm_now.tm_mday;
    uint32_t hour = tm_now.tm_hour;
    uint32_t min = tm_now.tm_min;
    uint32_t sec = tm_now.tm_sec;
	uint32_t mon = mon0, year = year0;
 
	if (0 >= (int)(mon -= 2)) {
		mon += 12;	/* Puts Feb last since it has leap day */
		year -= 1;
	}
 
	return ((((uint32_t)
		  (year / 4 - year / 100 + year / 400 + 367 * mon / 12 + day) +
		  year * 365 - 719499
	    ) * 24 + hour /* now have hours */
	  ) * 60 + min /* now have minutes */
	) * 60 + sec; /* finally seconds */
}

void self_gmtime(struct tm *tm_time, uint32_t timestamp) {
    uint32_t four_year_num;
    uint32_t one_year_hours;

    const static unsigned char Days[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    uint32_t ONE_YEAR_HOURS = 8760;        // 8760 = 365 * 24 
    uint32_t FOUR_YEAR_HOURS = 35064;      // 35064 = (365 * 3 + 366) * 24 

    if (timestamp > 0x7FFFFFFF) return;
        
    tm_time->tm_isdst = 0;
    tm_time->tm_sec = (int)(timestamp % 60);               
    timestamp /= 60;
    tm_time->tm_min = (int)(timestamp % 60);                
    timestamp /= 60;
    tm_time->tm_wday = (int)(timestamp / 24 + 4) % 7;       
    four_year_num = timestamp / FOUR_YEAR_HOURS;    
    tm_time->tm_year = (four_year_num << 2) + 70;           
    timestamp %= FOUR_YEAR_HOURS;                           
   
    while (1) {
        one_year_hours = ONE_YEAR_HOURS;
        if ((tm_time->tm_year & 3) == 0) {
            one_year_hours += 24;
        }
        if (timestamp < one_year_hours) {
            break;
        }

        tm_time->tm_year++;
        timestamp -= one_year_hours;
    }

    tm_time->tm_hour = (int)(timestamp % 24);               
    timestamp /= 24;                                        
    timestamp++;
    tm_time->tm_yday = timestamp-1;                         

    if ((tm_time->tm_year & 3) == 0) {
        if (timestamp > 60) {
            timestamp--;                                    
        }
        else if (timestamp == 60) {
            tm_time->tm_mon = 1;                            
            tm_time->tm_mday = 29;                          
            return;
        }
    }

    for (tm_time->tm_mon = 0; Days[tm_time->tm_mon] < timestamp; tm_time->tm_mon++) {
        timestamp -= Days[tm_time->tm_mon];
    }

    tm_time->tm_mday = (int)(timestamp);
}

