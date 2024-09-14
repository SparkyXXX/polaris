/*
 *  Project      : Polaris Robot
 * 
 *  FilePath     : periph_key.c
 *  Description  : This file contains key basic functions
 *  LastEditors  : Polaris
 *  Date         : 2022-04-16 22:53:07
 *  LastEditTime : 2023-08-24 00:21:45
 */


#include "periph_key.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"


/**
  * @brief          Key event handler
  * @param          gpio: interrupt header
  * @retval         NULL
  */
void Key_KeyEventHandler(GPIO_GPIOTypeDef *gpio) {
    uint16_t event = 0;
    uint8_t trigger = 0;
    uint32_t trigger_time = 0;
    uint8_t press_event;
    trigger = (gpio->pin_state == GPIO_PIN_SET) ? RISE_TRIGGER : DOWN_TRIGGER;

    if (trigger == DOWN_TRIGGER)
        gpio->last_tick = gpio->tick;

    else if (trigger == RISE_TRIGGER) {
        trigger_time = gpio->tick - gpio->last_tick;

        if (trigger_time <= FLASE_TRIGGER_TIME) {
            return;
        }
        else if ((trigger_time >= FLASE_TRIGGER_TIME) && (trigger_time <= LONG_PRESS_TIME)) {
            press_event = SHORT_PRESS_EVENT;
        }
        else if (trigger_time >= LONG_PRESS_TIME) {
            press_event = LONG_PRESS_EVENT;
        }
        event = (uint32_t)((press_event << 8) | (gpio->event_id));
        osStatus ret = osMessagePut(Key_QueueHandle, event, 10);
    }
}

void Key_TriggerKeyEventHandler(GPIO_GPIOTypeDef *gpio) {
    uint16_t event = 0;
    uint32_t trigger_time = 0;
    uint8_t press_event;

    trigger_time = gpio->tick - gpio->last_tick;
    gpio->last_tick = gpio->tick;

    if (trigger_time <= FLASE_TRIGGER_TIME) {
        return;
    }
    else if ((trigger_time >= FLASE_TRIGGER_TIME) && (trigger_time <= LONG_PRESS_TIME)) {
        press_event = LONG_PRESS_EVENT;
    }
    else if (trigger_time >= LONG_PRESS_TIME) {
        press_event = SHORT_PRESS_EVENT;
    }
    event = (uint32_t)((press_event << 8) | (gpio->event_id));
    osStatus ret = osMessagePut(Key_TriggerQueueHandle, event, 10);
}
