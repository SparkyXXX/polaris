/*
 *  Project      : Polaris Robot
 * 
 *  FilePath     : util_gpio.c
 *  Description  : This file contains the functions of GPIO
 *  LastEditors  : Polaris
 *  Date         : 2022-04-16 22:53:07
 *  LastEditTime : 2023-08-13 15:10:18
 */


#include "util_gpio.h"
#include "sys_dwt.h"
#include "periph_key.h"
#include "app_ins.h"

GPIO_GPIOTypeDef CS_ACCEL_START         = {GPIOA, GPIO_PIN_4 , 0xff, 0, GPIO_PIN_RESET};
GPIO_GPIOTypeDef CS_GYRO_START          = {GPIOB, GPIO_PIN_0 , 0xff, 0, GPIO_PIN_RESET};
GPIO_GPIOTypeDef IST8310_RST_START      = {GPIOB, GPIO_PIN_1 , 0xff, 0, GPIO_PIN_RESET};
GPIO_GPIOTypeDef IST8310_DRDY_START     = {GPIOB, GPIO_PIN_2 , 0xC1, 0, GPIO_PIN_RESET};
GPIO_GPIOTypeDef BMI_INT1_START         = {GPIOC, GPIO_PIN_4 , 0xB1, 0, GPIO_PIN_RESET};
GPIO_GPIOTypeDef BMI_INT2_START         = {GPIOC, GPIO_PIN_5 , 0xB2, 0, GPIO_PIN_RESET};
GPIO_GPIOTypeDef KEY_FUNC1_START        = {GPIOB, GPIO_PIN_14, KEY_FUNC_EVENT_ID, 0, GPIO_PIN_RESET};
GPIO_GPIOTypeDef KEY_FUNC2_START        = {GPIOB, GPIO_PIN_15, KEY_BACK_EVENT_ID, 0, GPIO_PIN_RESET};
GPIO_GPIOTypeDef BIG_KEY1_START         = {GPIOC, GPIO_PIN_0 , BIG_KEY1_EVENT_ID, 0, GPIO_PIN_RESET};
GPIO_GPIOTypeDef BIG_KEY2_START         = {GPIOC, GPIO_PIN_2 , BIG_KEY2_EVENT_ID, 0, GPIO_PIN_RESET};
    
GPIO_GPIOTypeDef *CS_ACCEL              = &CS_ACCEL_START;
GPIO_GPIOTypeDef *CS_GYRO               = &CS_GYRO_START;
GPIO_GPIOTypeDef *IST8310_RST           = &IST8310_RST_START;
GPIO_GPIOTypeDef *IST8310_DRDY          = &IST8310_DRDY_START;
GPIO_GPIOTypeDef *BMI_INT1              = &BMI_INT1_START;
GPIO_GPIOTypeDef *BMI_INT2              = &BMI_INT2_START;
GPIO_GPIOTypeDef *KEY_FUNC1             = &KEY_FUNC1_START;
GPIO_GPIOTypeDef *KEY_FUNC2             = &KEY_FUNC2_START;
GPIO_GPIOTypeDef *BIG_KEY1              = &BIG_KEY1_START;
GPIO_GPIOTypeDef *BIG_KEY2              = &BIG_KEY2_START;


/**
 * @brief        : Get the GPIO pin trigger tick
 * @param         [GPIO_GPIOTypeDef] *gpio
 * @return        [type]
 */
uint32_t GPIO_GetTriggerTick(GPIO_GPIOTypeDef *gpio) {
    return gpio->tick;
}


/**
 * @brief        : Set GPIO
 * @param         [GPIO_GPIOTypeDef] *gpio
 * @return        [type]
 */
void GPIO_Set(GPIO_GPIOTypeDef *gpio) {
    HAL_GPIO_WritePin(gpio->gpio_handle, gpio->gpio_pin, GPIO_PIN_SET);
    gpio->pin_state = GPIO_ReadPin(gpio);
}


/**
 * @brief        : Reset GPIO
 * @param         [GPIO_GPIOTypeDef] *gpio
 * @return        [type]
 */
void GPIO_Reset(GPIO_GPIOTypeDef *gpio) {
    HAL_GPIO_WritePin(gpio->gpio_handle, gpio->gpio_pin, GPIO_PIN_RESET);
    gpio->pin_state = GPIO_ReadPin(gpio);
}


/**
 * @brief        : Reset GPIO
 * @param         [GPIO_GPIOTypeDef] *gpio
 * @return        [type]
 */
GPIO_PinState GPIO_ReadPin(GPIO_GPIOTypeDef *gpio) {
    DWT_DataTypeDef* dwt = DWT_GetDWTDataPtr();
    gpio->tick = dwt->SysTime.ms_tick;
    gpio->pin_state = HAL_GPIO_ReadPin(gpio->gpio_handle, gpio->gpio_pin);
    return gpio->pin_state;
}


/**
 * @brief        : HAL GPIO interrupt call back
 * @param         [uint16_t] GPIO_Pin
 * @return        [type]
 */
void GPIO_IRQCallback(uint16_t GPIO_Pin) {
    GPIO_PinState pin_state = GPIO_PIN_RESET;
    switch (GPIO_Pin) {
        case Fun1_Pin:
            pin_state = GPIO_ReadPin(KEY_FUNC1);
            Key_KeyEventHandler(KEY_FUNC1);
            break;
        case Fun2_Pin:
            pin_state = GPIO_ReadPin(KEY_FUNC2);
            Key_KeyEventHandler(KEY_FUNC2);
            break;
        case BigKey1_Pin:
            pin_state = GPIO_ReadPin(BIG_KEY1);
            Key_TriggerKeyEventHandler(BIG_KEY1);
            break;
        case BigKey2_Pin:
            pin_state = GPIO_ReadPin(BIG_KEY2);
            Key_TriggerKeyEventHandler(BIG_KEY2);
            break;
        case BMI088_INT1_Pin:
            pin_state = GPIO_ReadPin(BMI_INT1);
            if (pin_state == GPIO_PIN_SET)
                Ins_GPIOExitCallback(BMI_INT1);
            break;
        case BMI088_INT2_Pin:
            pin_state = GPIO_ReadPin(BMI_INT2);
            if (pin_state == GPIO_PIN_SET)
                Ins_GPIOExitCallback(BMI_INT2);
            break;
        default:
            break;
    }
}
