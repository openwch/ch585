/********************************** (C) COPYRIGHT *******************************
* File Name          : ch58x_drv_ledc.h
* Author             : WCH
* Version            : V1.0
* Date               : 2024/11/20
* Description        : LED驱动头文件
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "stdint.h"
#include "CH58x_common.h"

//start of led controller
typedef enum {
    CH58X_LED_OUT_MODE_SINGLE = 0,  //data map to LED0
    CH58X_LED_OUT_MODE_DOUBLE,      //data map to LED0,LED1
    CH58X_LED_OUT_MODE_FOUR,        //data map to LED0,LED1,LED2,LED3
    CH58X_LED_OUT_MODE_FOUR_EXT,    //data map to LED0,LED1, data_aux map to LED2,LED3
}ch58x_led_out_mode_t;

/**
 * @brief   LED输出使能
 */
#define  LED_ENABLE()   (R8_LED_CTRL_MOD |= RB_LED_OUT_EN)

/**
 * @brief   LED输出失能
 */
#define  LED_DISABLE()  (R8_LED_CTRL_MOD &= ~(RB_LED_DMA_EN))

/**
 * @brief   清除LED中断标志
 *
 * @param   f       - refer to LED interrupt bit define
 */
#define LED_ClearITFlag(f)    (R16_LED_STATUS = f)

/**
 * @brief   查询LED中断标志状态
 *
 * @param   f       - refer to LED interrupt bit define
 */
#define LED_GetITFlag(f)      (R16_LED_STATUS & f)

void ch58x_led_controller_init(ch58x_led_out_mode_t mode, uint8_t led_clk_div);

void ch58x_led_controller_send(uint32_t *data, uint16_t length);

void TMR_DMACfg(uint8_t s, uint32_t startAddr, uint16_t len, DMAModeTypeDef m);

