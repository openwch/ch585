/********************************** (C) COPYRIGHT *******************************
* File Name          : ch58x_drv_ledc.c
* Author             : WCH
* Version            : V1.0
* Date               : 2024/11/20
* Description        : LED驱动相关
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "ch58x_drv_ledc.h"

/*********************************************************************
 * @fn      ch58x_led_controller_init
 *
 * @brief   LED初始化
 *
 * @return  none
 */
void ch58x_led_controller_init(ch58x_led_out_mode_t mode, uint8_t led_clk_div)
{
    R8_LED_CLOCK_DIV  = led_clk_div;
    R8_LED_CTRL_MOD = 0;
    R8_LED_CTRL_MOD |= (mode<<6);
    R8_LED_CTRL_MOD |= RB_LED_BIT_ORDER;
}

/*********************************************************************
 * @fn      ch58x_led_controller_send
 *
 * @brief   LED数据使用DMA发送
 *
 * @return  none
 */
void ch58x_led_controller_send(uint32_t *data, uint16_t length)
{
    R32_LED_DMA_BEG = ((uint32_t)(data) & RB_LED_DMA_BEG);
    R16_LED_DMA_LEN = length;
    R8_LED_CTRL_MOD |= RB_LED_DMA_EN;
}
