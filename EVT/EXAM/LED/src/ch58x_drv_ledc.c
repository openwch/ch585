/********************************** (C) COPYRIGHT *******************************
* File Name          : ch58x_drv_ledc.c
* Author             : WCH
* Version            : V1.0
* Date               : 2024/11/20
* Description        : LED�������
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "ch58x_drv_ledc.h"

/*********************************************************************
 * @fn      ch58x_led_controller_init
 *
 * @brief   LED��ʼ��
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
 * @brief   LED����ʹ��DMA����
 *
 * @return  none
 */
void ch58x_led_controller_send(uint32_t *data, uint16_t length)
{
    R32_LED_DMA_BEG = ((uint32_t)(data) & RB_LED_DMA_BEG);
    R16_LED_DMA_LEN = length;
    R8_LED_CTRL_MOD |= RB_LED_DMA_EN;
}

/*********************************************************************
 * @fn      LED_DMACfg
 *
 * @brief   ����DMA����
 *
 * @param   s           - �Ƿ��DMA����
 * @param   startAddr   - DMA ��ʼ��ַ
 * @param   len         - DMA ���ͳ���
 * @param   m           - ����DMAģʽ
 *
 * @return  none
 */
void TMR_DMACfg(uint8_t s, uint32_t startAddr, uint16_t len, DMAModeTypeDef m)
{
    if(s == DISABLE)
    {
        R8_LED_CTRL_MOD &= ~RB_LED_DMA_EN;
    }
    else
    {
        R32_LED_DMA_BEG = ((uint32_t)(startAddr)& RB_LED_DMA_BEG);
        R16_LED_DMA_LEN = len;
        if(m)
        {
            R8_LED_CTRL_MOD1 = RB_LED_DMA_LOOP;
            R8_LED_CTRL_MOD |= RB_LED_DMA_EN;
        }
        else
            R8_LED_CTRL_MOD |= RB_LED_DMA_EN;
    }
}
