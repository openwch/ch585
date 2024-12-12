/********************************** (C) COPYRIGHT *******************************
 * File Name          : Main.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2022/03/15
 * Description        : USBHS IAP例程
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CH58x_common.h"
#include "iap.h"
#include "usb_desc.h"

/*********************************************************************
 * @fn      SetSysClock
 *
 * @brief   配置系统运行时钟62.4Mhz
 *
 * @param   sc      - 系统时钟源选择 refer to SYS_CLKTypeDef
 *
 * @return  none
 */
__HIGH_CODE
void mySetSysClock()
{
    R32_SAFE_MODE_CTRL |= RB_XROM_312M_SEL;
    R8_SAFE_MODE_CTRL &= ~RB_SAFE_AUTO_EN;
    sys_safe_access_enable();
    R8_HFCK_PWR_CTRL |= RB_CLK_RC16M_PON;
    __nop();
    __nop();
    R8_HFCK_PWR_CTRL |= RB_CLK_PLL_PON;
    __nop();
    __nop();
    R8_FLASH_SCK = R8_FLASH_SCK & (~(1<<4));
    R8_FLASH_CFG = 0X02;
    __nop();
    R16_CLK_SYS_CFG = CLK_SOURCE_HSI_PLL_62_4MHz | 0xc0;
    R16_CLK_SYS_CFG = CLK_SOURCE_HSI_PLL_62_4MHz;
    __nop();
    R8_SAFE_MODE_CTRL |= RB_SAFE_AUTO_EN;
    sys_safe_access_disable();
}

/*********************************************************************
 * @fn      Main_Circulation
 *
 * @brief   IAP主循环,程序放ram中运行，提升速度.
 *
 * @param   None.
 *
 * @return  None.
 */
__HIGH_CODE
void Main_Circulation()
{
    uint16_t j = 0;
    while (1)
    {
        j++;
        if (j > 5)//100us处理一次数据
        {
            j = 0;
            USB_DevTransProcess();//采用查询方式进行usb操作，不使用中断。
        }
        DelayUs(20);
        g_tcnt++;
        if (g_tcnt > 3000000)
        {
            //1分钟没有操作，进入app
            R8_USB2_CTRL = USBHS_UD_RST_SIE;
            R16_PIN_CONFIG &= ~RB_PIN_USB2_EN;
            DelayMs(10);
            jumpApp();
        }
    }
}
IAPDataFlashInfo_t p_image_flash;

/*********************************************************************
 * @fn      main
 *
 * @brief   主函数
 *
 * @return  none
 */
int main()
{
    uint16_t i;
    uint8_t  s;

    mySetSysClock(); //为了精简程序体积，该函数比普通库的初始化函数有修改，只可以将时钟设置为62.4M

#if USE_EEPROM_FLAG
    EEPROM_READ(IAP_FLAG_DATAFLASH_ADD, &p_image_flash, 4);
    if ((p_image_flash.ImageFlag != FLAG_USER_CALL_IAP))
    {
        jumpApp();
    }
#else
    //PB4上拉输入,作按键检测跳转用
    R32_PB_PD_DRV &= ~GPIO_Pin_4;
    R32_PB_PU |= GPIO_Pin_4;
    R32_PB_DIR &= ~GPIO_Pin_4;
    DelayMs(10);
    if (GPIOB_ReadPortPin(GPIO_Pin_4))
    {
        DelayMs(5);
        if (GPIOB_ReadPortPin(GPIO_Pin_4))
        {
            //启动前判断是否进入iap，没有按键按下
            jumpApp();
        }
    }
#endif

    /* USBHS初始化 */
    R8_USBHS_PLL_CTRL = USBHS_PLL_EN;
    R16_PIN_CONFIG |= RB_PIN_USB2_EN;

    R8_USB2_CTRL = USBHS_UD_RST_LINK | USBHS_UD_PHY_SUSPENDM;
    R8_USB2_INT_EN = USBHS_UDIE_BUS_RST | USBHS_UDIE_SUSPEND | USBHS_UDIE_BUS_SLEEP | USBHS_UDIE_LPM_ACT | USBHS_UDIE_TRANSFER | USBHS_UDIE_LINK_RDY;

    R16_U2EP_TX_EN = RB_EP0_EN |  RB_EP2_EN ;
    R16_U2EP_RX_EN = RB_EP0_EN |  RB_EP2_EN ;

    R32_U2EP0_MAX_LEN  = DEF_USBD_UEP0_SIZE;
    R32_U2EP2_MAX_LEN  = DEF_USB_EP2_HS_SIZE;

    R32_U2EP0_DMA    = (uint32_t)(uint8_t *)USBHS_EP0_Buf;
    R32_U2EP2_RX_DMA = (uint32_t)(uint8_t *)USBHS_EP2_Rx_Buf;
    R32_U2EP2_TX_DMA = (uint32_t)(uint8_t *)USBHS_EP2_Tx_Buf;

    R16_U2EP0_T_LEN  = 0;
    R8_U2EP0_TX_CTRL = USBHS_UEP_T_RES_NAK;
    R8_U2EP0_RX_CTRL = USBHS_UEP_R_RES_ACK;

    R16_U2EP2_T_LEN  = 0;
    R8_U2EP2_TX_CTRL = USBHS_UEP_T_RES_NAK;
    R8_U2EP2_RX_CTRL = USBHS_UEP_R_RES_ACK;

    /* Clear End-points Busy Status */
    for( i = 0; i < DEF_UEP_NUM; i++ )
    {
        USBHS_Endp_Busy[ i ] = 0;
    }

    R8_USB2_DEV_AD = 0x00;
    R8_USB2_BASE_MODE = USBHS_UD_SPEED_HIGH;
    R8_USB2_CTRL = USBHS_UD_DEV_EN | USBHS_UD_DMA_EN | USBHS_UD_LPM_EN | USBHS_UD_PHY_SUSPENDM;
    R16_PIN_CONFIG |= RB_PIN_USB2_EN;
    R8_USB2_INT_FG = 0xFF;                       // 清中断标志
    R8_USB2_INT_EN = 0;                          //禁止usb中断，采用查询方式

    /* 进入highcode主循环 */
    Main_Circulation();
}
