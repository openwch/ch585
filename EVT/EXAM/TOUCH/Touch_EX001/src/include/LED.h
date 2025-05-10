/********************************** (C) COPYRIGHT *******************************
 * File Name          : LED.h
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2016/04/12
 * Description        :
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

/******************************************************************************/
#ifndef __LED_H
#define __LED_H

#ifdef __cplusplus
extern "C" {
#endif
#include "CH58x_common.h"
#include "TouchKey_CFG.h"
/*********************************************************************
 * CONSTANTS
 */

 
#define LED_SEG1_PIN	GPIO_Pin_13	//GPIOB
#define LED_SEG2_PIN	GPIO_Pin_17	//GPIOB
#define LED_SEG3_PIN	GPIO_Pin_19	//GPIOB
#define LED_SEG4_PIN	GPIO_Pin_21	//GPIOB

#define LED_SEG1_PORT	GPIOB
#define LED_SEG2_PORT	GPIOB
#define LED_SEG3_PORT	GPIOB
#define LED_SEG4_PORT	GPIOB	

#define LED_COM1_PIN	GPIO_Pin_20	//GPIOB
#define LED_COM2_PIN	GPIO_Pin_18	//GPIOB
#define LED_COM3_PIN	GPIO_Pin_16	//GPIOB
#define LED_COM4_PIN	GPIO_Pin_12	//GPIOB
#define LED_COM5_PIN	GPIO_Pin_8	//GPIOB

#define LED_COM1_PORT	GPIOB
#define LED_COM2_PORT	GPIOB
#define LED_COM3_PORT	GPIOB
#define LED_COM4_PORT	GPIOB
#define LED_COM5_PORT	GPIOB


#define LED_SEG1_ON()	(R32_PB_SET = LED_SEG1_PIN)                      	// 设置时钟线为高电平
#define LED_SEG1_OFF()	(R32_PB_CLR  = LED_SEG1_PIN)                        // 设置时钟线为低电平
#define LED_SEG2_ON()	(R32_PB_SET = LED_SEG2_PIN)                      	// 设置时钟线为高电平
#define LED_SEG2_OFF()	(R32_PB_CLR  = LED_SEG2_PIN)                        // 设置时钟线为低电平
#define LED_SEG3_ON()	(R32_PB_SET = LED_SEG3_PIN)                      	// 设置时钟线为高电平
#define LED_SEG3_OFF()	(R32_PB_CLR  = LED_SEG3_PIN)                        // 设置时钟线为低电平
#define LED_SEG4_ON()	(R32_PB_SET = LED_SEG4_PIN)                      	// 设置时钟线为高电平
#define LED_SEG4_OFF()	(R32_PB_CLR  = LED_SEG4_PIN)                        // 设置时钟线为低电平

#define LED_COM1_OFF()	(R32_PB_SET = LED_COM1_PIN)                       	// 设置时钟线为高电平
#define LED_COM1_ON()	(R32_PB_CLR  = LED_COM1_PIN)                       	// 设置时钟线为低电平
#define LED_COM2_OFF()	(R32_PB_SET = LED_COM2_PIN)                       	// 设置时钟线为高电平
#define LED_COM2_ON()	(R32_PB_CLR  = LED_COM2_PIN)                      	// 设置时钟线为低电平
#define LED_COM3_OFF()	(R32_PB_SET = LED_COM3_PIN)                       	// 设置时钟线为高电平
#define LED_COM3_ON()	(R32_PB_CLR  = LED_COM3_PIN)                      	// 设置时钟线为低电平
#define LED_COM4_OFF()	(R32_PB_SET = LED_COM4_PIN)                       	// 设置时钟线为高电平
#define LED_COM4_ON()	(R32_PB_CLR  = LED_COM4_PIN)                      	// 设置时钟线为低电平
#define LED_COM5_OFF()	(R32_PB_SET = LED_COM5_PIN)                       	// 设置时钟线为高电平
#define LED_COM5_ON()	(R32_PB_CLR  = LED_COM5_PIN)                      	// 设置时钟线为低电平


typedef enum
{
	Normal,
	Breathing,
} BacklightWorkTypeDef;

typedef struct
{
    uint16_t TKY_BacklightOnTime;
    uint8_t BacklightStates;
    uint8_t RFU;
} TKY_Backlight_S;
/*********************************************************************
*********************************************************************/
// 初始化背光和蜂鸣器
extern void TKY_BacklightInit(void);
extern void TKY_BacklightTaskStart();
extern void TKY_BacklightTaskStop();
extern void TKY_BacklightOn (void);
extern void TKY_BacklightOff (void);
extern void TKY_KeyBacklightOut(uint8_t key, FunctionalState s);
extern void TKY_BacklightProcess (void);
extern uint8_t getBacklightState(uint8_t key);

#ifdef __cplusplus
}
#endif

#endif
