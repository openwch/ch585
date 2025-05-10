/********************************** (C) COPYRIGHT *******************************
 * File Name          : wch_nfca_picc_config.h
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2024/11/13
 * Description        : nfc picc bsp底层配置
 *********************************************************************************
 * Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#ifndef _WCH_NFCA_PICC_CONFIG_H_
#define _WCH_NFCA_PICC_CONFIG_H_

#define PICC_DATA_BUF_LEN           20          /* NFC PICC信号数据区和校验位区缓存区大小配置，按照模拟的卡片协议配置即可 */
#define PICC_SIGNAL_BUF_LEN         512         /* NFC PICC信号捕获数据缓存区大小配置，需要满足下面的条件判断，比条件更大会更稳定，延误进中断的时间可以延长 */

#if PICC_SIGNAL_BUF_LEN < (PICC_DATA_BUF_LEN * 18 + 12)
#error "PICC_SIGNAL_BUF_LEN must bigger than (PICC_DATA_BUF_LEN * 18 + 12)."
#endif

#define NFCA_PICC_RSP_POLAR         0           /* NFC PICC信号回复调制极性，一般无需更改 */

#endif  /* _WCH_NFCA_PICC_CONFIG_H_ */
