/********************************** (C) COPYRIGHT *******************************
 * File Name          : hidservice.h
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2022/04/13
 * Description        : À¶ÑÀÒ£¿ØÆ÷
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/
#ifndef HIDSERVICE_H
#define HIDSERVICE_H


#ifdef __cplusplus
extern "C" {
#endif

/*********************************************************************
 * INCLUDES
 */
#include "CONFIG.h"

/*********************************************************************
 * CONSTANTS
 */

// Number of HID reports defined in the service
#define HID_NUM_REPORTS           	3

// HID Report IDs for the service
#define HID_RPT_ID_ADIG_IMOU        1                       //´¥ÃþÆÁ(Digitizer)
#define HID_RPT_ID_ACSU_ICSU        2                       //Consumer£¨ÒôÁ¿£©
#define HID_RPT_ID_AKBD_IPTR        3                       //¼üÅÌ


// HID feature flags
#define HID_FEATURE_FLAGS         	HID_FLAGS_REMOTE_WAKE

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */

/*********************************************************************
 * API FUNCTIONS
 */

/*********************************************************************
 * @fn      Hid_AddService
 *
 * @brief   Initializes the HID service for keyboard by registering
 *          GATT attributes with the GATT server.
 *
 * @param   none
 *
 * @return  Success or Failure
 */
extern bStatus_t Hid_AddService(void);

/*********************************************************************
 * @fn      Hid_SetParameter
 *
 * @brief   Set a HID parameter.
 *
 * @param   id     - HID report ID.
 * @param   type   - HID report type.
 * @param   uuid   - attribute uuid.
 * @param   len    - length of data to right.
 * @param   pValue - pointer to data to write.  This is dependent on
 *          the input parameters and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 *
 * @return  GATT status code.
 */
extern uint8_t Hid_SetParameter(uint8_t id, uint8_t type, uint16_t uuid, uint8_t len,
                                void *pValue);

/*********************************************************************
 * @fn      Hid_GetParameter
 *
 * @brief   Get a HID parameter.
 *
 * @param   id     - HID report ID.
 * @param   type   - HID report type.
 * @param   uuid   - attribute uuid.
 * @param   pLen   - length of data to be read.
 * @param   pValue - pointer to data to get.  This is dependent on
 *          the input parameters and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 *
 * @return  GATT status code.
 */
extern uint8_t Hid_GetParameter(uint8_t id, uint8_t type, uint16_t uuid, uint16_t *pLen, void *pValue);

/*********************************************************************
*********************************************************************/


#endif /* HIDSERVICE_H */
