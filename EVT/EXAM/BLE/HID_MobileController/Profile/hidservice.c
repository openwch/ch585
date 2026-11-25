/********************************** (C) COPYRIGHT *******************************
 * File Name          : hidservice.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2022/04/13
 * Description        : 蓝牙遥控器
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "CONFIG.h"
#include "hidservice.h"
#include "hiddev.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// HID service
const uint8_t hidServUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(HID_SERV_UUID), HI_UINT16(HID_SERV_UUID)
};

// HID Information characteristic
const uint8_t hidInfoUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(HID_INFORMATION_UUID), HI_UINT16(HID_INFORMATION_UUID)
};

// HID Report Map characteristic
const uint8_t hidReportMapUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(REPORT_MAP_UUID), HI_UINT16(REPORT_MAP_UUID)
};

// HID Control Point characteristic
const uint8_t hidControlPointUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(HID_CTRL_PT_UUID), HI_UINT16(HID_CTRL_PT_UUID)
};

// HID Report characteristic
const uint8_t hidReportUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(REPORT_UUID), HI_UINT16(REPORT_UUID)
};

// HID Protocol Mode characteristic
const uint8_t hidProtocolModeUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(PROTOCOL_MODE_UUID), HI_UINT16(PROTOCOL_MODE_UUID)
};

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// HID Information characteristic value
static const uint8_t hidInfo[HID_INFORMATION_LEN] =
{
    LO_UINT16(0x0111), HI_UINT16(0x0111), // bcdHID (USB HID version)
    0x00,                                 // bCountryCode
    HID_FEATURE_FLAGS                     // Flags
};

static const uint8_t aHidReportMap[] =
{
    0x05, 0x0D,        // Usage Page (Digitizer)
    0x09, 0x01,        // Usage (Digitizer)
    0xA1, 0x01,        // Collection (Application)

    0x85, 0x01,        //   Report ID (1)

    0x09, 0x22,        //   Usage (Finger)
    0xA1, 0x02,        //   Collection (Logical)
    0x09, 0x42,        //     Usage (Tip Switch)
    0x15, 0x00,        //     Logical Minimum (0)
    0x25, 0x01,        //     Logical Maximum (1)
    0x75, 0x01,        //     Report Size (1)
    0x95, 0x01,        //     Report Count (1)
    0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x09, 0x32,        //     Usage (In Range)
//    0x75, 0x07,
    0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x95, 0x06,        //     Report Count (6)
    0x81, 0x03,        //     Input (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x05, 0x01,        //     Usage Page (Generic Desktop Ctrls)
    0x26, 0xE8, 0x03,  //     Logical Maximum (1000)
    0x75, 0x10,        //     Report Size (16)
    0x95, 0x01,        //     Report Count (1)
    0x55, 0x00,        //     Unit Exponent (0)
    0x65, 0x00,        //     Unit (None)
    0x09, 0x30,        //     Usage (X)
    0x35, 0x00,        //     Physical Minimum (0)
    0x46, 0xE8, 0x03,  //     Physical Maximum (1000)
    0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x09, 0x31,        //     Usage (Y)
    0x46, 0xE8, 0x03,  //     Physical Maximum (1000)
    0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0xC0,              //   End Collection

    0x05, 0x0D,        //   Usage Page (Digitizer)
    0x09, 0x48,        //   Usage (0x48)
    0x09, 0x49,        //   Usage (0x49)
    0x95, 0x02,        //   Report Count (2)
    0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0xC0,              // End Collection

    /* consumer报表 */
    0x05, 0x0c, // USAGE_PAGE (Consumer Devices)
    0x09, 0x01, // USAGE (Consumer Control)
    0xa1, 0x01, // COLLECTION (Application)
    0x85, 0x02, //   REPORT_ID (2)

    0x15, 0x00, //   LOGICAL_MINIMUM (0)
    0x25, 0x01, //   LOGICAL_MAXIMUM (1)
    0x15, 0x00, //   LOGICAL_MINIMUM (0)
    0x25, 0x01, //   LOGICAL_MAXIMUM (1)
    0x75, 0x08, //   REPORT_SIZE (8)
    0x95, 0x01, //   REPORT_COUNT (1)
    0x09, 0xe9, //   USAGE (Volume Up)
    0x81, 0x06, //   INPUT (Data,Var,Rel)
    0x09, 0xea, //   USAGE (Volume Down)
    0x81, 0x06, //   INPUT (Data,Var,Rel)

    0xc0, // END_COLLECTION

    0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
    0x09, 0x06,        // Usage (Keyboard)
    0xA1, 0x01,        // Collection (Application)

    0x85, 0x03,        //   Report ID (3)

    0x05, 0x07,        //   Usage Page (Kbrd/Keypad)
    0x19, 0xE0,        //   Usage Minimum (0xE0)
    0x29, 0xE7,        //   Usage Maximum (0xE7)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x08,        //   Report Count (8)
    0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x01,        //   Report Count (1)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0xF4,        //   Logical Maximum (-12)
    0x05, 0x07,        //   Usage Page (Kbrd/Keypad)
    0x19, 0x00,        //   Usage Minimum (0x00)
    0x29, 0xF4,        //   Usage Maximum (0xF4)
    0x81, 0x00,        //   Input (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0xC0,              // End Collection
};


// HID Report Map characteristic value（仅安卓报表映射，Hid_AddService中装载）
uint8_t hidReportMap[sizeof(aHidReportMap)];

// HID report map length
uint16_t hidReportMapLen = sizeof(aHidReportMap);

// HID report mapping table
static hidRptMap_t hidRptMap[HID_NUM_REPORTS];

/*********************************************************************
 * Profile Attributes - variables
 */

// HID Service attribute
static const gattAttrType_t hidService = {ATT_BT_UUID_SIZE, hidServUUID};

// Include attribute (Battery service)
static uint16_t include = GATT_INVALID_HANDLE;

// HID Information characteristic
static uint8_t hidInfoProps = GATT_PROP_READ;

// HID Report Map characteristic
static uint8_t hidReportMapProps = GATT_PROP_READ;

// HID Control Point characteristic
static uint8_t hidControlPointProps = GATT_PROP_WRITE_NO_RSP;
static uint8_t hidControlPoint;

// HID Protocol Mode characteristic
static uint8_t hidProtocolModeProps = GATT_PROP_READ | GATT_PROP_WRITE_NO_RSP;
uint8_t        hidProtocolMode = HID_PROTOCOL_MODE_REPORT;

// HID Report characteristic, ADIG_IMOU input
static uint8_t       hidReportADIG_IMOUProps = GATT_PROP_READ | GATT_PROP_NOTIFY;
static uint8_t       hidReportADIG_IMOU;
static gattCharCfg_t hidReportADIG_IMOUClientCharCfg[GATT_MAX_NUM_CONN];

// HID Report Reference characteristic descriptor, ADIG_IMOU input
static uint8_t hidReportRefADIG_IMOU[HID_REPORT_REF_LEN] =
{HID_RPT_ID_ADIG_IMOU, HID_REPORT_TYPE_INPUT};


// HID Report characteristic, ACSU_ICSU input
static uint8_t hidReportACSU_ICSUProps = GATT_PROP_READ | GATT_PROP_NOTIFY;
static uint8_t hidReportACSU_ICSU;
static gattCharCfg_t hidReportACSU_ICSUClientCharCfg[GATT_MAX_NUM_CONN];
// HID Report Reference characteristic descriptor, ACSU_ICSU input
static uint8_t hidReportRefACSU_ICSU[HID_REPORT_REF_LEN] =
{ HID_RPT_ID_ACSU_ICSU, HID_REPORT_TYPE_INPUT };


// HID Report characteristic, AKBD_IPTR input
static uint8_t hidReportAKBD_IPTRProps = GATT_PROP_READ | GATT_PROP_NOTIFY;
static uint8_t hidReportAKBD_IPTR;
static gattCharCfg_t hidReportAKBD_IPTRClientCharCfg[GATT_MAX_NUM_CONN];

// HID Report Reference characteristic descriptor, AKBD_IPTR input
static uint8_t hidReportRefAKBD_IPTR[HID_REPORT_REF_LEN] =
{ HID_RPT_ID_AKBD_IPTR, HID_REPORT_TYPE_INPUT };

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t hidAttrTbl[] =
{

#if 1
    // HID Service
    {
        {ATT_BT_UUID_SIZE, primaryServiceUUID}, /* type */
        GATT_PERMIT_READ,                       /* permissions */
        0,                                      /* handle */
        (uint8_t *) &hidService                 /* pValue */
    },
#endif

#if 1
    // HID Information characteristic declaration
    {
        {ATT_BT_UUID_SIZE, characterUUID},
        GATT_PERMIT_READ,
        0,
        &hidInfoProps
    },

    // HID Information characteristic
    {
        {ATT_BT_UUID_SIZE, hidInfoUUID},
        GATT_PERMIT_ENCRYPT_READ,
        0,
        (uint8_t *)hidInfo
    },
#endif

#if 1
    // HID Control Point characteristic declaration
    {
        {ATT_BT_UUID_SIZE, characterUUID},
        GATT_PERMIT_READ,
        0,
        &hidControlPointProps
    },

    // HID Control Point characteristic
    {
        {ATT_BT_UUID_SIZE, hidControlPointUUID},
        GATT_PERMIT_ENCRYPT_WRITE,
        0,
        &hidControlPoint
    },
#endif

#if 1
    // HID Protocol Mode characteristic declaration
    {
        {ATT_BT_UUID_SIZE, characterUUID},
        GATT_PERMIT_READ,
        0,
        &hidProtocolModeProps
    },

    // HID Protocol Mode characteristic
    {
        {ATT_BT_UUID_SIZE, hidProtocolModeUUID},
        GATT_PERMIT_ENCRYPT_READ | GATT_PERMIT_ENCRYPT_WRITE,
        0,
        &hidProtocolMode
    },
#endif

#if 1
    // HID Report Map characteristic declaration
    {
        {ATT_BT_UUID_SIZE, characterUUID},
        GATT_PERMIT_READ,
        0,
        &hidReportMapProps
    },

    // HID Report Map characteristic
    {
        {ATT_BT_UUID_SIZE, hidReportMapUUID},
        GATT_PERMIT_ENCRYPT_READ,
        0,
        (uint8_t *)hidReportMap
    },
#endif

#if 1
    // HID Report characteristic, ADIG_IMOU input declaration
    {
        {ATT_BT_UUID_SIZE, characterUUID},
        GATT_PERMIT_READ,
        0,
        &hidReportADIG_IMOUProps
    },

    // HID Report characteristic, ADIG_IMOU input
    {
        {ATT_BT_UUID_SIZE, hidReportUUID},
        GATT_PERMIT_ENCRYPT_READ,
        0,
        &hidReportADIG_IMOU
    },

    // HID Report characteristic client characteristic configuration
    {
        {ATT_BT_UUID_SIZE, clientCharCfgUUID},
        GATT_PERMIT_READ | GATT_PERMIT_ENCRYPT_WRITE,
        0,
        (uint8_t *) &hidReportADIG_IMOUClientCharCfg
    },

    // HID Report Reference characteristic descriptor, ADIG_IMOU input
    {
        {ATT_BT_UUID_SIZE, reportRefUUID},
        GATT_PERMIT_READ,
        0,
        hidReportRefADIG_IMOU
    },
#endif

#if 1
    // HID Report characteristic, ACSU_ICSU input declaration
    {
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &hidReportACSU_ICSUProps
    },

    // HID Report characteristic, ACSU_ICSU input
    {
        { ATT_BT_UUID_SIZE, hidReportUUID },
        GATT_PERMIT_ENCRYPT_READ,
        0,
        &hidReportACSU_ICSU
    },

    // HID Report characteristic client characteristic configuration
    {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_ENCRYPT_WRITE,
        0,
        (uint8_t *) &hidReportACSU_ICSUClientCharCfg
    },

    // HID Report Reference characteristic descriptor, ACSU_ICSU input
    {
        { ATT_BT_UUID_SIZE, reportRefUUID },
        GATT_PERMIT_READ,
        0,
        hidReportRefACSU_ICSU
    },
#endif

#if 1
    // HID Report characteristic, AKBD_IPTR input declaration
    {
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &hidReportAKBD_IPTRProps
    },

    // HID Report characteristic, AKBD_IPTR input
    {
        { ATT_BT_UUID_SIZE, hidReportUUID },
        GATT_PERMIT_ENCRYPT_READ,
        0,
        &hidReportAKBD_IPTR
    },

    // HID Report characteristic client characteristic configuration
    {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_ENCRYPT_WRITE,
        0,
        (uint8_t *) &hidReportAKBD_IPTRClientCharCfg
    },

    // HID Report Reference characteristic descriptor, AKBD_IPTR input
    {
        { ATT_BT_UUID_SIZE, reportRefUUID },
        GATT_PERMIT_READ,
        0,
        hidReportRefAKBD_IPTR
    },
#endif

};

// Attribute index enumeration-- these indexes match array elements above
enum
{
#if 1
    HID_SERVICE_IDX,                 // HID Service
    HID_INFO_DECL_IDX,               // HID Information characteristic declaration
    HID_INFO_IDX,                    // HID Information characteristic
#endif

#if 1
    HID_CONTROL_POINT_DECL_IDX,      // HID Control Point characteristic declaration
    HID_CONTROL_POINT_IDX,           // HID Control Point characteristic
#endif

#if 1
    HID_PROTOCOL_MODE_DECL_IDX,      // HID Protocol Mode characteristic declaration
    HID_PROTOCOL_MODE_IDX,           // HID Protocol Mode characteristic
#endif

#if 1
    HID_REPORT_MAP_DECL_IDX,         // HID Report Map characteristic declaration
    HID_REPORT_MAP_IDX,              // HID Report Map characteristic
#endif

#if 1
    HID_REPORT_ADIG_IMOU_DECL_IDX, // HID Report characteristic, ADIG_IMOU input declaration
    HID_REPORT_ADIG_IMOU_IDX,      // HID Report characteristic, ADIG_IMOU input
    HID_REPORT_ADIG_IMOU_CCCD_IDX, // HID Report characteristic client characteristic configuration
    HID_REPORT_REF_ADIG_IMOU_IDX,  // HID Report Reference characteristic descriptor, ADIG_IMOU input
#endif

#if 1
    HID_REPORT_ACSU_ICSU_DECL_IDX,     // HID Report characteristic, ACSU_ICSU input declaration
    HID_REPORT_ACSU_ICSU_IDX,          // HID Report characteristic, ACSU_ICSU input
    HID_REPORT_ACSU_ICSU_CCCD_IDX,     // HID Report characteristic client characteristic configuration
    HID_REPORT_REF_ACSU_ICSU_IDX,      // HID Report Reference characteristic descriptor, ACSU_ICSU input
#endif

#if 1
    HID_REPORT_AKBD_IPTR_DECL_IDX, // HID Report characteristic, AKBD_IPTR input declaration
    HID_REPORT_AKBD_IPTR_IDX,      // HID Report characteristic, AKBD_IPTR input
    HID_REPORT_AKBD_IPTR_CCCD_IDX, // HID Report characteristic client characteristic configuration
    HID_REPORT_REF_AKBD_IPTR_IDX,  // HID Report Reference characteristic descriptor, AKBD_IPTR input
#endif

};

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * PROFILE CALLBACKS
 */

// Service Callbacks
gattServiceCBs_t hidCBs =
{
    HidDev_ReadAttrCB,  // Read callback function pointer
    HidDev_WriteAttrCB, // Write callback function pointer
    NULL                // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Hid_AddService
 *
 * @brief   Initializes the HID Service by registering
 *          GATT attributes with the GATT server.
 *
 * @return  Success or Failure
 */
bStatus_t Hid_AddService(void)
{
    uint8_t status = SUCCESS;

    // 装载HID报表映射（仅安卓：触摸屏+Consumer+键盘）
    tmos_memcpy(hidReportMap, aHidReportMap, sizeof(aHidReportMap));

    // Initialize Client Characteristic Configuration attributes
    GATTServApp_InitCharCfg(INVALID_CONNHANDLE, hidReportAKBD_IPTRClientCharCfg);

    GATTServApp_InitCharCfg(INVALID_CONNHANDLE, hidReportADIG_IMOUClientCharCfg);

    GATTServApp_InitCharCfg(INVALID_CONNHANDLE, hidReportACSU_ICSUClientCharCfg);

    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService(hidAttrTbl, GATT_NUM_ATTRS(hidAttrTbl), GATT_MAX_ENCRYPT_KEY_SIZE, &hidCBs);

    // Construct map of reports to characteristic handles
    // Each report is uniquely identified via its ID and type

    uint8_t cnt = 0;


    // ADIG_IMOU input report
    hidRptMap[cnt].id = hidReportRefADIG_IMOU[0];
    hidRptMap[cnt].type = hidReportRefADIG_IMOU[1];
    hidRptMap[cnt].handle = hidAttrTbl[HID_REPORT_ADIG_IMOU_IDX].handle;
    hidRptMap[cnt].cccdHandle = hidAttrTbl[HID_REPORT_ADIG_IMOU_CCCD_IDX].handle;
    hidRptMap[cnt].mode = HID_PROTOCOL_MODE_REPORT;
    cnt++;

    // ACSU_ICSU input report
    hidRptMap[cnt].id = hidReportRefACSU_ICSU[0];
    hidRptMap[cnt].type = hidReportRefACSU_ICSU[1];
    hidRptMap[cnt].handle = hidAttrTbl[HID_REPORT_ACSU_ICSU_IDX].handle;
    hidRptMap[cnt].cccdHandle = hidAttrTbl[HID_REPORT_ACSU_ICSU_CCCD_IDX].handle;
    hidRptMap[cnt].mode = HID_PROTOCOL_MODE_REPORT;
    cnt++;

    // AKBD_IPTR input report
    hidRptMap[cnt].id = hidReportRefAKBD_IPTR[0];
    hidRptMap[cnt].type = hidReportRefAKBD_IPTR[1];
    hidRptMap[cnt].handle = hidAttrTbl[HID_REPORT_AKBD_IPTR_IDX].handle;
    hidRptMap[cnt].cccdHandle = hidAttrTbl[HID_REPORT_AKBD_IPTR_CCCD_IDX].handle;
    hidRptMap[cnt].mode = HID_PROTOCOL_MODE_REPORT;
    cnt++;

    // Setup report ID map
    HidDev_RegisterReports(HID_NUM_REPORTS, hidRptMap);

    return (status);
}

/*********************************************************************
 * @fn      Hid_SetParameter
 *
 * @brief   Set a HID Kbd parameter.
 *
 * @param   id - HID report ID.
 * @param   type - HID report type.
 * @param   uuid - attribute uuid.
 * @param   len - length of data to right.
 * @param   pValue - pointer to data to write.  This is dependent on
 *          the input parameters and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 *
 * @return  GATT status code.
 */
uint8_t Hid_SetParameter(uint8_t id, uint8_t type, uint16_t uuid, uint8_t len, void *pValue)
{
    bStatus_t ret = SUCCESS;

    switch (uuid)
    {
    case REPORT_UUID:
        if (type == HID_REPORT_TYPE_OUTPUT)
        {
            if (len == 1)
            {

            }
            else
            {
                ret = ATT_ERR_INVALID_VALUE_SIZE;
            }
        }
        else if (type == HID_REPORT_TYPE_FEATURE)
        {
            if (len == 1)
            {
            }
            else
            {
                ret = ATT_ERR_INVALID_VALUE_SIZE;
            }
        }
        else
        {
            ret = ATT_ERR_ATTR_NOT_FOUND;
        }
        break;

    case BOOT_KEY_OUTPUT_UUID:
        if (len == 1)
        {
        }
        else
        {
            ret = ATT_ERR_INVALID_VALUE_SIZE;
        }
        break;

    default:
        // ignore the request
        break;
    }

    return (ret);
}

/*********************************************************************
 * @fn      Hid_GetParameter
 *
 * @brief   Get a HID Kbd parameter.
 *
 * @param   id - HID report ID.
 * @param   type - HID report type.
 * @param   uuid - attribute uuid.
 * @param   pLen - length of data to be read
 * @param   pValue - pointer to data to get.  This is dependent on
 *          the input parameters and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 *
 * @return  GATT status code.
 */
uint8_t Hid_GetParameter(uint8_t id, uint8_t type, uint16_t uuid, uint16_t *pLen, void *pValue)
{
    switch (uuid)
    {
    case REPORT_UUID:
        if (type == HID_REPORT_TYPE_OUTPUT)
        {
            *pLen = 1;
        }
        else
        {
            *pLen = 0;
        }
        break;

    case BOOT_KEY_OUTPUT_UUID:
        *pLen = 1;
        break;

    default:
        *pLen = 0;
        break;
    }

    return (SUCCESS);
}

/*********************************************************************
*********************************************************************/

