/********************************** (C) COPYRIGHT *******************************
 * File Name          : hidconsumer.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2020/08/06
 * Description        : 蓝牙音量控制应用程序，初始化广播连接参数，然后广播，直至连接主机后，定时上传音量键下键
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "CONFIG.h"
#include "devinfoservice.h"
#include "battservice.h"
#include "hiddev.h"
#include "hidconsumer.h"
#include "hidconsumerservice.h"
#include "nfc_btssp_t2t.h"
#include "glue.h"

/*********************************************************************
 * MACROS
 */
/* 地址类型是否继续配制成static类型 */
#define NFC_BTSSP_CFG_ADDR_STATIC            1

// HID consumer input report length
#define HID_CONSUMER_IN_RPT_LEN              2

/*********************************************************************
 * CONSTANTS
 */
// Param update delay
#define START_PARAM_UPDATE_EVT_DELAY         12800

// Param update delay
#define START_PHY_UPDATE_DELAY               1600

// HID idle timeout in msec; set to zero to disable timeout
#define DEFAULT_HID_IDLE_TIMEOUT             60000

// Minimum connection interval (units of 1.25ms)
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL    8

// Maximum connection interval (units of 1.25ms)
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL    8

// Slave latency to use if parameter update request
#define DEFAULT_DESIRED_SLAVE_LATENCY        0

// Supervision timeout value (units of 10ms)
#define DEFAULT_DESIRED_CONN_TIMEOUT         500

// Default passcode
#define DEFAULT_PASSCODE                     0

// Default GAP pairing mode
#define DEFAULT_PAIRING_MODE                 GAPBOND_PAIRING_MODE_WAIT_FOR_REQ

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                    TRUE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                 TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES              GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT    //GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT GAPBOND_IO_CAP_DISPLAY_ONLY

// Battery level is critical when it is less than this %
#define DEFAULT_BATT_CRITICAL_LEVEL          6

// Default sc protection, TRUE to enable.
#define DEFAULT_SC_PROTECTION                TRUE

#if DEFAULT_SC_PROTECTION
#if BLE_BUFF_MAX_LEN < 80
#error "DEFAULT_SC_PROTECTION need BLE_BUFF_MAX_LEN > 80."
#endif
#endif

/*********************************************************************
 * TYPEDEFS
 */
typedef struct _ble_p256_ecdh_data_struct
{
    uint8_t public_key[64];
    uint8_t private_key[64];
    uint8_t random[16];
    uint8_t confirm[16];
} ble_p256_ecdh_data_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */
ble_p256_ecdh_data_t ble_p256_ecdh_data;
uint8_t hid_bd_addr[6];

// Task ID
static uint8_t hidEmuTaskId = INVALID_TASK_ID;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// GAP Profile - Name attribute for SCAN RSP data
static uint8_t scanRspData[] = {
    0x0e, // length of this data
    GAP_ADTYPE_LOCAL_NAME_COMPLETE,
    'H',
    'I',
    'D',
    ' ',
    'B',
    'T',
    'S',
    'S',
    'P',
    ' ',
    'N',
    'F',
    'C',
    // connection interval range
    0x05, // length of this data
    GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
    LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL), // 100ms
    HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
    LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL), // 1s
    HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),

    // Tx power level
    0x02, // length of this data
    GAP_ADTYPE_POWER_LEVEL,
    0 // 0dBm
};

// Advertising data
static uint8_t advertData[] = {
    // flags
    0x02, // length of this data
    GAP_ADTYPE_FLAGS,
    GAP_ADTYPE_FLAGS_LIMITED | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

    // appearance
    0x03, // length of this data
    GAP_ADTYPE_APPEARANCE,
    LO_UINT16(GAP_APPEARE_GENERIC_HID),
    HI_UINT16(GAP_APPEARE_GENERIC_HID)
};

// Device name attribute value
static CONST uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "HID BTSSP NFC";

// HID Dev configuration
static hidDevCfg_t hidEmuCfg = {
    DEFAULT_HID_IDLE_TIMEOUT, // Idle timeout
    HID_FEATURE_FLAGS         // HID feature flags
};

static uint16_t hidEmuConnHandle = GAP_CONNHANDLE_INIT;

int tmp_sm_alg_gen_key_pair(uint8_t *pub, uint8_t *priv)
{
    tmos_memcpy(pub, ble_p256_ecdh_data.public_key, 64);
    tmos_memcpy(priv, ble_p256_ecdh_data.private_key, 32);

    return 0;
}

static gapEccCBs_t eccCB =
{
        .gen_key_pair = tmp_sm_alg_gen_key_pair,    //tmp_sm_alg_gen_key_pair ble_sm_alg_gen_key_pair
        .gen_dhkey = ble_sm_alg_gen_dhkey,
        .alg_f4 = ble_sm_alg_f4,
        .alg_g2 = ble_sm_alg_g2,
        .alg_f5 = ble_sm_alg_f5,
        .alg_f6 = ble_sm_alg_f6,
        .randkey = NULL,
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void    hidEmu_ProcessTMOSMsg(tmos_event_hdr_t *pMsg);
static void    hidEmuSendConsumerReport(uint8_t volume_up, uint8_t volume_dowm);
static uint8_t hidEmuRptCB(uint8_t id, uint8_t type, uint16_t uuid,
                           uint8_t oper, uint16_t *pLen, uint8_t *pData);
static void    hidEmuEvtCB(uint8_t evt);
static void    hidEmuStateCB(gapRole_States_t newState, gapRoleEvent_t *pEvent);

/*********************************************************************
 * PROFILE CALLBACKS
 */

static hidDevCB_t hidEmuHidCBs = {
    hidEmuRptCB,
    hidEmuEvtCB,
    NULL,
    hidEmuStateCB
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      HidEmu_Init
 *
 * @brief   Initialization function for the HidEmuKbd App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by TMOS.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void HidEmu_Init()
{
    hidEmuTaskId = TMOS_ProcessEventRegister(HidEmu_ProcessEvent);

    // Setup the GAP Peripheral Role Profile
    {
        uint8_t initial_advertising_enable = TRUE;

        // Set the GAP Role Parameters
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &initial_advertising_enable);

        GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);
        GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData), scanRspData);
    }

    // Set the GAP Characteristics
    GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, (void *)attDeviceName);

    // Setup the GAP Bond Manager
    {
        uint32_t passkey = DEFAULT_PASSCODE;
        uint8_t  pairMode = DEFAULT_PAIRING_MODE;
        uint8_t  mitm = DEFAULT_MITM_MODE;
        uint8_t  ioCap = DEFAULT_IO_CAPABILITIES;
        uint8_t  bonding = DEFAULT_BONDING_MODE;

        GAPBondMgr_SetParameter(GAPBOND_PERI_DEFAULT_PASSCODE, sizeof(uint32_t), &passkey);
        GAPBondMgr_SetParameter(GAPBOND_PERI_PAIRING_MODE, sizeof(uint8_t), &pairMode);
        GAPBondMgr_SetParameter(GAPBOND_PERI_MITM_PROTECTION, sizeof(uint8_t), &mitm);
        GAPBondMgr_SetParameter(GAPBOND_PERI_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
        GAPBondMgr_SetParameter(GAPBOND_PERI_BONDING_ENABLED, sizeof(uint8_t), &bonding);

#if DEFAULT_SC_PROTECTION

        ble_sm_alg_ecc_init();

        GAPBondMgr_EccInit(&eccCB); /* 必须先注册回调，才能设置GAPBOND_PERI_SC_PROTECTION为TRUE，否则会失败。 */

        uint8_t  sc = DEFAULT_SC_PROTECTION;
        GAPBondMgr_SetParameter(GAPBOND_PERI_SC_PROTECTION, sizeof(sc), &sc);

        ble_sm_alg_gen_key_pair(ble_p256_ecdh_data.public_key, ble_p256_ecdh_data.private_key);

        if(ble_sm_alg_rand(ble_p256_ecdh_data.random, 16) == 0)
        {
            for(uint8_t i = 0; i < 16; i++)
            {
                ble_p256_ecdh_data.random[i] = i;
            }
        }

        GAPBondMgr_SetParameter(GAPBOND_PERI_OOB_DATA, 16, ble_p256_ecdh_data.random);

        ble_sm_alg_f4(ble_p256_ecdh_data.public_key, ble_p256_ecdh_data.public_key, ble_p256_ecdh_data.random, 0, ble_p256_ecdh_data.confirm);
#endif

    }

    // Setup Battery Characteristic Values
    {
        uint8_t critical = DEFAULT_BATT_CRITICAL_LEVEL;
        Batt_SetParameter(BATT_PARAM_CRITICAL_LEVEL, sizeof(uint8_t), &critical);
    }

    // Set up HID keyboard service
    Hid_AddService();

    // Register for HID Dev callback
    HidDev_Register(&hidEmuCfg, &hidEmuHidCBs);

    // Setup a delayed profile startup
    tmos_set_event(hidEmuTaskId, START_DEVICE_EVT);

    GATT_InitClient( );
}

/*********************************************************************
 * @fn      HidEmu_ProcessEvent
 *
 * @brief   HidEmuKbd Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The TMOS assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16_t HidEmu_ProcessEvent(uint8_t task_id, uint16_t events)
{
    if(events & SYS_EVENT_MSG)
    {
        uint8_t *pMsg;

        if((pMsg = tmos_msg_receive(hidEmuTaskId)) != NULL)
        {
            hidEmu_ProcessTMOSMsg((tmos_event_hdr_t *)pMsg);

            // Release the TMOS message
            tmos_msg_deallocate(pMsg);
        }

        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }

    if(events & START_DEVICE_EVT)
    {
        return (events ^ START_DEVICE_EVT);
    }

    if(events & START_PARAM_UPDATE_EVT)
    {
        // Send connect param update request
        GAPRole_PeripheralConnParamUpdateReq(hidEmuConnHandle,
                                             DEFAULT_DESIRED_MIN_CONN_INTERVAL,
                                             DEFAULT_DESIRED_MAX_CONN_INTERVAL,
                                             DEFAULT_DESIRED_SLAVE_LATENCY,
                                             DEFAULT_DESIRED_CONN_TIMEOUT,
                                             hidEmuTaskId);
//        GAPBondMgr_PeriSecurityReq(hidEmuConnHandle);
        return (events ^ START_PARAM_UPDATE_EVT);
    }

    if(events & START_PHY_UPDATE_EVT)
    {
        // start phy update
        PRINT("Send Phy Update %x...\n", GAPRole_UpdatePHY(hidEmuConnHandle, 0,
                    GAP_PHY_BIT_LE_2M, GAP_PHY_BIT_LE_2M, 0));

        return (events ^ START_PHY_UPDATE_EVT);
    }

    if(events & START_REPORT_EVT)
    {
        //Send volume down and release
        //Report for PC
//        hidEmuSendConsumerReport(0xea, 0);
//        hidEmuSendConsumerReport(0, 0);

        //Report for Android
        hidEmuSendConsumerReport(0, 1);
        hidEmuSendConsumerReport(0, 0);

        tmos_start_task(hidEmuTaskId, START_REPORT_EVT, 1600);
        return (events ^ START_REPORT_EVT);
    }
    return 0;
}

/*********************************************************************
 * @fn      hidEmu_ProcessTMOSMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void hidEmu_ProcessTMOSMsg(tmos_event_hdr_t *pMsg)
{
    switch(pMsg->event)
    {
        default:
            break;
    }
}

/*********************************************************************
 * @fn      hidEmuSendConsumerReport
 *
 * @brief   Build and send a HID consumer report.
 *
 * @param   volume_up - consumer volume up
 *					volume_dowm - consumer volume dowm
 *
 * @return  none
 */
static void hidEmuSendConsumerReport(uint8_t volume_up, uint8_t volume_dowm)
{
    uint8_t buf[HID_CONSUMER_IN_RPT_LEN];

    buf[0] = volume_up;   // volume up
    buf[1] = volume_dowm; // volume dowm

    HidDev_Report(HID_RPT_ID_CONSUMER_IN, HID_REPORT_TYPE_INPUT,
                  HID_CONSUMER_IN_RPT_LEN, buf);
}

/*********************************************************************
 * @fn      hidEmuStateCB
 *
 * @brief   GAP state change callback.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void hidEmuStateCB(gapRole_States_t newState, gapRoleEvent_t *pEvent)
{
    switch(newState & GAPROLE_STATE_ADV_MASK)
    {
        case GAPROLE_STARTED:
        {
            const uint8_t other_adv_data[] = {
                    0x03, GAP_ADTYPE_APPEARANCE, 0xC1, 0x03, 0x02, GAP_ADTYPE_FLAGS, 0x04,
            };
            uint8_t uid[7] = {0x00, 0xae, 0x38, 0xe2, 0xb5, 0x4c, 0x80};    /* NFC 7字节卡号，第一个字节为厂商字节，0为未知厂商，自定义uid时需要注意不可侵权 */
            nfc_btssp_t2t_init_t cfg;
            NFC_BTSSP_T2T_INIT_ERR_t res;

            GAPRole_GetParameter(GAPROLE_BD_ADDR, hid_bd_addr);

            cfg.bd_addr = hid_bd_addr;

#if NFC_BTSSP_CFG_ADDR_STATIC       /* 是否将地址配置成静态地址 */
            hid_bd_addr[5] |= 0xc0;
            GAP_ConfigDeviceAddr(ADDRTYPE_STATIC, hid_bd_addr);
            cfg.bd_addr_type = ADDRTYPE_STATIC;
#else
            cfg.bd_addr_type = ADDRTYPE_PUBLIC;
#endif

            PRINT("Initialized..\n");
            PRINT("hid_bd_addr: %x %x %x %x %x %x\n",
                    hid_bd_addr[0], hid_bd_addr[1], hid_bd_addr[2],
                    hid_bd_addr[3], hid_bd_addr[4], hid_bd_addr[5]);

            cfg.t2t_uid = uid;
            cfg.le_role = 0;

            uint8_t invalid_buf[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
            cfg.sm_tk = invalid_buf;
            cfg.le_sc_confirm = ble_p256_ecdh_data.confirm;
            cfg.le_sc_random = ble_p256_ecdh_data.random;

            cfg.other_adv_data = (uint8_t *)other_adv_data;;
            cfg.other_adv_data_len = sizeof(other_adv_data);

            cfg.local_name_complete = (uint8_t *)attDeviceName;

            res = nfc_btssp_t2t_init(&cfg);
            if(res == NFC_BTSSP_T2T_INIT_OK)
            {
                PRINT("NFC BTSSP T2T INIT OK\n");
            }
            else
            {
                PRINT("NFC BTSSP T2T INIT ERR: %d\n", res);
            }
        }
        break;

        case GAPROLE_ADVERTISING:
            if(pEvent->gap.opcode == GAP_MAKE_DISCOVERABLE_DONE_EVENT)
            {
                PRINT("Advertising..\n");
            }
            break;

        case GAPROLE_CONNECTED:
            if(pEvent->gap.opcode == GAP_LINK_ESTABLISHED_EVENT)
            {
                gapEstLinkReqEvent_t *event = (gapEstLinkReqEvent_t *)pEvent;

                // get connection handle
                hidEmuConnHandle = event->connectionHandle;
                tmos_start_task(hidEmuTaskId, START_PARAM_UPDATE_EVT, START_PARAM_UPDATE_EVT_DELAY);

                {
                    attExchangeMTUReq_t Req;
                    Req.clientRxMTU = BLE_BUFF_MAX_LEN - 7;
                    GATT_ExchangeMTU(hidEmuConnHandle, &Req, hidEmuTaskId);
                }
                PRINT("Connected..\n");
            }
            break;

        case GAPROLE_CONNECTED_ADV:
            if(pEvent->gap.opcode == GAP_MAKE_DISCOVERABLE_DONE_EVENT)
            {
                PRINT("Connected Advertising..\n");
            }
            break;

        case GAPROLE_WAITING:
            if(pEvent->gap.opcode == GAP_END_DISCOVERABLE_DONE_EVENT)
            {
                PRINT("Waiting for advertising..\n");
            }
            else if(pEvent->gap.opcode == GAP_LINK_TERMINATED_EVENT)
            {
                PRINT("Disconnected.. Reason:%x\n", pEvent->linkTerminate.reason);
            }
            else if(pEvent->gap.opcode == GAP_LINK_ESTABLISHED_EVENT)
            {
                PRINT("Advertising timeout..\n");
            }
            // Enable advertising
            {
                uint8_t initial_advertising_enable = TRUE;
                // Set the GAP Role Parameters
                GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &initial_advertising_enable);
            }
            break;

        case GAPROLE_ERROR:
            PRINT("Error %x ..\n", pEvent->gap.opcode);
            break;

        default:
            break;
    }
}

/*********************************************************************
 * @fn      hidEmuRptCB
 *
 * @brief   HID Dev report callback.
 *
 * @param   id - HID report ID.
 * @param   type - HID report type.
 * @param   uuid - attribute uuid.
 * @param   oper - operation:  read, write, etc.
 * @param   len - Length of report.
 * @param   pData - Report data.
 *
 * @return  GATT status code.
 */
static uint8_t hidEmuRptCB(uint8_t id, uint8_t type, uint16_t uuid,
                           uint8_t oper, uint16_t *pLen, uint8_t *pData)
{
    uint8_t status = SUCCESS;

    // write
    if(oper == HID_DEV_OPER_WRITE)
    {
        status = Hid_SetParameter(id, type, uuid, *pLen, pData);
    }
    // read
    else if(oper == HID_DEV_OPER_READ)
    {
        status = Hid_GetParameter(id, type, uuid, pLen, pData);
    }
    // notifications enabled
    else if(oper == HID_DEV_OPER_ENABLE)
    {
        tmos_start_task(hidEmuTaskId, START_REPORT_EVT, 500);
    }
    return status;
}

/*********************************************************************
 * @fn      hidEmuEvtCB
 *
 * @brief   HID Dev event callback.
 *
 * @param   evt - event ID.
 *
 * @return  HID response code.
 */
static void hidEmuEvtCB(uint8_t evt)
{
    // process enter/exit suspend or enter/exit boot mode
    return;
}

/*********************************************************************
*********************************************************************/
