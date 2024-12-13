/********************************** (C) COPYRIGHT *******************************
 * File Name          : hidkbd.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2018/12/10
 * Description        : Bluetooth keyboard application. Initialize broadcast connection parameters, then broadcast.
 *                      After connecting to the host, upload key values regularly.
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
#include "hidkbdservice.h"
#include "hiddev.h"
#include "hidkbd.h"
#include "Touch.h"

/*********************************************************************
 * MACROS
 */
// HID keyboard input report length
#define HID_KEYBOARD_IN_RPT_LEN              8

// HID LED output report length
#define HID_LED_OUT_RPT_LEN                  1

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
#define DEFAULT_MITM_MODE                    FALSE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                 TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES              GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT

// Battery level is critical when it is less than this %
#define DEFAULT_BATT_CRITICAL_LEVEL          6

#define DEFAULT_SPECIAL_KEY_TIMEOUT          960

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

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
    0x0D,                           // length of this data
    GAP_ADTYPE_LOCAL_NAME_COMPLETE, // AD Type = Complete local name
    'H',
    'I',
    'D',
    ' ',
    'K',
    'e',
    'y',
    'b',
    'r',
    'o',
    'a',
    'd',  // connection interval range
    0x05, // length of this data
    GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
    LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL), // 100ms
    HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
    LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL), // 1s
    HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),

    // service UUIDs
    0x05, // length of this data
    GAP_ADTYPE_16BIT_MORE,
    LO_UINT16(HID_SERV_UUID),
    HI_UINT16(HID_SERV_UUID),
    LO_UINT16(BATT_SERV_UUID),
    HI_UINT16(BATT_SERV_UUID),

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
    LO_UINT16(GAP_APPEARE_HID_KEYBOARD),
    HI_UINT16(GAP_APPEARE_HID_KEYBOARD)};

// Device name attribute value
static CONST uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "HID Keyboard";

// HID Dev configuration
static hidDevCfg_t hidEmuCfg = {
    DEFAULT_HID_IDLE_TIMEOUT, // Idle timeout
    HID_FEATURE_FLAGS         // HID feature flags
};

static uint16_t hidEmuConnHandle = GAP_CONNHANDLE_INIT;

static uint8_t modifier_key_flag = 0;

static uint8_t fn_key_flag = 0;

static uint8_t key_buf[HID_KEYBOARD_IN_RPT_LEN] = {0};

static CONST uint8_t keyboard_val_table[48] = {
     // ESC   q     w     e     r     t     y     u     i     o     p     Backspace
        0x29, 0x14, 0x1A, 0x08, 0x15, 0x17, 0x1C, 0x18, 0x0C, 0x12, 0x13, 0x2A,
     // Tab   a     s     d     f     g     h     j     k     l     ;     '
        0x2B, 0x04, 0x16, 0x07, 0x09, 0x0A, 0x0B, 0x0D, 0x0E, 0x0F, 0x33, 0x34,
     // Shift z     x     c     v     b     n     m     ,     .     Up    Enter
        0x02, 0x1D, 0x1B, 0x06, 0x19, 0x05, 0x11, 0x10, 0x36, 0x37, 0x52, 0x28,
     // Ctrl  Win   Alt   Caps  Space Space Space Fn    /     Left  Down  Right
        0x01, 0x08, 0x04, 0x39, 0x2C, 0x2C, 0x2C, 0xFE, 0x38, 0x50, 0x51, 0x4F
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void    hidEmu_ProcessTMOSMsg(tmos_event_hdr_t *pMsg);
static uint8_t hidEmuRcvReport(uint8_t len, uint8_t *pData);
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
    hidEmuStateCB};

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
    static uint8_t modi_key = 0, fn_key = 0, keycode = 0;

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

        return (events ^ START_PARAM_UPDATE_EVT);
    }

    if(events & START_PHY_UPDATE_EVT)
    {
        // start phy update
        PRINT("Send Phy Update %x...\n", GAPRole_UpdatePHY(hidEmuConnHandle, 0, 
                    GAP_PHY_BIT_LE_2M, GAP_PHY_BIT_LE_2M, 0));

        return (events ^ START_PHY_UPDATE_EVT);
    }

    if(events & START_SPECIAL_KEY_TIMEOUT_EVT) //Modifier Keys timeout report
    {
        PRINT("special key timeout report\n");
        if(fn_key_flag)
        {
            // Add Fn key function
        }
//        PRINT("special press:");
//        for(uint8_t i=0; i<8; ++i)
//        {
//            PRINT("%x ", key_buf[i]);
//        }
//        PRINT("\n");
        HidDev_Report(HID_RPT_ID_KEY_IN, HID_REPORT_TYPE_INPUT, //Press Report
                      HID_KEYBOARD_IN_RPT_LEN, key_buf);

        modifier_key_flag = 0;
        fn_key_flag = 0;
        key_buf[0] = 0;
        key_buf[1] = 0;
//        PRINT("special release:");
//        for(uint8_t i=0; i<8; ++i)
//        {
//            PRINT("%x ", key_buf[i]);
//        }
//        PRINT("\n");
        HidDev_Report(HID_RPT_ID_KEY_IN, HID_REPORT_TYPE_INPUT, //Release Report
                      HID_KEYBOARD_IN_RPT_LEN, key_buf);

        return (events ^ START_SPECIAL_KEY_TIMEOUT_EVT);
    }

    if(events & START_REPORT_EVT)
    {
        uint8_t key_val = 0;
        static uint16_t print_time = 0;

        touch_KeyScan();
#if PRINT_EN
        print_time++;
        if(print_time == 500)
        {
            print_time = 0;
            touch_InfoDebug();
        }
#endif

        //Get all the saved key values ​​and exit.
        //This example will get the key values ​​after each scan, so it will only be executed once.
        //There is only one key value to process
        while(1)
        {
            // The key value of each key is 3*n+s.
            // n: key number, starting from 1.
            // s: key status, 0 is pressed, 1 is released, 2 is long pressed.
            key_val = touch_GetKey();
            if(key_val)
            {
                if (key_val % 3 == 0) //key pressed
                {
                    key_val = key_val / 3 - 1; //Get the key sequence number from left to right and from top to bottom.
                    switch (key_val)
                    {
                        //If Modifier Key(s) + a normal key are pressed, the key combination will be reported immediately.
                        //If only Modifier Key(s) is pressed, it will be reported after a timeout.
                        case 24:                                                //SHIFT
                            key_buf[0] += keyboard_val_table[key_val];
                            modifier_key_flag = 1;
                            tmos_start_task(hidEmuTaskId, START_SPECIAL_KEY_TIMEOUT_EVT, DEFAULT_SPECIAL_KEY_TIMEOUT); //Key combination timeout task
                            break;
                        case 36:                                                //CTRL
                            key_buf[0] += keyboard_val_table[key_val];
                            modifier_key_flag = 1;
                            tmos_start_task(hidEmuTaskId, START_SPECIAL_KEY_TIMEOUT_EVT, DEFAULT_SPECIAL_KEY_TIMEOUT); //Key combination timeout task
                            break;
                        case 37:                                                //WIN
                            key_buf[0] += keyboard_val_table[key_val];
                            modifier_key_flag = 1;
                            tmos_start_task(hidEmuTaskId, START_SPECIAL_KEY_TIMEOUT_EVT, DEFAULT_SPECIAL_KEY_TIMEOUT); //Key combination timeout task
                            break;
                        case 38:                                                //ALT
                            key_buf[0] += keyboard_val_table[key_val];
                            modifier_key_flag = 1;
                            tmos_start_task(hidEmuTaskId, START_SPECIAL_KEY_TIMEOUT_EVT, DEFAULT_SPECIAL_KEY_TIMEOUT); //Key combination timeout task
                            break;
                        case 43:                                                //Fn
                            key_buf[1] = keyboard_val_table[key_val];
                            fn_key_flag = 1;
                            tmos_start_task(hidEmuTaskId, START_SPECIAL_KEY_TIMEOUT_EVT, DEFAULT_SPECIAL_KEY_TIMEOUT); //Key combination timeout task
                            break;
                        default:
                            if(fn_key_flag)
                            {
                                // Add Fn key function
                            }

                            key_buf[2] = keyboard_val_table[key_val];
//                            PRINT("press:");
//                            for(uint8_t i=0; i<8; ++i)
//                            {
//                                PRINT("%x ", key_buf[i]);
//                            }
//                            PRINT("\n");
                            HidDev_Report(HID_RPT_ID_KEY_IN, HID_REPORT_TYPE_INPUT,
                                          HID_KEYBOARD_IN_RPT_LEN, key_buf);

                            if((modifier_key_flag) || (fn_key_flag))
                            {
                                modifier_key_flag = 0;
                                fn_key_flag = 0;
                                key_buf[0] = 0;
                                key_buf[1] = 0;
                                key_buf[2] = 0;
                                tmos_stop_task(hidEmuTaskId, START_SPECIAL_KEY_TIMEOUT_EVT);
                                tmos_clear_event(hidEmuTaskId, START_SPECIAL_KEY_TIMEOUT_EVT);
                            }
                            break;
                    }
                }
                else if (key_val%3 == 1) //key release
                {
                    if((modifier_key_flag == 0) && (fn_key_flag == 0))
                    {
                        key_buf[2] = 0;
//                        PRINT("release:");
//                        for(uint8_t i=0; i<8; ++i)
//                        {
//                            PRINT("%x ", key_buf[i]);
//                        }
//                        PRINT("\n");
                        HidDev_Report(HID_RPT_ID_KEY_IN, HID_REPORT_TYPE_INPUT,
                                      HID_KEYBOARD_IN_RPT_LEN, key_buf);
                    }
                }
            }
            else
            {
                break;
            }
        }

        tmos_start_task(hidEmuTaskId, START_REPORT_EVT, MS1_TO_SYSTEM_TIME(2)); //Key scan interval
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
            uint8_t ownAddr[6];
            GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddr);
            GAP_ConfigDeviceAddr(ADDRTYPE_STATIC, ownAddr);
            PRINT("Initialized..\n");
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
 * @fn      hidEmuRcvReport
 *
 * @brief   Process an incoming HID keyboard report.
 *
 * @param   len - Length of report.
 * @param   pData - Report data.
 *
 * @return  status
 */
static uint8_t hidEmuRcvReport(uint8_t len, uint8_t *pData)
{
    // verify data length
    if(len == HID_LED_OUT_RPT_LEN)
    {
        // set LEDs
        return SUCCESS;
    }
    else
    {
        return ATT_ERR_INVALID_VALUE_SIZE;
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
        if(uuid == REPORT_UUID)
        {
            // process write to LED output report; ignore others
            if(type == HID_REPORT_TYPE_OUTPUT)
            {
                status = hidEmuRcvReport(*pLen, pData);
            }
        }

        if(status == SUCCESS)
        {
            status = Hid_SetParameter(id, type, uuid, *pLen, pData);
        }
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
