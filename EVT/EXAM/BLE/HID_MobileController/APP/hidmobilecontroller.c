/********************************** (C) COPYRIGHT *******************************
 * File Name          : hidmobilecontroller.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2026/10/30
 * Description        : 蓝牙移动端遥控器应用程序
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "CONFIG.h"
#include "devinfoservice.h"
#include "battservice.h"
#include "hiddev.h"
#include "hidmobilecontroller.h"
#include "hidservice.h"
#include "m3-key.h"
#include "sys_info.h"
#include "HAL.h"

/*********************************************************************
 * MACROS
 */
/* 长时间未被连接时进入shutdown睡眠的超时时间(ms) */
#define SHUT_DOWN_WAIT_TIME                  60000

/* 未连接时LED闪烁周期(ms) */
#define LED_BLINKY_PERIOD_TIME               200

#define ANDROID_TOUCH_FRAME_LEN              9

/*********************************************************************
 * CONSTANTS
 */
// Param update delay
#define START_PARAM_UPDATE_EVT_DELAY         12800

// HID idle timeout in msec; set to zero to disable timeout
#define DEFAULT_HID_IDLE_TIMEOUT             60000

// Minimum connection interval (units of 1.25ms)
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL    12

// Maximum connection interval (units of 1.25ms)
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL    12

// Slave latency to use if parameter update request
#define DEFAULT_DESIRED_SLAVE_LATENCY        5

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

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Task ID
uint8_t hidEmuTaskId = INVALID_TASK_ID;

volatile uint8_t g_camera_down = 0;         //拍照键是否按下
volatile uint8_t g_change_camera = 0;       //是否在改变拍照按键
volatile uint8_t g_up_long = 0;             //上键是否触发长按
volatile uint8_t g_down_long = 0;           //下键是否触发长按
uint8_t g_key_processing = 0;               //正在处理的按键值
uint8_t g_key_frame_idx = 0;                //按键值已经发送到第几个notify

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8_t scanRspData[] =
{
    // complete name
    0x15,                           // length of this data
    GAP_ADTYPE_LOCAL_NAME_COMPLETE, // AD Type = Complete local name
    'H',
    'I',
    'D',
    ' ',
    'M',
    'o',
    'b',
    'i',
    'l',
    'e',
    'C',
    'o',
    'n',
    't',
    'r',
    'o',
    'l',
    'l',
    'e',
    'r',

    // connection interval range
    0x05, // length of this data
    GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
    LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL), // 15ms
    HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
    LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL), // 15ms
    HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),

    // Tx power level
    0x02, // length of this data
    GAP_ADTYPE_POWER_LEVEL,
    0 // 0dBm
};

// Advertising data
static uint8_t advertData[] =
{
    // flags
    0x02, // length of this data
    GAP_ADTYPE_FLAGS,
    GAP_ADTYPE_FLAGS_LIMITED | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

    // appearance
    0x03, // length of this data
    GAP_ADTYPE_APPEARANCE,
    LO_UINT16(GAP_APPEARE_GENERIC_HID),
    HI_UINT16(GAP_APPEARE_GENERIC_HID),

    // service UUIDs（HID服务+电池服务；扫描响应数据已满31字节，故放在广播数据中）
    0x05, // length of this data
    GAP_ADTYPE_16BIT_MORE,
    LO_UINT16(HID_SERV_UUID),
    HI_UINT16(HID_SERV_UUID),
    LO_UINT16(BATT_SERV_UUID),
    HI_UINT16(BATT_SERV_UUID)
};

// Device name attribute value
static CONST uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "HID MobileController";

// HID Dev configuration
static hidDevCfg_t hidEmuCfg =
{
    DEFAULT_HID_IDLE_TIMEOUT, // Idle timeout
    HID_FEATURE_FLAGS         // HID feature flags
};

static uint16_t hidEmuConnHandle = GAP_CONNHANDLE_INIT;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void    hidEmu_ProcessTMOSMsg(tmos_event_hdr_t *pMsg);
static void    m3_hid_report(void);
static uint8_t hidEmuRptCB(uint8_t id, uint8_t type, uint16_t uuid,
                           uint8_t oper, uint16_t *pLen, uint8_t *pData);
static void    hidEmuEvtCB(uint8_t evt);
static void    hidEmuStateCB(gapRole_States_t newState, gapRoleEvent_t *pEvent);

/*********************************************************************
 * PROFILE CALLBACKS
 */

static hidDevCB_t hidEmuHidCBs =
{
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
 * @brief   Initialization function for the HidEmu App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification ... ).
 *
 * @param   none
 *
 * @return  none
 */
void HidEmu_Init(void)
{
    m3_key_init();
    GPIOB_SetBits(GPIO_Pin_4);
    GPIOB_ModeCfg(GPIO_Pin_4, GPIO_ModeOut_PP_5mA);
    g_key_processing = 0;
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

    // Set up HID service
    Hid_AddService();

    // Register for HID Dev callback
    HidDev_Register(&hidEmuCfg, &hidEmuHidCBs);

}

/*********************************************************************
 * @fn      HidEmu_ProcessEvent
 *
 * @brief   HidEmu Application Task event processor.  This function
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
    if (events & SYS_EVENT_MSG)
    {
        uint8_t *pMsg;

        if ((pMsg = tmos_msg_receive(hidEmuTaskId)) != NULL)
        {
            hidEmu_ProcessTMOSMsg((tmos_event_hdr_t *)pMsg);

            // Release the TMOS message
            tmos_msg_deallocate(pMsg);
        }

        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }

    if (events & LED_BLINK_EVT)
    {
        GPIOB_InverseBits(GPIO_Pin_4);
        return (events ^ LED_BLINK_EVT);
    }

    if (events & KEY_SCAN_EVT)
    {
        /* 按键检测扫描 */
        bsp_key_scan();
        if (m3_keys_ringmem.CurrentLen != 0)
        {
            if (g_key_processing == 0)
            {
                tmos_start_task(hidEmuTaskId, START_REPORT_EVT, 0);
            }
        }
        return (events ^ KEY_SCAN_EVT);
    }

    if (events & HIDEMU_START_DEVICE_EVT)
    {
        return (events ^ HIDEMU_START_DEVICE_EVT);
    }

    if (events & START_PARAM_UPDATE_EVT)
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

    if (events & START_REPORT_EVT)
    {
        uint8_t keyCode;
        if (g_key_processing == 0)
        {
            if (RingMemRead(&m3_keys_ringmem, &keyCode, 1) == SUCCESS)
            {
                g_key_processing = keyCode;
                g_key_frame_idx = 0;
            }
        }
        if (g_key_processing != 0)
        {
            m3_hid_report();
        }

        if ((g_key_processing != 0) || (m3_keys_ringmem.CurrentLen != 0) || (is_any_btn_down()))
        {
            tmos_start_task(hidEmuTaskId, START_REPORT_EVT, MS1_TO_SYSTEM_TIME(45));
        }
        return (events ^ START_REPORT_EVT);
    }
    if (events & START_SLEEP_EVT)
    {
        /* 长时间没有连接，直接shutdown睡眠 */
        PRINT("No Connect... Shutdown\n");
        GPIOB_SetBits(GPIO_Pin_4);
        GPIOA_ITModeCfg(GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_4 | GPIO_Pin_5, GPIO_ITMode_FallEdge); //需要睡眠时再启用中断，下降沿唤醒
        PFIC_EnableIRQ(GPIO_A_IRQn);
        PWR_PeriphWakeUpCfg(ENABLE, RB_SLP_GPIO_WAKE, Long_Delay);
        DelayMs(2);
        LowPower_Shutdown(0); //全部断电，唤醒后复位
        /*
                         此模式唤醒后会执行复位，所以下面代码不会运行，
                         注意要确保系统睡下去再唤醒才是唤醒复位，否则有可能变成IDLE等级唤醒
         */
        HSECFG_Current(HSE_RCur_100); // 降为额定电流(低功耗函数中提升了HSE偏置电流)
        PRINT("wake.. \n");
        DelayMs(100);
        return (events ^ START_SLEEP_EVT);
    }

    if (events & SAVE_AN_CONFIG_EVT)
    {
        save_android_config();
        return (events ^ SAVE_AN_CONFIG_EVT);
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
    switch (pMsg->event)
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
    switch (newState & GAPROLE_STATE_ADV_MASK)
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
        PRINT("Advertising..\n");
        tmos_start_task(hidEmuTaskId, START_SLEEP_EVT, MS1_TO_SYSTEM_TIME(SHUT_DOWN_WAIT_TIME));
        tmos_start_reload_task(hidEmuTaskId, LED_BLINK_EVT, MS1_TO_SYSTEM_TIME(LED_BLINKY_PERIOD_TIME));
        break;

    case GAPROLE_CONNECTED:
    {
        gapEstLinkReqEvent_t *event = (gapEstLinkReqEvent_t *)pEvent;
        tmos_stop_task(hidEmuTaskId, START_SLEEP_EVT);
        tmos_clear_event(hidEmuTaskId, START_SLEEP_EVT);
        /* 连接上关灯 */
        tmos_stop_task(hidEmuTaskId, LED_BLINK_EVT);
        tmos_clear_event(hidEmuTaskId, LED_BLINK_EVT);
        GPIOB_SetBits(GPIO_Pin_4);

        g_key_processing = 0;  //正在处理的按键值
        g_key_frame_idx = 0;   //按键值已经发送到第几个notify
        g_camera_down = 0;     //拍照键是否按下
        g_change_camera = 0;
        g_up_long = 0;
        g_down_long = 0;
        //清除按键缓存，开始按键扫描
        m3_key_start();
        //tmos_start_reload_task(hidEmuTaskId, START_REPORT_EVT, MS1_TO_SYSTEM_TIME(20));
        // get connection handle
        hidEmuConnHandle = event->connectionHandle;
        tmos_start_task(hidEmuTaskId, START_PARAM_UPDATE_EVT, START_PARAM_UPDATE_EVT_DELAY);
        PRINT("Connected..\n");
    }
    break;

    case GAPROLE_CONNECTED_ADV:
        PRINT("Connected Advertising..\n");
        break;

    case GAPROLE_WAITING:
        if (pEvent->gap.opcode == GAP_END_DISCOVERABLE_DONE_EVENT)
        {
            PRINT("Waiting for advertising..\n");
        }
        else if (pEvent->gap.opcode == GAP_LINK_TERMINATED_EVENT)
        {
            PRINT("Disconnected.. Reason:%x\n", pEvent->linkTerminate.reason);
        }
        else if (pEvent->gap.opcode == GAP_LINK_ESTABLISHED_EVENT)
        {
            PRINT("Advertising timeout..\n");
        }
        // Enable advertising
        {
            uint8_t initial_advertising_enable = TRUE;
            // Set the GAP Role Parameters
            GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &initial_advertising_enable);
        }
        m3_key_stop();
        tmos_stop_task(hidEmuTaskId, START_REPORT_EVT);
        tmos_clear_event(hidEmuTaskId, START_REPORT_EVT);
        break;

    case GAPROLE_ERROR:
        PRINT("Error..\n");
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
    if (oper == HID_DEV_OPER_WRITE)
    {
        status = Hid_SetParameter(id, type, uuid, *pLen, pData);
    }
    // read
    else if (oper == HID_DEV_OPER_READ)
    {
        status = Hid_GetParameter(id, type, uuid, pLen, pData);
    }
    // notifications enabled
    else if (oper == HID_DEV_OPER_ENABLE)
    {
        //tmos_start_task(hidEmuTaskId, START_REPORT_EVT, 500);
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
 * @fn      m3_hid_report
 *
 * @brief   根据当前待处理的按键事件，按预设报文序列分次上报HID报文
 *          （触摸屏/Consumer，仅支持安卓手机），直至本次按键事件处理完毕
 *
 * @param   none
 *
 * @return  none
 */
static void m3_hid_report(void)
{
    uint8_t              status;
    //PRINT("mr:%x,%d\n",g_key_processing,g_key_frame_idx);
    switch (g_key_processing)
    {

    case MIDDLE_KEY_DOWN:  //中间
        if (g_camera_down == 0)
        {
            KEY_MIDDLE_ANDROID[0] = 0x07;
            status = HidDev_Report(HID_RPT_ID_ADIG_IMOU, HID_REPORT_TYPE_INPUT, ANDROID_TOUCH_FRAME_LEN, &KEY_MIDDLE_ANDROID[0]);
            if (status == 0)
            {
                g_key_processing = 0;
            }
        }
        else
        {
            //修改拍照按键点位，恢复默认点
            if (g_change_camera == 0)
            {
                g_an_camera_pointer.down_pointer[0] = 0x50;
                g_an_camera_pointer.down_pointer[1] = 0x03;

                g_an_camera_pointer.left_pointer[0] = 0xF5;
                g_an_camera_pointer.left_pointer[1] = 0x01;

                KEY_CAMERA_ANDROID[1] = g_an_camera_pointer.left_pointer[0];
                KEY_CAMERA_ANDROID[2] = g_an_camera_pointer.left_pointer[1];

                KEY_CAMERA_ANDROID[3] = g_an_camera_pointer.down_pointer[0];
                KEY_CAMERA_ANDROID[4] = g_an_camera_pointer.down_pointer[1];
                tmos_start_task(hidEmuTaskId, SAVE_AN_CONFIG_EVT, MS1_TO_SYSTEM_TIME(5000));    /* 5s以后保存参数，提升flash寿命：save_android_config(); */
                g_change_camera = 1;    //锁定
                PRINT("cl:%x,%x\n", KEY_CAMERA_ANDROID[1], KEY_CAMERA_ANDROID[2]);
                PRINT("cd:%x,%x\n", KEY_CAMERA_ANDROID[3], KEY_CAMERA_ANDROID[4]);
            }

            if (g_key_frame_idx == 0)
            {
                /* 先释放按键，再发送点位改变 */
                KEY_CAMERA_ANDROID[0] = ANDROID_FINGER_UP;
                status = HidDev_Report(HID_RPT_ID_ADIG_IMOU, HID_REPORT_TYPE_INPUT, ANDROID_TOUCH_FRAME_LEN, &KEY_CAMERA_ANDROID[0]);
                if (status == 0)
                {
                    g_key_frame_idx = 1;
                }
            }

            if (g_key_frame_idx == 1)
            {
                KEY_CAMERA_ANDROID[0] = ANDROID_FINGER_DOWN;
                status = HidDev_Report(HID_RPT_ID_ADIG_IMOU, HID_REPORT_TYPE_INPUT, ANDROID_TOUCH_FRAME_LEN, &KEY_CAMERA_ANDROID[0]);
                if (status == 0)
                {
                    g_key_processing = 0;
                    g_change_camera = 0;    //清除锁定
                }
            }
        }
        break;
    case DOWN_KEY_UP:   //下松开
        if (g_down_long == 0)
        {
            if (g_camera_down == 0)
            {
                while (1)
                {
                    status = HidDev_Report(HID_RPT_ID_ADIG_IMOU, HID_REPORT_TYPE_INPUT, ANDROID_TOUCH_FRAME_LEN, (uint8_t *)&KEY_DOWN_ANDROID[ANDROID_TOUCH_FRAME_LEN * g_key_frame_idx]);
                    if (status)
                    {
                        break;
                    }
                    else
                    {
                        g_key_frame_idx++;
                        if (g_key_frame_idx >= (sizeof(KEY_DOWN_ANDROID) / ANDROID_TOUCH_FRAME_LEN))
                        {
                            g_key_processing = 0;
                            break;
                        }
                    }
                }
            }
            else
            {
                //修改拍照按键点位
                if (g_change_camera == 0)
                {
                    if ((g_an_camera_pointer.down_pointer[1] >= 0x03) && (g_an_camera_pointer.down_pointer[0] >= 0xe0))
                    {
                        g_an_camera_pointer.down_pointer[0] = 0x10;
                        g_an_camera_pointer.down_pointer[1] = 0x00;
                    }
                    else
                    {
                        if (g_an_camera_pointer.down_pointer[0] >= 0xf0)
                        {
                            g_an_camera_pointer.down_pointer[0] = 0x10;
                            g_an_camera_pointer.down_pointer[1]++;
                        }
                        else
                        {
                            g_an_camera_pointer.down_pointer[0] = g_an_camera_pointer.down_pointer[0] + 0x10;
                        }
                    }
                    KEY_CAMERA_ANDROID[3] = g_an_camera_pointer.down_pointer[0];
                    KEY_CAMERA_ANDROID[4] = g_an_camera_pointer.down_pointer[1];
                    tmos_start_task(hidEmuTaskId, SAVE_AN_CONFIG_EVT, MS1_TO_SYSTEM_TIME(5000));    /* 5s以后保存参数，提升flash寿命：save_android_config(); */
                    g_change_camera = 1;    //锁定
                    PRINT("cd:%x,%x\n", KEY_CAMERA_ANDROID[3], KEY_CAMERA_ANDROID[4]);
                }

                if (g_key_frame_idx == 0)
                {
                    /* 先释放按键，再发送点位改变 */
                    KEY_CAMERA_ANDROID[0] = ANDROID_FINGER_UP;
                    status = HidDev_Report(HID_RPT_ID_ADIG_IMOU, HID_REPORT_TYPE_INPUT, ANDROID_TOUCH_FRAME_LEN, &KEY_CAMERA_ANDROID[0]);
                    if (status == 0)
                    {
                        g_key_frame_idx = 1;
                    }
                }

                if (g_key_frame_idx == 1)
                {
                    KEY_CAMERA_ANDROID[0] = ANDROID_FINGER_DOWN;
                    status = HidDev_Report(HID_RPT_ID_ADIG_IMOU, HID_REPORT_TYPE_INPUT, ANDROID_TOUCH_FRAME_LEN, &KEY_CAMERA_ANDROID[0]);
                    if (status == 0)
                    {
                        g_key_processing = 0;
                        g_change_camera = 0;    //清除锁定
                    }
                }
            }
        }
        else
        {
            /* 长按事件触发，不发送下按下事件 */
            g_down_long = 0;
            g_key_processing = 0;
        }
        break;

    case UP_KEY_UP:  //上
        if (g_up_long == 0)
        {
            if (g_camera_down == 0)
            {
                while (1)
                {
                    status = HidDev_Report(HID_RPT_ID_ADIG_IMOU, HID_REPORT_TYPE_INPUT, ANDROID_TOUCH_FRAME_LEN, (uint8_t *)&KEY_UP_ANDROID[ANDROID_TOUCH_FRAME_LEN * g_key_frame_idx]);
                    if (status)
                    {
                        break;
                    }
                    else
                    {
                        g_key_frame_idx++;
                        if (g_key_frame_idx >= (sizeof(KEY_UP_ANDROID) / ANDROID_TOUCH_FRAME_LEN))
                        {
                            g_key_processing = 0;
                            break;
                        }
                    }
                }
            }
            else
            {
                //修改拍照按键点位
                if (g_change_camera == 0)
                {
                    if ((g_an_camera_pointer.down_pointer[1] <= 0x00) && (g_an_camera_pointer.down_pointer[0] <= 0x10))
                    {
                        g_an_camera_pointer.down_pointer[0] = 0xe0;
                        g_an_camera_pointer.down_pointer[1] = 0x03;
                    }
                    else
                    {
                        if (g_an_camera_pointer.down_pointer[0] <= 0x10)
                        {
                            g_an_camera_pointer.down_pointer[0] = 0xf0;
                            g_an_camera_pointer.down_pointer[1]--;
                        }
                        else
                        {
                            g_an_camera_pointer.down_pointer[0] = g_an_camera_pointer.down_pointer[0] - 0x10;
                        }
                    }
                    KEY_CAMERA_ANDROID[3] = g_an_camera_pointer.down_pointer[0];
                    KEY_CAMERA_ANDROID[4] = g_an_camera_pointer.down_pointer[1];
                    tmos_start_task(hidEmuTaskId, SAVE_AN_CONFIG_EVT, MS1_TO_SYSTEM_TIME(5000));    /* 5s以后保存参数，提升flash寿命：save_android_config(); */
                    g_change_camera = 1;    //锁定
                    PRINT("cd:%x,%x\n", KEY_CAMERA_ANDROID[3], KEY_CAMERA_ANDROID[4]);
                }

                if (g_key_frame_idx == 0)
                {
                    /* 先释放按键，再发送点位改变 */
                    KEY_CAMERA_ANDROID[0] = ANDROID_FINGER_UP;
                    status = HidDev_Report(HID_RPT_ID_ADIG_IMOU, HID_REPORT_TYPE_INPUT, ANDROID_TOUCH_FRAME_LEN, &KEY_CAMERA_ANDROID[0]);
                    if (status == 0)
                    {
                        g_key_frame_idx = 1;
                    }
                }

                if (g_key_frame_idx == 1)
                {
                    KEY_CAMERA_ANDROID[0] = ANDROID_FINGER_DOWN;
                    status = HidDev_Report(HID_RPT_ID_ADIG_IMOU, HID_REPORT_TYPE_INPUT, ANDROID_TOUCH_FRAME_LEN, &KEY_CAMERA_ANDROID[0]);
                    if (status == 0)
                    {
                        g_key_processing = 0;
                        g_change_camera = 0;    //清除锁定
                    }
                }
            }
        }
        else
        {
            /* 长按事件触发，不发送上按下事件 */
            g_up_long = 0;
            g_key_processing = 0;
        }
        break;

    case RIGHT_KEY_UP:  //右
        if (g_camera_down == 0)
        {
            while (1)
            {
                status = HidDev_Report(HID_RPT_ID_ADIG_IMOU, HID_REPORT_TYPE_INPUT, ANDROID_TOUCH_FRAME_LEN, (uint8_t *)&KEY_RIGHT_ANDROID[ANDROID_TOUCH_FRAME_LEN * g_key_frame_idx]);
                if (status)
                {
                    break;
                }
                else
                {
                    g_key_frame_idx++;
                    if (g_key_frame_idx >= (sizeof(KEY_RIGHT_ANDROID) / ANDROID_TOUCH_FRAME_LEN))
                    {
                        g_key_processing = 0;
                        break;
                    }
                }
            }
        }
        else
        {
            //修改拍照按键点位
            if (g_change_camera == 0)
            {
                if ((g_an_camera_pointer.left_pointer[1] >= 0x03) && (g_an_camera_pointer.left_pointer[0] >= 0xd5))
                {
                    g_an_camera_pointer.left_pointer[0] = 0x15;
                    g_an_camera_pointer.left_pointer[1] = 0x00;
                }
                else
                {
                    if (g_an_camera_pointer.left_pointer[0] >= 0xf5)
                    {
                        g_an_camera_pointer.left_pointer[0] = 0x15;
                        g_an_camera_pointer.left_pointer[1]++;
                    }
                    else
                    {
                        g_an_camera_pointer.left_pointer[0] = g_an_camera_pointer.left_pointer[0] + 0x10;
                    }
                }
                KEY_CAMERA_ANDROID[1] = g_an_camera_pointer.left_pointer[0];
                KEY_CAMERA_ANDROID[2] = g_an_camera_pointer.left_pointer[1];
                PRINT("cl:%x,%x\n", KEY_CAMERA_ANDROID[1], KEY_CAMERA_ANDROID[2]);
                tmos_start_task(hidEmuTaskId, SAVE_AN_CONFIG_EVT, MS1_TO_SYSTEM_TIME(5000));    /* 5s以后保存参数，提升flash寿命：save_android_config(); */
                g_change_camera = 1;    //锁定
            }

            if (g_key_frame_idx == 0)
            {
                /* 先释放按键，再发送点位改变 */
                KEY_CAMERA_ANDROID[0] = ANDROID_FINGER_UP;
                status = HidDev_Report(HID_RPT_ID_ADIG_IMOU, HID_REPORT_TYPE_INPUT, ANDROID_TOUCH_FRAME_LEN, &KEY_CAMERA_ANDROID[0]);
                if (status == 0)
                {
                    g_key_frame_idx = 1;
                }
            }

            if (g_key_frame_idx == 1)
            {
                KEY_CAMERA_ANDROID[0] = ANDROID_FINGER_DOWN;
                status = HidDev_Report(HID_RPT_ID_ADIG_IMOU, HID_REPORT_TYPE_INPUT, ANDROID_TOUCH_FRAME_LEN, &KEY_CAMERA_ANDROID[0]);
                if (status == 0)
                {
                    g_key_processing = 0;
                    g_change_camera = 0;    //清除锁定
                }
            }
        }
        break;

    case LEFT_KEY_UP:  //左
        if (g_camera_down == 0)
        {
            while (1)
            {
                status = HidDev_Report(HID_RPT_ID_ADIG_IMOU, HID_REPORT_TYPE_INPUT, ANDROID_TOUCH_FRAME_LEN, (uint8_t *)&KEY_LEFT_ANDROID[ANDROID_TOUCH_FRAME_LEN * g_key_frame_idx]);
                if (status)
                {
                    break;
                }
                else
                {
                    g_key_frame_idx++;
                    if (g_key_frame_idx >= (sizeof(KEY_LEFT_ANDROID) / ANDROID_TOUCH_FRAME_LEN))
                    {
                        g_key_processing = 0;
                        break;
                    }
                }
            }
        }
        else
        {
            //修改拍照按键点位
            if (g_change_camera == 0)
            {
                if ((g_an_camera_pointer.left_pointer[1] <= 0x00) && (g_an_camera_pointer.left_pointer[0] <= 0x15))
                {
                    g_an_camera_pointer.left_pointer[0] = 0xd5;
                    g_an_camera_pointer.left_pointer[1] = 0x03;
                }
                else
                {
                    if (g_an_camera_pointer.left_pointer[0] <= 0x15)
                    {
                        g_an_camera_pointer.left_pointer[0] = 0xf5;
                        g_an_camera_pointer.left_pointer[1]--;
                    }
                    else
                    {
                        g_an_camera_pointer.left_pointer[0] = g_an_camera_pointer.left_pointer[0] - 0x10;
                    }
                }
                KEY_CAMERA_ANDROID[1] = g_an_camera_pointer.left_pointer[0];
                KEY_CAMERA_ANDROID[2] = g_an_camera_pointer.left_pointer[1];
                tmos_start_task(hidEmuTaskId, SAVE_AN_CONFIG_EVT, MS1_TO_SYSTEM_TIME(5000));    /* 5s以后保存参数，提升flash寿命：save_android_config(); */
                g_change_camera = 1;    //锁定
                PRINT("cl:%x,%x\n", KEY_CAMERA_ANDROID[1], KEY_CAMERA_ANDROID[2]);
            }

            if (g_key_frame_idx == 0)
            {
                /* 先释放按键，再发送点位改变 */
                KEY_CAMERA_ANDROID[0] = ANDROID_FINGER_UP;
                status = HidDev_Report(HID_RPT_ID_ADIG_IMOU, HID_REPORT_TYPE_INPUT, ANDROID_TOUCH_FRAME_LEN, &KEY_CAMERA_ANDROID[0]);
                if (status == 0)
                {
                    g_key_frame_idx = 1;
                }
            }

            if (g_key_frame_idx == 1)
            {
                KEY_CAMERA_ANDROID[0] = ANDROID_FINGER_DOWN;
                status = HidDev_Report(HID_RPT_ID_ADIG_IMOU, HID_REPORT_TYPE_INPUT, ANDROID_TOUCH_FRAME_LEN, &KEY_CAMERA_ANDROID[0]);
                if (status == 0)
                {
                    g_key_processing = 0;
                    g_change_camera = 0;    //清除锁定
                }
            }
        }
        break;
    case CAMERA_KEY_DOWN:  //拍照
        KEY_CAMERA_ANDROID[0] = ANDROID_FINGER_DOWN;
        status = HidDev_Report(HID_RPT_ID_ADIG_IMOU, HID_REPORT_TYPE_INPUT, ANDROID_TOUCH_FRAME_LEN, &KEY_CAMERA_ANDROID[0]);
        if (status == 0)
        {
            g_camera_down = 1; /* 照相按键按下 */
            g_key_processing = 0;
            g_change_camera = 0;    //清除锁定
        }
        break;

    case CAMERA_KEY_UP:  //拍照
        KEY_CAMERA_ANDROID[0] = ANDROID_FINGER_UP;
        status = HidDev_Report(HID_RPT_ID_ADIG_IMOU, HID_REPORT_TYPE_INPUT, ANDROID_TOUCH_FRAME_LEN, &KEY_CAMERA_ANDROID[0]);
        if (status == 0)
        {
            g_camera_down = 0;
            g_key_processing = 0;
        }
        break;
    case MIDDLE_KEY_UP:  //中间
        if (g_camera_down == 0)
        {
            KEY_MIDDLE_ANDROID[0] = 0;
            status = HidDev_Report(HID_RPT_ID_ADIG_IMOU, HID_REPORT_TYPE_INPUT, ANDROID_TOUCH_FRAME_LEN, &KEY_MIDDLE_ANDROID[0]);
            if (status == 0)
            {
                g_key_processing = 0;
            }
        }
        else
        {
            g_key_processing = 0;
        }
        break;
    case DOWN_KEY_LONG:
        if (g_camera_down == 0)
        {
            /* 拍照键没有按下时，才触发长按 */
            g_down_long = 1;
        }
        g_key_processing = 0;
        break;
    case DOWN_KEY_DOWN:  //下
        if (g_down_long != 0)  /* 长按已触发 */
        {
            /* 发送音量--， KEY_VOLUME */
            if (g_key_frame_idx == 0)
            {
                status = HidDev_Report(HID_RPT_ID_ACSU_ICSU, HID_REPORT_TYPE_INPUT, 2, (uint8_t *)&KEY_VOLUME[4]);
                if (status == 0)
                {
                    g_key_frame_idx = 1;
                }
            }

            if (g_key_frame_idx == 1)
            {
                status = HidDev_Report(HID_RPT_ID_ACSU_ICSU, HID_REPORT_TYPE_INPUT, 2, (uint8_t *)&KEY_VOLUME[0]);
                if (status == 0)
                {
                    g_key_processing = 0;
                }
            }
        }
        else
        {
            g_key_processing = 0;
        }
        break;

    case UP_KEY_LONG:
        if (g_camera_down == 0)
        {
            /* 拍照键没有按下时，才触发长按 */
            g_up_long = 1;
        }
        g_key_processing = 0;
        break;
    case UP_KEY_DOWN:
        if (g_up_long != 0)
        {
            /* 发送音量++ */
            if (g_key_frame_idx == 0)
            {
                status = HidDev_Report(HID_RPT_ID_ACSU_ICSU, HID_REPORT_TYPE_INPUT, 2, (uint8_t *)&KEY_VOLUME[2]);
                if (status == 0)
                {
                    g_key_frame_idx = 1;
                }
            }

            if (g_key_frame_idx == 1)
            {
                status = HidDev_Report(HID_RPT_ID_ACSU_ICSU, HID_REPORT_TYPE_INPUT, 2, (uint8_t *)&KEY_VOLUME[0]);
                if (status == 0)
                {
                    g_key_processing = 0;
                }
            }
        }
        else
        {
            g_key_processing = 0;
        }
        break;
    default:
        g_key_processing = 0;
        break;
    }
}
/*********************************************************************
*********************************************************************/

