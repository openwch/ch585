/********************************** (C) COPYRIGHT *******************************
 * File Name          : app_tmos.C
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2023/8/5
 * Description        : ������������
 *******************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "Touch.h"
#include "CONFIG.h"
#include "app_tmos.h"
#include "peripheral.h"
#include "backlight.h"
#include "HAL.h"
/*********************
 *      DEFINES
 *********************/
#define SLEEP_TRIGGER_TIME MS1_TO_SYSTEM_TIME(500) // 500ms
#define TRIGGER_TIME MS1_TO_SYSTEM_TIME(100)       // 100ms
#define WAKEUP_TIME MS1_TO_SYSTEM_TIME(5)          // 5ms

/**********************
 *      VARIABLES
 **********************/
tmosTaskID TouchKey_TaskID = 0x00;
uint16_t triggerTime = SLEEP_TRIGGER_TIME;
extern volatile uint8_t led_scanflag;

static const uint8_t touch_key_ch[ TOUCH_KEY_ELEMENTS ] = {TOUCH_KEY_CHS};
KEY_T s_tBtn[TOUCH_KEY_ELEMENTS] = {0};

static const uint8_t touch_wheel_ch[ TOUCH_WHEEL_ELEMENTS ] = {TOUCH_WHEEL_CHS};
uint16_t wheel_data[ TOUCH_WHEEL_ELEMENTS ] = {0};

static const uint8_t touch_slidel_ch[ TOUCH_SLIDER_ELEMENTS ] = {TOUCH_SLIDER_CHS};
uint16_t slider_data[ TOUCH_SLIDER_ELEMENTS ] = {0};

touch_button_cfg_t p_selfkey = 
{
    .num_elements = TOUCH_KEY_ELEMENTS,
    .p_elem_index = touch_key_ch,
    .p_stbtn = s_tBtn
};

touch_wheel_cfg_t p_wheel = {
    .num_elements = TOUCH_WHEEL_ELEMENTS,
    .p_elem_index = touch_wheel_ch,
    .threshold = 200,
    .decimal_point_percision = TOUCH_DECIMAL_POINT_PRECISION,
    .wheel_resolution = TOUCH_WHEEL_RESOLUTION,
    .pdata = wheel_data};

touch_slider_cfg_t p_slider = {
    .num_elements = TOUCH_SLIDER_ELEMENTS,
    .p_elem_index = touch_slidel_ch,
    .threshold = 100,
    .decimal_point_percision = TOUCH_DECIMAL_POINT_PRECISION,
    .slider_resolution = TOUCH_SLIDER_RESOLUTION,
    .pdata = slider_data
    };

touch_cfg_t touch_cfg = 
{
    .touch_button_cfg = &p_selfkey,
    .touch_slider_cfg = &p_slider,
    .touch_wheel_cfg = &p_wheel
};
/**********************
 *  STATIC PROTOTYPES
 **********************/
static void TKY_PeripheralInit(void);
static void peripherals_EnterSleep(void);
static void peripherals_WakeUp(void);
void TKY_LineSliderLedProcess (uint16_t pros);
void TKY_WheelSliderLedProcess (uint16_t pros);

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

/*********************************************************************
 * @fn      tky_on_TMOS_dataProcess
 *
 * @brief   �������ݴ�����������TMOS�����������ӳɹ��󽫻�ȡ����ֵ��֪ͨ����ʽ�ϱ�����λ������
 *
 * @return  none
 */
__HIGH_CODE
void tky_on_TMOS_dataProcess(void)
{
    uint8_t key_val = 0x00;
    uint16_t Wheel_pros = 0;
    uint16_t Slider_pros = 0;
    static uint8_t touchinsflag = 0;

    key_val = touch_GetKey();

    if (key_val != 0x00)
    {
        if (bleConnectState )
        {
            peripheralChar2Notify( &key_val, 1 );//����ֵ�ϱ�����λ������
        }
    }

    switch (key_val)
    {
        case KEY_0_DOWN :
        case KEY_0_LONG :
            key_val = 0;
            touchinsflag |= 1 << 0;
            TKY_KeyBacklightOut (0, !getBacklightState(0));
            break;
        case KEY_1_DOWN :
        case KEY_1_LONG :
            key_val = 1;
            if (!(touchinsflag & 0x0C))//������ʵ���������Ի���ʵ���ڴ���ʱ����������ǰ����ʵ��
            {
                touchinsflag |= 1 << 1;
                TKY_KeyBacklightOut (7, !getBacklightState(7));
            }
            break;
        case KEY_NONE :
            key_val = 0xfe;
            break;
        case KEY_0_UP :
            key_val = 0xff;
            touchinsflag &= ~(1 << 0);
            TKY_KeyBacklightOut (0, DISABLE);
            break;
        case KEY_1_UP :
            key_val = 0xff;
            touchinsflag &= ~(1 << 1);
            TKY_KeyBacklightOut (7, DISABLE);
            break;
        default :
            break;
    }

    if (!(touchinsflag & 0x0A))//������ʵ���������Ի���ʵ���ڴ���ʱ����������ǰ����ʵ��
    {
        Wheel_pros = touch_GetWheelSliderData();

        if (Wheel_pros < TOUCH_OFF_VALUE)
        {
            touchinsflag |= 1 << 2;

            if (bleConnectState )
            {
                peripheralChar3Notify( (uint8_t *)&Wheel_pros, 2 );//����ֵ�ϱ�����λ������
            }
        }
        else
        {
            touchinsflag &= ~(1 << 2);
        }
        TKY_WheelSliderLedProcess(Wheel_pros);
    }

    if (!(touchinsflag & 0x06))//������ʵ�����߻���ʵ���ڴ���ʱ����������ǰ���Ի���ʵ��
    {
        Slider_pros = touch_GetLineSliderData();
        if (Slider_pros < TOUCH_OFF_VALUE)
        {
            touchinsflag |= 1 << 3;

            if (bleConnectState )
            {
                peripheralChar4Notify( (uint8_t *)&Slider_pros, 2 );//����ֵ�ϱ�����λ������
            }
        }
        else
        {
            touchinsflag &= ~(1 << 3);
        }
        TKY_LineSliderLedProcess(Slider_pros);
    }

}


/*********************************************************************
 * @fn      PeriodicDealData
 *
 * @brief    ��������״̬����
 *
 * @return  none
 */
void PeriodicDealData(void)
{
    uint16_t scandata,keydata;
    TKY_LoadAndRun(); //---��������ǰ����Ĳ�������---
//    GPIOTK_PinSleep(  );

    //---����̬������ʱ�����л���ʾ���ݡ������߻����ֵ��ÿ���д���ʱ������10��wakeupʱ�䣬���˶�ʱ������ʱ��Ϊ5s---
    if (wakeUpCount)
    {
        wakeUpCount--;
//        dg_log("wakeUpCount: :%d\n", wakeUpCount);
        //---wakeUpCount����Ϊ0������̬����ת����---
        if (wakeUpCount == 0)
        {
        	touch_ScanEnterSleep();

            tmos_stop_task(TouchKey_TaskID, WAKEUP_DATA_DEAL_EVT);
            triggerTime = SLEEP_TRIGGER_TIME;
            /*-------------------------
             * Call your peripherals sleep function
             * -----------------------*/
            peripherals_EnterSleep();
        }
    }
    else //---����״̬ʱ�������������ɨ��---
    {
        dg_log("wake up...\n");

        scandata = TKY_ScanForWakeUp(tkyQueueAll); //---����ѡ��Ķ���ͨ������ɨ��---
        if (scandata) //---��ɨ�����쳣���������ʽɨ�躯��ģʽ3~4---
        {
            for (uint8_t i = 0; i < 40; i++) //---����һ��Ҫɨ��64�Σ�20�����ϽԿɣ���������������е�ɨ���а������£����˳�ѭ������������ɨ��---
            {
                keydata = TKY_PollForFilter();
                if (keydata) //---һ����⵽�а������£����˳�ѭ��ɨ��---
                {
                	touch_ScanWakeUp();
                    triggerTime = TRIGGER_TIME;
                    tky_DealData_start();
                    tmos_start_task(TouchKey_TaskID, WAKEUP_DATA_DEAL_EVT, 0);
                    /*-------------------------
                     * Call your peripherals WakeUp function
                     * -----------------------*/
                    peripherals_WakeUp();
                    break;
                }
            }
        }
    }
    TKY_SaveAndStop(); //---����ؼĴ������б���---
}


/*********************************************************************
 * @fn      tky_DealData_start
 *
 * @brief   ����ɨ�迪������
 *
 * @return  none
 */
void tky_DealData_start(void)
{
    tmos_set_event(TouchKey_TaskID, DEALDATA_EVT);
}

/*********************************************************************
 * @fn      tky_DealData_stop
 *
 * @brief   ����ɨ��ֹͣ����
 *
 * @return  none
 */
void tky_DealData_stop(void)
{
    tmos_stop_task(TouchKey_TaskID, DEALDATA_EVT);
}


/*********************************************************************
 * @fn      Touch_Key_ProcessEvent
 *
 * @brief   ��������������
 *
 * @return  none
 */
tmosEvents Touch_Key_ProcessEvent(tmosTaskID task_id, tmosEvents events)
{
    uint16_t res;

    if (events & WAKEUP_DATA_DEAL_EVT)
    {
        touch_Scan();
        tky_on_TMOS_dataProcess();
#if TKY_SLEEP_EN
        if (wakeupflag)
#endif
        tmos_start_task (TouchKey_TaskID, WAKEUP_DATA_DEAL_EVT, WAKEUP_TIME);
        return (events ^ WAKEUP_DATA_DEAL_EVT);
    }

    if (events & DEALDATA_EVT)
    {
        PeriodicDealData();
#if TKY_SLEEP_EN
        if (!advState || wakeupflag)
#endif
            tmos_start_task(TouchKey_TaskID, DEALDATA_EVT, triggerTime);
        return (events ^ DEALDATA_EVT);
    }

#if PRINT_EN
    if (events & DEBUG_PRINT_EVENT)
    {
        touch_InfoDebug();

        tmos_start_task(TouchKey_TaskID, DEBUG_PRINT_EVENT,SLEEP_TRIGGER_TIME);
        return (events ^ DEBUG_PRINT_EVENT);
    }
#endif

    if(events & TKY_KEEPALIVE_EVENT)
    {
        return events;
    }

    return 0;
}


/*********************************************************************
 * @fn      touch_on_TMOS_init
 *
 * @brief   ������ʼ������������TMOS��
 *
 * @return  none
 */
void touch_on_TMOS_init(void)
{
    TouchKey_TaskID = TMOS_ProcessEventRegister(Touch_Key_ProcessEvent);
    TKY_PeripheralInit();       /* ��ʼ���裬���米��ͷ������� */
    touch_Init(&touch_cfg);				/* ��ʼ��������  */

    wakeUpCount = 50; // ���Ѻ����ʱ�䣬��λTRIGGER_TIME(100ms)
    wakeupflag = 1;   // �óɻ���״̬
    triggerTime = TRIGGER_TIME;
    TKY_SetSleepStatusValue(~tkyQueueAll);
#if TKY_SLEEP_EN
    tky_DealData_start();
#else
    tky_DealData_stop();
#endif

#if PRINT_EN
    tmos_set_event(TouchKey_TaskID, DEBUG_PRINT_EVENT);
#endif
    tmos_set_event(TouchKey_TaskID, WAKEUP_DATA_DEAL_EVT);
    tmos_set_event(TouchKey_TaskID, TKY_KEEPALIVE_EVENT);

    TMR0_TimerInit(FREQ_SYS/1000);               //��ʱ����Ϊ1ms
    TMR0_ITCfg(ENABLE, TMR0_3_IT_CYC_END);
    PFIC_EnableIRQ( TMR0_IRQn );

    dg_log("Touch Key init Finish!\n");
}


/**********************
 *   STATIC FUNCTIONS
 **********************/


/*********************************************************************
 * @fn      TKY_PeripheralInit
 *
 * @brief   ���������ʼ�����������ڳ�ʼ���봥��������ص����蹦��
 *
 * @return  none
 */
static void TKY_PeripheralInit(void)
{
    /*You code here*/
    TKY_BacklightInit();
}


void TKY_LineSliderLedProcess (uint16_t pros)
{
    uint8_t idx = 0;
    if (pros != TOUCH_OFF_VALUE)
    {
        if (pros > 20)
            idx = (pros - 20) / 20 + 1;
        else
            idx = (pros + 100) / 20 + 1;
        for (uint8_t i = 1; i < 7; i++)
        {
            if (i == idx)
            {
                TKY_KeyBacklightOut (i, ENABLE);
            }
            else
            {
                TKY_KeyBacklightOut (i, DISABLE);
            }
        }
        printf("%d\n",pros);
    }
    else
    {
        for (uint8_t i = 1; i < 7; i++)
        {
            TKY_KeyBacklightOut (i, DISABLE);
        }
    }
}

void TKY_WheelSliderLedProcess (uint16_t pros)
{
    uint8_t idx = 0;
    if (pros != TOUCH_OFF_VALUE)
    {
        if (pros > 10)
            idx = (pros - 10) / 10 + 8;
        else
            idx = (pros + 110) / 10 + 8;

        for (uint8_t i = 8; i < 20; i++)
        {
            if (idx == i)
            {
                TKY_KeyBacklightOut (i, ENABLE);
            }
            else
            {
                TKY_KeyBacklightOut (i, DISABLE);
            }
        }
    }
    else
    {
        for (uint8_t i = 8; i < 20; i++)
        {
            TKY_KeyBacklightOut (i, DISABLE);
        }
    }
}


/*********************************************************************
 * @fn      peripherals_EnterSleep
 *
 * @brief   ����˯�ߺ������ڴ���׼������ʱ����
 *
 * @return  none
 */
static void peripherals_EnterSleep(void)
{
    /*You code here*/
    TKY_BacklightTaskStop();
    PFIC_DisableIRQ( TMR0_IRQn );
    tmos_stop_task(TouchKey_TaskID, TKY_KEEPALIVE_EVENT);
}


/*********************************************************************
 * @fn      peripherals_WakeUp
 *
 * @brief   ���軽�Ѻ������ڴ���������ʱ����
 *
 * @return  none
 */
static void peripherals_WakeUp(void)
{
    /*You code here*/
    TKY_BacklightTaskStart();
    PFIC_EnableIRQ( TMR0_IRQn );
    tmos_set_event(TouchKey_TaskID, TKY_KEEPALIVE_EVENT);
}


/*********************************************************************
 * @fn      TMR0_IRQHandler
 *
 * @brief   ��ʱ��0�жϷ�����
 *
 * @return  none
 */
__INTERRUPT
__HIGH_CODE
void TMR0_IRQHandler( void )
{
    if( TMR0_GetITFlag( TMR0_3_IT_CYC_END ) )
    {
        TMR0_ClearITFlag( TMR0_3_IT_CYC_END );
        if (led_scanflag)
        {
            /*led scan*/
            TKY_BacklightProcess();          // poll����״̬
        }
    }
}
