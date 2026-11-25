/********************************** (C) COPYRIGHT *******************************
 * File Name          : Touch.C
 * Author             : WCH
 * Version            : V1.6
 * Date               : 2021/12/1
 * Description        : 触摸按键例程
 *******************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "Touch.h"

/*********************
 *      DEFINES
 *********************/
#define WAKEUPTIME          20                             // Sleep Time = 20 * SLEEP_TRIGGER_TIME(500ms) = 10s

/**********************
 *      VARIABLES
 **********************/
__attribute__( ( aligned( 4 ) ) ) uint32_t TKY_MEMBUF[ ( TKY_MEMHEAP_SIZE - 1 ) / 4 + 1 ] = { 0 };
static uint16_t       keyData       = 0;                // 触摸按键转换结果
static uint16_t       WheelData     = TOUCH_OFF_VALUE;  // 触摸滑轮转换结果
static uint16_t       SilderData    = TOUCH_OFF_VALUE;  // 触摸滑条转换结果
static touch_cfg_t*   p_touch_cfg   = NULL;
uint8_t               wakeUpCount   = 0;
uint8_t               wakeupflag    = 0;
uint16_t              tkyQueueAll   = 0;
uint16_t              tkyChAll      = 0;
static uint32_t touch_pin_a = 0;
uint16_t CTransCC[ 4 ]              = { 1, 0, 2, 3 };
const uint32_t TKY_Pin[ 14 ] = {
    GPIO_Pin_4, GPIO_Pin_5, GPIO_Pin_12, GPIO_Pin_13, GPIO_Pin_14, GPIO_Pin_15, GPIO_Pin_3,
    GPIO_Pin_2, GPIO_Pin_1, GPIO_Pin_0, GPIO_Pin_6, GPIO_Pin_7, GPIO_Pin_8, GPIO_Pin_9 };
static const TKY_ChannelInitTypeDef my_tky_ch_init[ TKY_QUEUE_END ] = { TKY_CHS_INIT };
/**********************
 *  STATIC PROTOTYPES
 **********************/

static KEY_FIFO_T s_tKey; /* 按键FIFO变量,结构体 */
static void       touch_InitHard( void );
static void       touch_InitVar( touch_cfg_t* p );
static void       touch_PutKey( uint8_t _KeyCode );
static void       touch_DetectKey( touch_button_cfg_t* p );
static void       touch_Regcfg( void );
static void       touch_Baseinit( void );
static void       touch_Channelinit( void );
static uint16_t   touch_DetecLineSlider( touch_slider_cfg_t* p_slider );
static uint16_t   touch_DetectWheelSlider( touch_wheel_cfg_t* p_wheel );
void              touch_ResetBaseline( void );
static uint8_t    touch_enable_version2_mode( void );

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

/********************************************************************************************************
 * @fn      touch_Init
 *
 * @brief   初始化按键. 该函数被 TKY_Init() 调用。
 *
 * @return  none
 */
void touch_Init( touch_cfg_t* p )
{
    touch_InitHard();   /* 初始化Touch硬件和库基本参数 */
    touch_InitVar( p ); /* 初始化按键变量 */
}

/********************************************************************************************************
 * @fn      touch_PutKey
 * @brief   将1个键值压入按键FIFO缓冲区。可用于模拟一个按键。
 * @param   _KeyCode - 按键代码
 * @return  none
 */
static void touch_PutKey( uint8_t _KeyCode )
{
    s_tKey.Buf[ s_tKey.Write ] = _KeyCode;

    if ( ++s_tKey.Write >= KEY_FIFO_SIZE )
    {
        s_tKey.Write = 0;
    }
}

/********************************************************************************************************
 * @fn      touch_GetKey
 * @brief   从按键FIFO缓冲区读取一个键值。
 * @param   无
 * @return  按键代码
 */
uint8_t touch_GetKey( void )
{
    uint8_t ret;

    if ( s_tKey.Read == s_tKey.Write )
    {
        return KEY_NONE;
    }
    else
    {
        ret = s_tKey.Buf[ s_tKey.Read ];

        if ( ++s_tKey.Read >= KEY_FIFO_SIZE )
        {
            s_tKey.Read = 0;
        }
        return ret;
    }
}

/********************************************************************************************************
 * @fn      touch_GetKeyState
 * @brief   读取按键的状态
 * @param   _ucKeyID - 按键ID，从0开始
 * @return  1 - 按下
 *          0 - 未按下
 *********************************************************************************************************
 */
uint8_t touch_GetKeyState( KEY_ID_E _ucKeyID )
{
    return p_touch_cfg->touch_button_cfg->p_stbtn[ _ucKeyID ].State;
}

/********************************************************************************************************
 * @fn      touch_SetKeyParam
 * @brief   设置按键参数
 * @param   _ucKeyID     - 按键ID，从0开始
 *          _LongTime    - 长按事件时间
 *          _RepeatSpeed - 连发速度
 * @return  none
 */
void touch_SetKeyParam( uint8_t _ucKeyID, uint16_t _LongTime, uint8_t _RepeatSpeed )
{
    p_touch_cfg->touch_button_cfg->p_stbtn[ _ucKeyID ].LongTime    = _LongTime;    /* 长按时间 0 表示不检测长按键事件 */
    p_touch_cfg->touch_button_cfg->p_stbtn[ _ucKeyID ].RepeatSpeed = _RepeatSpeed; /* 按键连发的速度，0表示不支持连发 */
    p_touch_cfg->touch_button_cfg->p_stbtn[ _ucKeyID ].RepeatCount = 0;            /* 连发计数器 */
}

/********************************************************************************************************
 * @fn      touch_ClearKey
 * @brief   清空按键FIFO缓冲区
 * @param   无
 * @return  按键代码
 */
void touch_ClearKey( void )
{
    s_tKey.Read = s_tKey.Write;
}

/********************************************************************************************************
 * @fn      touch_ScanWakeUp
 * @brief   触摸扫描唤醒函数
 * @param   无
 * @return  无
 */
void touch_ScanWakeUp( void )
{
    wakeUpCount = WAKEUPTIME;  //---唤醒时间---
    wakeupflag  = 1;           // 置成唤醒状态
    touch_ResetBaselineWakeup();  // 唤醒后重置扫描基线
    dg_log( "wake up for a while\n" );

}

/********************************************************************************************************
 * @fn      touch_ScanEnterSleep
 * @brief   触摸扫描休眠函数
 * @param   无
 * @return  无
 */
void touch_ScanEnterSleep( void )
{
    wakeupflag = 0;              // 置成睡眠状态:0,唤醒态:1
    touch_ResetBaselineSleep();  // 进入睡眠前重置睡眠基线
    dg_log( "Ready to sleep\n" );
}

static uint16_t  tky_baseval[TKY_MAX_QUEUE_NUM] = {0};
static uint16_t  tky_th1[TKY_MAX_QUEUE_NUM] = {0};
static uint16_t  tky_th2[TKY_MAX_QUEUE_NUM] = {0};
static uint16_t  tky_realval[TKY_MAX_QUEUE_NUM] = {0};
static uint16_t  tky_filterval[TKY_MAX_QUEUE_NUM] = {0};
/********************************************************************************************************
 * @fn      touch_Scan
 * @brief   扫描所有按键。非阻塞，应在主循环或TMOS任务中周期调用，切勿在中断中调用（内部含阻塞轮询与日志打印）
 * @param   无
 * @return  无
 */
void touch_Scan( void )
{
    uint8_t i;

    TKY_LoadAndRun();   //---载入休眠前保存的部分设置---
    keyData = TKY_PollForFilter();
    TKY_SaveAndStop();  //---对相关寄存器进行保存---

    //TODO:
    // uint16_t tky_thr1 = 0, tky_thr2 = 0, key_num = 0, key_num2 = 0;
    // uint16_t diffval = 0,baseline = 0,realval = 0;
    // for (uint8_t j = 0; j < TKY_TOUCH_QUEUE_NUM; j++)
    // {
    //    TKY_GetCurQueueThreshold( j ,&tky_thr1, &tky_thr2);
    //    TKY_GetCurQueueBaseLine(j, &tky_baseval[j]);
    //    TKY_GetCurQueueFilterVal(j, &tky_filterval[j]);
    //    if ( tky_baseval[j] < tky_filterval[j] )
    //    {
    //         diffval = tky_filterval[j] - tky_baseval[j];
    //         if ( ( tky_thr1 >> 1 ) < diffval )  //- TOUCH_MIN_THR
    //         {
    //             key_num++;
    //         }
    //    }
    //    else
    //    {
    //         diffval = tky_baseval[j] - tky_filterval[j];
    //         if ( ( tky_thr1 ) < diffval )
    //         {
    //             key_num2++;
    //         }
    //    }
    // }

    // if(((key_num >= (TKY_TOUCH_QUEUE_NUM>>1))&& keyData == 0) || key_num2)
    // {
    //     // printf("*****************************************\r\n");
    //     for (uint8_t i = 0; i < TKY_TOUCH_QUEUE_NUM; i++)
    //     {
    //         TKY_SetCurQueueBaseLine(i, tky_filterval[i]);
    //     }
    //     keyData = 0;
    //     TKY_ClearHistoryData(TKY_FILTER_MODE);
    // }

#if TKY_SLEEP_EN
    if ( keyData )
    {
        wakeUpCount = WAKEUPTIME;  //---唤醒时间---
    }
#endif

    touch_DetectKey( p_touch_cfg->touch_button_cfg );

    WheelData  = touch_DetectWheelSlider( p_touch_cfg->touch_wheel_cfg );

    SilderData = touch_DetecLineSlider( p_touch_cfg->touch_slider_cfg );
}

/********************************************************************************************************
 * @fn      touch_IOSleep
 * @brief   触摸按键IO模式配置
 * @param   无
 * @return  无
 */
void touch_IOSleep(void)
{
    R32_PA_PU &= ~touch_pin_a;
    R32_PA_PD_DRV &= ~touch_pin_a;
    R32_PA_DIR |= touch_pin_a;
    R32_PA_CLR |= touch_pin_a;
}
/********************************************************************************************************
 * @fn      touch_IOWork
 * @brief   触摸按键IO模式配置
 * @param   无
 * @return  无
 */
void touch_IOWork(void)
{
    R32_PA_PD_DRV &= ~touch_pin_a;
    R32_PA_PU &= ~touch_pin_a;
    R32_PA_DIR &= ~touch_pin_a;
}
/********************************************************************************************************
 * @fn      touch_InitHard
 * @brief   初始化触摸按键
 * @param   无
 * @return  无
 */
static void touch_InitHard( void )
{
    touch_Baseinit();
    touch_Regcfg();
    touch_Channelinit();    
    
}

/********************************************************************************************************
 * @fn      touch_InitVar
 * @brief   初始化触摸按键变量
 * @param   无
 * @return  无
 */
static void touch_InitVar( touch_cfg_t* p )
{
    uint8_t i;

    p_touch_cfg  = p;

    /* 对按键FIFO读写指针清零 */
    s_tKey.Read  = 0;
    s_tKey.Write = 0;

    /* 给每个按键结构体成员变量赋一组缺省值 */
    for ( i = 0; i < p_touch_cfg->touch_button_cfg->num_elements; i++ )
    {
        p_touch_cfg->touch_button_cfg->p_stbtn[ i ].LongTime    = KEY_LONG_TIME;       /* 长按时间 0 表示不检测长按键事件 */
        p_touch_cfg->touch_button_cfg->p_stbtn[ i ].Count       = KEY_FILTER_TIME / 2; /* 计数器设置为滤波时间的一半 */
        p_touch_cfg->touch_button_cfg->p_stbtn[ i ].State       = 0;                   /* 按键缺省状态，0为未按下 */
        p_touch_cfg->touch_button_cfg->p_stbtn[ i ].RepeatSpeed = KEY_REPEAT_TIME;     /* 按键连发的速度，0表示不支持连发 */
        p_touch_cfg->touch_button_cfg->p_stbtn[ i ].RepeatCount = 0;                   /* 连发计数器 */
    }

    /* 如果需要单独更改某个按键的参数，可以在此单独重新赋值 */
    /* 比如，我们希望按键1按下超过1秒后，自动重发相同键值 */
    //  s_tBtn[KID_K1].LongTime = 100;
    //  s_tBtn[KID_K1].RepeatSpeed = 5; /* 每隔50ms自动发送键值 */
}

/********************************************************************************************************
 * @fn      touch_InfoDebug
 * @brief   触摸数据打印函数
 * @param   无
 * @return  无
 */
void touch_InfoDebug( void )
{
    uint8_t  i;
    uint16_t data_dispNum[TKY_MAX_QUEUE_NUM ] = { 0 };
    dg_log( "\r\n" );
    if ( wakeupflag )
    {
        for ( i = 0; i < TKY_TOUCH_QUEUE_NUM; i++ )
        {
            TKY_GetCurQueueValue( i, &data_dispNum[ i ] );
        }
        for ( i = 0; i < TKY_TOUCH_QUEUE_NUM; i++ )
        {
            dg_log( "%04d,", data_dispNum[ i ] );
        }
        dg_log( "\n" );

        for ( i = 0; i < TKY_TOUCH_QUEUE_NUM; i++ )
        {
            TKY_GetCurQueueBaseLine( i, &data_dispNum[ i ] );
        }
        for ( i = 0; i < TKY_TOUCH_QUEUE_NUM; i++ )
        {
            dg_log( "%04d,", data_dispNum[ i ] );
        }
        dg_log( "\n" );

        for ( i = 0; i < TKY_TOUCH_QUEUE_NUM; i++ )
        {
            TKY_GetCurQueueRealVal( i, &data_dispNum[ i ] );
        }
        for ( i = 0; i < TKY_TOUCH_QUEUE_NUM; i++ )
        {
            dg_log( "%04d,", data_dispNum[ i ] );
        }
        dg_log( "\n" );

        uint16_t CTransN[ TKY_TOUCH_QUEUE_NUM ], CTransCC[ TKY_TOUCH_QUEUE_NUM ];
        for ( i = 0; i < TKY_TOUCH_QUEUE_NUM; i++ )
        {
            TKY_GetCurQueueChargeTime( i, &CTransN[ i ], &CTransCC[ i ] );
        }
        for ( i = 0; i < TKY_TOUCH_QUEUE_NUM; i++ )
        {
            dg_log( "%04d,", CTransN[ i ] );
        }
        dg_log( "\n" );
        for ( i = 0; i < TKY_TOUCH_QUEUE_NUM; i++ )
        {
            dg_log( "%04d,", CTransCC[ i ] );
        }
        dg_log( "\n" );

        for ( i = 0; i < TKY_TOUCH_QUEUE_NUM; i++ )
        {
            TKY_GetCurQueueNoiseThreshold( i, &data_dispNum[ i ] );
        }
        for ( i = 0; i < TKY_TOUCH_QUEUE_NUM; i++ )
        {
            dg_log( "%04d,", data_dispNum[ i ] );
        }
        dg_log( "\n" );

        uint8_t level[ TKY_TOUCH_QUEUE_NUM ];
        for ( i = 0; i < TKY_TOUCH_QUEUE_NUM; i++ )
        {
            TKY_GetCurQueueSensitivityLevel( i, &level[ i ] );
        }
        for ( i = 0; i < TKY_TOUCH_QUEUE_NUM; i++ )
        {
            dg_log( "%04x,", level[ i ] );
        }
        dg_log( "\n" );

        uint16_t th1[ TKY_TOUCH_QUEUE_NUM ], th2[ TKY_TOUCH_QUEUE_NUM ];
        for ( i = 0; i < TKY_TOUCH_QUEUE_NUM; i++ )
        {
            TKY_GetCurQueueThreshold( i, &th1[ i ], &th2[ i ] );
        }
        for ( i = 0; i < TKY_TOUCH_QUEUE_NUM; i++ )
        {
            dg_log( "%04d,", th1[ i ] );
        }
        dg_log( "\n" );
        for ( i = 0; i < TKY_TOUCH_QUEUE_NUM; i++ )
        {
            dg_log( "%04d,", th2[ i ] );
        }
        dg_log( "\n" );
    }
    else
    {
        for ( i = 0; i < TKY_SLEEP_QUEUE_NUM; i++ )
        {
            TKY_GetCurQueueValue( i+TKY_TOUCH_QUEUE_NUM, &data_dispNum[ i ] );
        }
        for ( i = 0; i < TKY_SLEEP_QUEUE_NUM; i++ )
        {
            dg_log( "%04u,", data_dispNum[ i ] );
        }
        dg_log( "\n" );

        for ( i = 0; i < TKY_SLEEP_QUEUE_NUM; i++ )
        {
            TKY_GetCurQueueBaseLine( i+TKY_TOUCH_QUEUE_NUM, &data_dispNum[ i ] );
        }
        for ( i = 0; i < TKY_SLEEP_QUEUE_NUM; i++ )
        {
            dg_log( "%04u,", data_dispNum[ i ] );
        }
        dg_log( "\n" );

        for ( i = 0; i < TKY_SLEEP_QUEUE_NUM; i++ )
        {
            TKY_GetCurQueueRealVal( i+TKY_TOUCH_QUEUE_NUM, &data_dispNum[ i ] );
        }
        for ( i = 0; i < TKY_SLEEP_QUEUE_NUM; i++ )
        {
            dg_log( "%04u,", data_dispNum[ i ] );
        }
        dg_log( "\n" );

        uint16_t CTransN[ TKY_SLEEP_QUEUE_NUM ], CTransCC[ TKY_SLEEP_QUEUE_NUM ];
        for ( i = 0; i < TKY_SLEEP_QUEUE_NUM; i++ )
        {
            TKY_GetCurQueueChargeTime( i+TKY_TOUCH_QUEUE_NUM, &CTransN[ i ], &CTransCC[ i ] );
        }
        for ( i = 0; i < TKY_SLEEP_QUEUE_NUM; i++ )
        {
            dg_log( "%04u,", CTransN[ i ] );
        }
        dg_log( "\n" );
        for ( i = 0; i < TKY_SLEEP_QUEUE_NUM; i++ )
        {
            dg_log( "%04u,", CTransCC[ i ] );
        }
        dg_log( "\n" );

        for ( i = 0; i < TKY_SLEEP_QUEUE_NUM; i++ )
        {
            TKY_GetCurQueueNoiseThreshold( i+TKY_TOUCH_QUEUE_NUM, &data_dispNum[ i ] );
        }
        for ( i = 0; i < TKY_SLEEP_QUEUE_NUM; i++ )
        {
            dg_log( "%04u,", data_dispNum[ i ] );
        }
        dg_log( "\n" );

        uint8_t level[ TKY_SLEEP_QUEUE_NUM ];
        for ( i = 0; i < TKY_SLEEP_QUEUE_NUM; i++ )
        {
            TKY_GetCurQueueSensitivityLevel( i+TKY_TOUCH_QUEUE_NUM, &level[ i ] );
        }
        for ( i = 0; i < TKY_SLEEP_QUEUE_NUM; i++ )
        {
            dg_log( "%04x,", level[ i ] );
        }
        dg_log( "\n" );

        uint16_t th1[ TKY_SLEEP_QUEUE_NUM ], th2[ TKY_SLEEP_QUEUE_NUM ];
        for ( i = 0; i < TKY_SLEEP_QUEUE_NUM; i++ )
        {
            TKY_GetCurQueueThreshold( i+TKY_TOUCH_QUEUE_NUM, &th1[ i ], &th2[ i ] );
        }
        for ( i = 0; i < TKY_SLEEP_QUEUE_NUM; i++ )
        {
            dg_log( "%04d,", th1[ i ] );
        }
        dg_log( "\n" );
        for ( i = 0; i < TKY_SLEEP_QUEUE_NUM; i++ )
        {
            dg_log( "%04d,", th2[ i ] );
        }
        dg_log( "\n" );
    }
    dg_log( "\r\n" );
}

/********************************************************************************************************
 * @fn      touch_DetectKey
 * @brief   检测一个按键。非阻塞状态，必须被周期性的调用。
 * @param   i - 按键结构变量指针
 * @return  无
 */
static void touch_DetectKey( touch_button_cfg_t* p )
{
    KEY_T* pBtn;

    if ( p == NULL )
    {
        return;
    }

    for ( uint8_t i = 0; i < p->num_elements; i++ )
    {
        /*按键按下*/
        pBtn = NULL;
        pBtn = &p_touch_cfg->touch_button_cfg->p_stbtn[ i ];
        if ( keyData & ( 1 << p->p_elem_index[ i ] ) )  // pBtn->IsKeyDownFunc()==1
        {
            /*短按键，软件处理消抖*/
            if(pBtn->Count < KEY_FILTER_TIME){
                pBtn->Count = KEY_FILTER_TIME;
            }
            else if(pBtn->Count < 2 * KEY_FILTER_TIME){
                pBtn->Count++;
            }
            else{
                if ( pBtn->State == 0 )
                {
                    pBtn->State = 1;
#if !KEY_MODE
                    /* 发送按钮按下的消息 */
                    touch_PutKey( (uint8_t)( 3 * i + 1 ) );
#endif
                }
            }

            /*处理长按键*/
            if ( pBtn->LongTime > 0 )
            {
                if ( pBtn->LongCount < pBtn->LongTime )
                {
                    /* 发送按钮长按下的消息 */
                    if ( ++pBtn->LongCount == pBtn->LongTime )
                    {
#if !KEY_MODE
                        pBtn->State = 2;

#ifdef KEY_PRESS_TIMEOUT_ENABLE
                        /* 长按无效按键异常，重置基线 */
                        touch_ResetBaselineWakeup();
#else
                        /* 键值放入按键FIFO */
                         touch_PutKey ((uint8_t)(3 * i + 3));
#endif
#endif
                    }
                }
                else
                {
                    if ( pBtn->RepeatSpeed > 0 )
                    {
                        if ( ++pBtn->RepeatCount >= pBtn->RepeatSpeed )
                        {
                            pBtn->RepeatCount = 0;
#if !KEY_MODE
                            /* 长按键后，每隔pBtn->RepeatSpeed*10ms发送1个按键 */
                            touch_PutKey( (uint8_t)( 3 * i + 1 ) );
#endif
                        }
                    }
                }
            }
        }
        else
        {
            if(pBtn->Count > KEY_FILTER_TIME)
            {
                pBtn->Count = KEY_FILTER_TIME;
            }
            else if(pBtn->Count != 0)
            {
                pBtn->Count--;
            }
            else{
                if ( pBtn->State )
                {
#if KEY_MODE
                    if ( pBtn->State == 1 )
                        /* 发送按钮按下的消息 */
                        touch_PutKey( (uint8_t)( 3 * i + 1 ) );
#endif
                    pBtn->State = 0;

#if !KEY_MODE
                    /* 松开按键KEY_FILTER_TIME后 发送按钮弹起的消息 */
                    touch_PutKey( (uint8_t)( 3 * i + 2 ) );
#endif
                }
            }

            pBtn->LongCount   = 0;
            pBtn->RepeatCount = 0;
        }
    }
}

static void touch_Regcfg( void )
{
    ADC_SignalPGATypeDef ga = ADC_PGA_0;
    R8_ADC_CFG      = RB_ADC_POWER_ON | RB_ADC_BUF_EN | ( (ga&0xF) << 4 ) | ( SampleFreq_8_or_4 << 6 );
    if( ga & ADC_PGA_2_ )
    {
        R8_ADC_CONVERT |= RB_ADC_PGA_GAIN2;
    }
    else
    {
        R8_ADC_CONVERT &= ~RB_ADC_PGA_GAIN2;
    }
    R8_ADC_CONVERT &= ~( RB_ADC_SAMPLE_TIME );
    R8_ADC_CONVERT |= RB_ADC_SAMPLE_TIME;
    R8_TKEY_CFG     = 0;
    // 配置外接电容Cx引脚IO
    R32_PIN_IN_DIS |= touch_pin_a;//关闭数字输入功能，降低功耗
    touch_IOWork();
    ADC_ChannelCfg( TKY_CX_CH_IDX ); // 配置接入外部参考电容Cx的通道
#if TKY_SHIELD_EN
    R8_TKEY_CFG  |= RB_TKEY_DRV_EN;
    R32_TKEY_SEL |= ( tkyChAll << 18 );
#endif
    TKY_SaveCfgReg();
}

/********************************************************************************************************
 * @fn      touch_Baseinit
 * @brief   触摸基础库初始化
 * @param   无
 * @return  无
 */
static void touch_Baseinit( void )
{
    uint8_t sta = 0xff;    
    TKY_BaseInitTypeDef TKY_BaseInitStructure = { 0 };

    // if ( !touch_enable_version2_mode() )
    // {
    //     PRINT( "chip version error...\n" );
    //     while ( 1 );
    // }

    if ( memcmp( TOUCH_VER_LIB, TOUCH_VER_FILE, strlen( TOUCH_VER_FILE ) ) )
    {
        PRINT( "head file error...\n" );
        while ( 1 );
    }


    for (uint8_t i = 0; i < TKY_MAX_QUEUE_NUM; i++)  // 初始化tkyQueueAll变量
    {
        tkyQueueAll |= 1 << i;
        if(i < TKY_TOUCH_QUEUE_NUM)
        {
            tkyChAll |= (1 << my_tky_ch_init[i].channelNum);
            touch_pin_a |= TKY_Pin[my_tky_ch_init[i].channelNum];
        }
        else
        {
            tkyChAll |= my_tky_ch_init[i].channelNum;
            for(uint32_t j = 0; j < 14; j++)
            {
                if(my_tky_ch_init[i].channelNum &(1<<j))
                {
                    touch_pin_a |= TKY_Pin[j];
                }
            }
            
        }

    } 
    touch_pin_a |= TKY_Pin[TKY_CX_CH_IDX];
    dg_log( "tQ : %08x;%04x;%04X\n", tkyQueueAll,tkyChAll,touch_pin_a );

    //----------触摸按键基础设置初始化--------
    TKY_BaseInitStructure.filterMode           = TKY_FILTER_MODE;
    TKY_BaseInitStructure.shieldEn             = TKY_SHIELD_EN;
    TKY_BaseInitStructure.singlePressMod       = TKY_SINGLE_PRESS_MODE;
    TKY_BaseInitStructure.filterGrade          = TKY_FILTER_GRADE;
    TKY_BaseInitStructure.maxQueueNum          = TKY_MAX_QUEUE_NUM;
    TKY_BaseInitStructure.maxNoiseChCount      = TKY_MAX_NOISE_CH_COUNT;
    TKY_BaseInitStructure.sleepChNum           = TKY_SLEEP_QUEUE_NUM;
    TKY_BaseInitStructure.baseRefreshOnPress   = TKY_BASE_REFRESH_ON_PRESS;
    //---基线更新速度，baseRefreshSampleNum和filterGrade，与基线更新速度成反比，基线更新速度还与代码结构相关，可通过函数GetCurQueueBaseLine来观察---
    TKY_BaseInitStructure.baseRefreshSampleNum = TKY_BASE_REFRESH_SAMPLE_NUM;
    TKY_BaseInitStructure.baseUpRefreshDouble  = TKY_BASE_UP_REFRESH_DOUBLE;
    TKY_BaseInitStructure.baseDownRefreshSlow  = TKY_BASE_DOWN_REFRESH_SLOW;
    TKY_BaseInitStructure.tkyBufP              = TKY_MEMBUF;
    sta                                        = TKY_BaseInit( TKY_BaseInitStructure );
    dg_log( "TKY_BaseInit:%02X\r\n", sta );
}

#define TKY_MAX_VOLTAGE 2100
#define TKY_DST_VOLTAGE 2100

#define TKY_MAX_FACTOR  70
#define TKY_MIN_FACTOR  50

static uint32_t maxCDParams = ( TKY_MAX_FACTOR * TKY_DST_VOLTAGE * 4096 ) / ( TKY_MAX_VOLTAGE * 100 );
static uint32_t minCDParams = ( TKY_MIN_FACTOR * TKY_DST_VOLTAGE * 4096 ) / ( TKY_MAX_VOLTAGE * 100 );

/********************************************************************************************************
 * @fn      touch_Channelinit
 * @brief   触摸通道初始化
 * @param   无
 * @return  无
 */
static void     touch_Channelinit( void )
{
#define SAMPLENUM 128
    uint8_t  error_flag = 0;
    uint16_t chx_mean = 0, chx_mean_last = 0;  //, noise = 0;
    uint16_t CTransN;
    uint16_t noise[ TKY_MAX_QUEUE_NUM ] = { 0 };
    uint16_t temp_base[TKY_MAX_QUEUE_NUM][4] = {0};
    uint16_t sampleData[ SAMPLENUM ];
    uint16_t testbit = 0;
    uint8_t  calflag = 0;

    // uint32_t maxCDParams = 0;
    // uint32_t minCDParams = 0;
    for ( uint8_t i = 0; i < TKY_MAX_QUEUE_NUM; i++ )
    {
        TKY_CHInit( my_tky_ch_init[ i ] );
    }
    TKY_LoadAndRun();
    dg_log( "Chbit :%x , minCDParams : %d, maxCDParams : %d\n", tkyChAll, minCDParams, maxCDParams );
    uint16_t th1[ TKY_MAX_QUEUE_NUM ], th2[ TKY_MAX_QUEUE_NUM ];
    for ( uint8_t i = 0; i < TKY_MAX_QUEUE_NUM; i++ )
    {
        TKY_GetCurQueueThreshold( i, &th1[ i ], &th2[ i ] );
    }
    for ( uint8_t i = 0; i < TKY_MAX_QUEUE_NUM; i++ )
    {
        dg_log( "%04d,", th1[ i ] );
    }
    dg_log( "\n" );
    for ( uint8_t i = 0; i < TKY_MAX_QUEUE_NUM; i++ )
    {
        dg_log( "%04d,", th2[ i ] );
    }
    dg_log( "\n" );

    for ( uint8_t j = 0; j < 4; j++ )
    {
        for ( uint8_t i = 0; i < TKY_MAX_QUEUE_NUM; i++ )
        {
            CTransN  = 1;
            chx_mean = 0;
            calflag  = 0;
            if ( i < TKY_TOUCH_QUEUE_NUM )
            {
                testbit     = 1 << my_tky_ch_init[ i ].channelNum;
//                maxCDParams = ( 70 * 4096 ) / 100;
//                minCDParams = ( 50 * 4096 ) / 100;
            }
            else
            {
                testbit     = my_tky_ch_init[ i ].channelNum;
//                maxCDParams = ( 50 * 4096 ) / 100;
//                minCDParams = ( 35 * 4096 ) / 100;
            }
            dg_log( "channel:%u, testbit:%x;\r\n", i, testbit );

            while ( 1 )
            {
basecal:
                TKY_GetCurChannelData( testbit, CTransN, CTransCC[ j ], sampleData, 5 );
                TKY_NoiseAndMeanEstimate( &sampleData[ 2 ], 3, NULL, &chx_mean );

                // dg_log( "testing CC:%u.... N : %u, baseline : %u\n", CTransCC[j],CTransN, chx_mean );  // 打印基线值

                if ( ( CTransN == 1 ) && ( ( chx_mean > maxCDParams ) ) )
                {  // 低于最小充电参数
                    dg_log( "Error, %u KEY%u Too large C base,Please check the hardware !\r\n", chx_mean, i );
                    calflag = 0;
                    break;
                }
                else
                {
                    // dg_log("0CTransN:%u\r\n",CTransN);
                    if ( ( chx_mean > minCDParams ) && ( chx_mean < maxCDParams ) )
                    {   // 充电参数正常
                        // dg_log("0calflag:%u\r\n",calflag);
                        if ( calflag == 0 )
                        {
                            calflag = 1;
                            if ( CTransN > 10 )
                            {
                                CTransN -= 10;
                            }
                            else
                            {
                                CTransN = 1;
                            }

                            goto basecal;
                        }
                        else if ( calflag == 1 )
                        {
                            calflag = 2;
                            break;
                        }
                    }
                    else if ( chx_mean >= maxCDParams )
                    {
                        // dg_log("1calflag:%u\r\n",calflag);
                        CTransN = CTransN - 1;
                        dg_log( "Warning,channel:%u Too large CC, chargetime:%u,BaseLine:%u\r\n",
                                i, CTransN, chx_mean );
                        if ( calflag == 0 )
                        {
                            calflag = 1;
                            if ( CTransN > 10 )
                            {
                                CTransN -= 10;
                            }
                            else
                            {
                                CTransN = 1;
                            }
                            goto basecal;
                        }
                        else if ( calflag == 1 )
                        {
                            calflag = 2;
                            break;
                        }
                    }
                    else
                    {
                        // dg_log("2calflag:%u\r\n",calflag);
                        if ( calflag == 0 )
                        {
                            CTransN += 10;
                            if ( CTransN > 1023 )
                            {  // 超出最大充电参数
                                CTransN -= 10;
                                calflag  = 1;
                                dg_log( "Error, Chargetime Max,KEY%u Too large CX,Please check the hardware !\r\n", i );
                            }
                        }
                        else if ( calflag == 1 )
                        {
                            CTransN++;
                            if ( CTransN > 1023 )
                            {  // 超出最大充电参数
                                calflag = 0;
                                dg_log( "Error, Chargetime Max,KEY%u Too large CX,Please check the hardware !\r\n", i );
                                break;
                            }
                        }
                    }
                }
            }
            if ( calflag == 2 )
            {
                uint16_t temp = 0;
                TKY_GetCurChannelData( testbit, CTransN, CTransCC[ j ], sampleData, SAMPLENUM );  //
                TKY_NoiseAndMeanEstimate( sampleData, SAMPLENUM, &noise[ i ], &chx_mean );

                dg_log( "SET ch %u;testing CC:%u.... N : %u, baseline : %u, noise : %u\n", i, CTransCC[ j ], CTransN, chx_mean, noise[ i ] );
                temp_base[i][j] = chx_mean;
                // TKY_SetGroupCurQueueBaseLine( i, j, chx_mean );
                TKY_SetGroupCurQueueChargeTime( i, j, CTransN, CTransCC[ j ] );


                // TKY_GetGroupCurQueueBaseLine( i, j, &chx_mean );
                // dg_log( "GET ch %u;group:%u.... baseline : %u\n", i, j, chx_mean );
                TKY_GetGroupCurQueueChargeTime( i, j, &CTransN, &chx_mean );
                dg_log( "GET ch %u;group:%u.... CTransCC:%u, CTransN : %u\n", i, j, chx_mean, CTransN );
                TKY_GetCurQueueNoiseThreshold( i, &temp );
                dg_log( "GET ch %u;noise : %u\n", i, temp );
            }
            else
            {
                dg_log( "CH %u Cal Fail!\n", i );
            }
        }
        TKY_SetCurGroupNoiseThreshold( j, noise, TKY_MAX_QUEUE_NUM );
    }
    uint16_t cc = 4;
    TKY_GetCurQueueChargeTime(0,NULL,&cc);
    for ( uint8_t j = 0; j < 4; j++ )
    {
        if ( cc == CTransCC[ j ] )
        {
            cc = j;
            break;
        }
    }
    if ( cc > 3 ) { cc = 0; } /* CC组匹配失败时回退组0，防止temp_base[m][cc]数组越界 */
    // 设置灵敏度
    for ( uint8_t m = 0; m < TKY_MAX_QUEUE_NUM; m++ )
    {
        TKY_SetCurQueueBaseLine(m,temp_base[m][cc]);
        TKY_SetCurQueueSensitivityLevel( m, my_tky_ch_init[ m ].sensitivityLevel );
    }

    for ( uint8_t m = 0; m < TKY_MAX_QUEUE_NUM; m++ )
    {
        uint8_t level = 0;uint16_t base = 0;
        TKY_GetCurQueueBaseLine(m,&base);
        TKY_GetCurQueueSensitivityLevel( m, &level );
        PRINT( "ch:%u;level:%u;baseline:%u\r\n", m, level,base );
    }
    TKY_SaveAndStop();
}

/********************************************************************************************************
 * @fn      touch_Recalibrate_Sleep
 * @brief   触摸参数重新校准
 * @param   无
 * @return  无
 */
void touch_Recalibrate_Sleep( void )
{
    uint16_t chx_mean = 0, chx_mean_last = 0;
    uint8_t  recal_flag1 = 0, recal_flag2 = 0;
    uint16_t sampleData[ SAMPLENUM ] = { 0 };
    uint16_t CTransN = 1;
    uint16_t testbit = 0;
    uint8_t  calflag = 0;
    uint16_t noise[ TKY_MAX_QUEUE_NUM ] = { 0 };

    {
        uint8_t j;

        //    for ( j = 0; j < TKY_MAX_QUEUE_NUM; j++)
        //    {
        //        dg_log("realval %d %d %d - %d %d\n",j,tky_baseval[j],tky_realval[j],tky_th1[j],tky_th2[j]);
        //    }

        for ( j = TKY_SLEEP_QUEUE_NUM; j < TKY_MAX_QUEUE_NUM; j++)
        {
            TKY_GetCurQueueRealVal( j, &tky_realval[j]);
            TKY_GetCurQueueThreshold( j, &tky_th1[j], &tky_th2[j]);
            TKY_GetCurQueueBaseLine( j, &tky_baseval[j]);
            if(tky_realval[j] < tky_baseval[j])
            {
                if(tky_baseval[j] -  tky_realval[j]> tky_th1[j])
                {
                    tky_baseval[j] = 0;
                    recal_flag1 = 1;

                    dg_log("err channel : %d, %04x\n",j ,my_tky_ch_init[j].channelNum);
                    for(uint8_t k = 0; k < TKY_TOUCH_QUEUE_NUM; k++)
                    {
                        if(my_tky_ch_init[j].channelNum & (1 << my_tky_ch_init[k].channelNum))
                        {
                            tky_baseval[k] = 0;
                            PRINT("sub channel : %d, %04x\n",k, 1 << my_tky_ch_init[k].channelNum);
                        }
                    }

                }
            }
            else if(tky_realval[j] - tky_baseval[j] > (tky_th1[j] >> 1))
            {
                recal_flag2++;
    //            tky_baseval[j] = 0;
            }
        }
    }

    dg_log("***recal_flag1 : %d, recal_flag2 : %d\n",recal_flag1, recal_flag2);

    if((1 == recal_flag1) || (recal_flag2 > (TKY_SLEEP_QUEUE_NUM >> 1)))
    {
        touch_ResetBaselineSleep();
    }
}

void touch_ResetBaselineWakeup (void) {
    uint16_t var = 0;
    TKY_LoadAndRun();
    TKY_PollForFilter();
    TKY_SaveAndStop();
    for (uint8_t i = 0; i < TKY_TOUCH_QUEUE_NUM; i++) {
        TKY_GetCurQueueFilterVal (i, &var);
        TKY_SetCurQueueBaseLine (i,var);
    }
    TKY_ClearHistoryData(TKY_FILTER_MODE);
}

void touch_ResetBaselineSleep (void) {
    uint16_t var = 0;
    TKY_LoadAndRun();
    TKY_ScanForWakeUp();
    TKY_SaveAndStop();
    for (uint8_t i = TKY_TOUCH_QUEUE_NUM; i < TKY_MAX_QUEUE_NUM; i++) {
        TKY_GetCurQueueRealVal (i, &var);
        TKY_SetCurQueueBaseLine (i, var);
    }
    TKY_ClearHistoryData(TKY_FILTER_MODE);
}
void touch_ResetBaseline( void )
{
    uint16_t var = 0;
    TKY_LoadAndRun();
    TKY_PollForFilter();
    TKY_SaveAndStop();
    for ( uint8_t i = 0; i < TKY_MAX_QUEUE_NUM; i++ )
    {
        TKY_GetCurQueueFilterVal( i, &var);
        TKY_SetCurQueueBaseLine( i, var );
    }
    TKY_ClearHistoryData( TKY_FILTER_MODE );
}

/********************************************************************************************************
 * @fn      touch_DetectWheelSlider
 * @brief   触摸滑轮数据处理
 * @param   无
 * @return  无
 */
static uint16_t touch_DetectWheelSlider( touch_wheel_cfg_t* p_wheel )
{
    uint8_t   loop;
    uint8_t   max_data_idx;
    uint16_t  d1;
    uint16_t  d2;
    uint16_t  d3;
    uint16_t  wheel_rpos;
    uint16_t  dsum;
    uint16_t  unit;
    uint8_t   num_elements;
    uint16_t  p_threshold;
    uint16_t* wheel_data;

    if ( p_wheel == NULL )
    {
        return TOUCH_OFF_VALUE;
    }

    num_elements = p_wheel->num_elements;
    p_threshold  = p_wheel->threshold;
    wheel_data   = p_wheel->pdata;

    if ( num_elements < 3 )
    {
        return TOUCH_OFF_VALUE;
    }

    for ( loop = 0; loop < p_wheel->num_elements; loop++ )
    {
        TKY_GetCurQueueValue( p_wheel->p_elem_index[ loop ], &wheel_data[ loop ] );
    }

    /* Search max data in slider */
    max_data_idx = 0;
    for ( loop = 0; loop < ( num_elements - 1 ); loop++ )
    {
        if ( wheel_data[ max_data_idx ] < wheel_data[ loop + 1 ] )
        {
            max_data_idx = (uint8_t)( loop + 1 );
        }
    }
    /* Array making for wheel operation          */
    /*    Maximum change CH_No -----> Array"0"    */
    /*    Maximum change CH_No + 1 -> Array"2"    */
    /*    Maximum change CH_No - 1 -> Array"1"    */
    if ( 0 == max_data_idx )
    {
        d1   = (uint16_t)( wheel_data[ 0 ] - wheel_data[ num_elements - 1 ] );
        d2   = (uint16_t)( wheel_data[ 0 ] - wheel_data[ 1 ] );
        dsum = (uint16_t)( wheel_data[ 0 ] + wheel_data[ 1 ] + wheel_data[ num_elements - 1 ] );
    }
    else if ( ( num_elements - 1 ) == max_data_idx )
    {
        d1   = (uint16_t)( wheel_data[ num_elements - 1 ] - wheel_data[ num_elements - 2 ] );
        d2   = (uint16_t)( wheel_data[ num_elements - 1 ] - wheel_data[ 0 ] );
        dsum = (uint16_t)( wheel_data[ 0 ] + wheel_data[ num_elements - 2 ] + wheel_data[ num_elements - 1 ] );
    }
    else
    {
        d1   = (uint16_t)( wheel_data[ max_data_idx ] - wheel_data[ max_data_idx - 1 ] );
        d2   = (uint16_t)( wheel_data[ max_data_idx ] - wheel_data[ max_data_idx + 1 ] );
        dsum = (uint16_t)( wheel_data[ max_data_idx + 1 ] + wheel_data[ max_data_idx ] + wheel_data[ max_data_idx - 1 ] );
    }

    if ( 0 == d1 )
    {
        d1 = 1;
    }
    /* Constant decision for operation of angle of wheel */
    if ( dsum > p_threshold )
    {
        d3         = (uint16_t)( p_wheel->decimal_point_percision + ( ( d2 * p_wheel->decimal_point_percision ) / d1 ) );

        unit       = (uint16_t)( p_wheel->wheel_resolution / num_elements );
        wheel_rpos = (uint16_t)( ( ( unit * p_wheel->decimal_point_percision ) / d3 ) + ( unit * max_data_idx ) );

        /* Angle division output */
        /* diff_angle_ch = 0 -> 359 ------ diff_angle_ch output 1 to 360 */
        if ( 0 == wheel_rpos )
        {
            wheel_rpos = p_wheel->wheel_resolution;
        }
        else if ( ( p_wheel->wheel_resolution + 1 ) < wheel_rpos )
        {
            wheel_rpos = 1;
        }
        else
        {
            /* Do Nothing */
        }
    }
    else
    {
        wheel_rpos = TOUCH_OFF_VALUE;
    }

    return wheel_rpos;
}

/********************************************************************************************************
 * @fn      touch_DetectWheelSlider
 * @brief   触摸滑条数据处理
 * @param   无
 * @return  滑条坐标
 */
static uint16_t touch_DetecLineSlider( touch_slider_cfg_t* p_slider )
{

    uint8_t   loop;
    uint8_t   max_data_idx;
    uint16_t  d1;
    uint16_t  d2;
    uint16_t  d3;
    uint16_t  slider_rpos;
    uint16_t  resol_plus;
    uint16_t  dsum;
    uint8_t   num_elements = 0;
    uint16_t  p_threshold  = 0;
    uint16_t* slider_data  = 0;

    if ( p_slider == NULL )
    {
        return TOUCH_OFF_VALUE;
    }

    num_elements = p_slider->num_elements;
    p_threshold  = p_slider->threshold;
    slider_data  = p_slider->pdata;

    if ( num_elements < 3 )
    {
        return TOUCH_OFF_VALUE;
    }

    for ( uint8_t loop = 0; loop < num_elements; loop++ )
    {
        TKY_GetCurQueueValue( p_slider->p_elem_index[ loop ], &slider_data[ loop ] );
    }
    /* Search max data in slider */
    max_data_idx = 0;
    for ( loop = 0; loop < ( num_elements - 1 ); loop++ )
    {
        if ( slider_data[ max_data_idx ] < slider_data[ loop + 1 ] )
        {
            max_data_idx = (uint8_t)( loop + 1 );
        }
    }

    /* Array making for slider operation-------------*/
    /*     |    Maximum change CH_No -----> Array"0"    */
    /*     |    Maximum change CH_No + 1 -> Array"2"    */
    /*     |    Maximum change CH_No - 1 -> Array"1"    */
#if 0
    if (0 == max_data_idx)
    {
        d1 = (uint16_t)(slider_data[0] - slider_data[2]);
        d2 = (uint16_t)(slider_data[0] - slider_data[1]);
    }
    else if ((num_elements - 1) == max_data_idx)
    {
        d1 = (uint16_t)(slider_data[num_elements - 1] - slider_data[num_elements - 2]);
        d2 = (uint16_t)(slider_data[num_elements - 1] - slider_data[num_elements - 3]);
    }
    else
    {
        d1 = (uint16_t)(slider_data[max_data_idx] - slider_data[max_data_idx - 1]);
        d2 = (uint16_t)(slider_data[max_data_idx] - slider_data[max_data_idx + 1]);
    }

    dsum = (uint16_t)(d1 + d2);

    /* Constant decision for operation of angle of slider */
    /* Scale results to be 0-TOUCH_SLIDER_RESOLUTION */
    if (dsum > p_threshold)
    {
        if (0 == d1)
        {
            d1 = 1;
        }

        /* x : y = d1 : d2 */
        d3 = (uint16_t)(p_slider->decimal_point_percision + ((d2 * p_slider->decimal_point_percision) / d1));

        slider_rpos = (uint16_t)(((p_slider->decimal_point_percision * p_slider->slider_resolution) / d3) + (p_slider->slider_resolution * max_data_idx));

        resol_plus = (uint16_t)(p_slider->slider_resolution * (num_elements - 1));

        if (0 == slider_rpos)
        {
            slider_rpos = 1;
        }
        else if (slider_rpos >= resol_plus)
        {
            slider_rpos = (uint16_t)(((slider_rpos - resol_plus) * 2) + resol_plus);
            if (slider_rpos > (p_slider->slider_resolution * num_elements))
            {
                slider_rpos = p_slider->slider_resolution;
            }
            else
            {
                slider_rpos = (uint16_t)(slider_rpos / num_elements);
            }
        }
        else if (slider_rpos <= p_slider->slider_resolution)
        {
            if (slider_rpos < (p_slider->slider_resolution / 2))
            {
                slider_rpos = 1;
            }
            else
            {
                slider_rpos = (uint16_t)(slider_rpos - (p_slider->slider_resolution / 2));
                if (0 == slider_rpos)
                {
                    slider_rpos = 1;
                }
                else
                {
                    slider_rpos = (uint16_t)((slider_rpos * 2) / num_elements);
                }
            }
        }
        else
        {
            slider_rpos = (uint16_t)(slider_rpos / num_elements);
        }
    }
    else
    {
        slider_rpos = TOUCH_OFF_VALUE;
    }

#else
    // int16_t dval;
    uint16_t unit;

    if ( 0 == max_data_idx )
    {
        d1   = (uint16_t)( slider_data[ 0 ] - slider_data[ num_elements - 1 ] );
        d2   = (uint16_t)( slider_data[ 0 ] - slider_data[ 1 ] );
        dsum = (uint16_t)( slider_data[ 0 ] + slider_data[ 1 ] + slider_data[ num_elements - 1 ] );
    }
    else if ( ( num_elements - 1 ) == max_data_idx )
    {
        d1   = (uint16_t)( slider_data[ num_elements - 1 ] - slider_data[ num_elements - 2 ] );
        d2   = (uint16_t)( slider_data[ num_elements - 1 ] - slider_data[ 0 ] );
        dsum = (uint16_t)( slider_data[ 0 ] + slider_data[ num_elements - 2 ] + slider_data[ num_elements - 1 ] );
    }
    else
    {
        d1   = (uint16_t)( slider_data[ max_data_idx ] - slider_data[ max_data_idx - 1 ] );
        d2   = (uint16_t)( slider_data[ max_data_idx ] - slider_data[ max_data_idx + 1 ] );
        dsum = (uint16_t)( slider_data[ max_data_idx + 1 ] + slider_data[ max_data_idx ] + slider_data[ max_data_idx - 1 ] );
    }

    if ( 0 == d1 )
    {
        d1 = 1;
    }
    /* Constant decision for operation of angle of wheel    */
    if ( dsum > p_threshold )
    {
        d3          = (uint16_t)( p_slider->decimal_point_percision + ( ( d2 * p_slider->decimal_point_percision ) / d1 ) );

        unit        = (uint16_t)( p_slider->slider_resolution / num_elements );
        slider_rpos = (uint16_t)( ( ( unit * p_slider->decimal_point_percision ) / d3 ) + ( unit * max_data_idx ) );

        /* Angle division output */
        /* diff_angle_ch = 0 -> 359 ------ diff_angle_ch output 1 to 360 */
        if ( 0 == slider_rpos )
        {
            slider_rpos = p_slider->slider_resolution;
        }
        else if ( ( p_slider->slider_resolution + 1 ) < slider_rpos )
        {
            slider_rpos = 1;
        }
        else
        {
            /* Do Nothing */
        }
    }
    else
    {
        slider_rpos = TOUCH_OFF_VALUE;
    }
#endif
    return slider_rpos;
}

uint16_t touch_GetLineSliderData( void )
{
    return SilderData;
}

uint16_t touch_GetWheelSliderData( void )
{
    return WheelData;
}

static uint8_t touch_enable_version2_mode( void )
{
    uint8_t chip_version, chip_version_auth;

    // 0x00FFFFF0 | (d << 24) | ChipID
    // chip_version      = ( R32_ROM_CFG_ADR_VER >> 24 ) & 0xff;
    chip_version_auth = chip_version >> 4;
    chip_version_auth = ( ~( chip_version_auth ) ) & 0x0f;
    chip_version      = chip_version & 0x0f;
    if ( chip_version == chip_version_auth )
    {
        if ( chip_version > 0 )
        {
            return 1;
        }
    }

    return 0;
}
