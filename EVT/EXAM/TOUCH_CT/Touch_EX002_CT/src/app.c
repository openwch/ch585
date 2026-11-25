/********************************** (C) COPYRIGHT *******************************
 * File Name          : app.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2023/8/5
 * Description        : 触摸按键例程
 *******************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "Touch.h"
#include "app.h"
#include "backlight.h"
/*********************
 *      DEFINES
 *********************/

/**********************
 *      VARIABLES
 **********************/
uint8_t volatile timerFlag = 0;
volatile uint16_t recal_time = 0;

static const uint8_t touch_key_ch[ TOUCH_KEY_ELEMENTS ] = {TOUCH_KEY_CHS};
KEY_T s_tBtn[TOUCH_KEY_ELEMENTS] = {0};

touch_button_cfg_t p_selfkey = 
{
    .num_elements = TOUCH_KEY_ELEMENTS,
    .p_elem_index = touch_key_ch,
    .p_stbtn = s_tBtn
};

touch_cfg_t touch_cfg = 
{
    .touch_button_cfg = &p_selfkey,
    .touch_slider_cfg = NULL,
    .touch_wheel_cfg = NULL
};
/**********************
 *  STATIC PROTOTYPES
 **********************/
static void TKY_PeripheralInit(void);

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
/*********************************************************************
 * @fn      TKY_dataProcess
 *
 * @brief   触摸数据处理函数（裸跑），打印获取到的按键触发情况
 *
 * @return  none
 */
void TKY_dataProcess(void)
{
    uint8_t key_val = 0;

    if (timerFlag) {
        timerFlag = 0;
        touch_Scan();
    }
    if (recal_time >= 500) {
#if PRINT_EN
        touch_InfoDebug();
#endif
        recal_time = 0;
    }
    key_val = touch_GetKey();
    switch(key_val)
    {
       case KEY_NONE   :   break;
       case KEY_0_DOWN :   TKY_KeyBacklightOut(0,!TKY_GetBacklightState(0));  PRINT("KEY_1_DOWN !\n");break;
       case KEY_0_LONG :   PRINT("KEY_1_LONG !\n");break;
       case KEY_0_UP   :   TKY_KeyBacklightOut(0,DISABLE); PRINT("KEY_1_UP   !\n");break;
       case KEY_1_DOWN :   TKY_KeyBacklightOut(1,!TKY_GetBacklightState(1));  PRINT("KEY_2_DOWN !\n");break;
       case KEY_1_LONG :   PRINT("KEY_2_LONG !\n");break;
       case KEY_1_UP   :   TKY_KeyBacklightOut(1,DISABLE); PRINT("KEY_2_UP   !\n");break;
       case KEY_2_DOWN :   TKY_KeyBacklightOut(2,!TKY_GetBacklightState(2));  PRINT("KEY_3_DOWN !\n");break;
       case KEY_2_LONG :   PRINT("KEY_3_LONG !\n");break;
       case KEY_2_UP   :   TKY_KeyBacklightOut(2,DISABLE); PRINT("KEY_3_UP   !\n");break;
       case KEY_3_DOWN :   TKY_KeyBacklightOut(3,!TKY_GetBacklightState(3));  PRINT("KEY_4_DOWN !\n");break;
       case KEY_3_LONG :   PRINT("KEY_4_LONG !\n");break;
       case KEY_3_UP   :   TKY_KeyBacklightOut(3,DISABLE); PRINT("KEY_4_UP   !\n");break;
       case KEY_4_DOWN :   TKY_KeyBacklightOut(4,!TKY_GetBacklightState(4));  PRINT("KEY_5_DOWN !\n");break;
       case KEY_4_LONG :   PRINT("KEY_5_LONG !\n");break;
       case KEY_4_UP   :   TKY_KeyBacklightOut(4,DISABLE); PRINT("KEY_5_UP   !\n");break;
       case KEY_5_DOWN :   TKY_KeyBacklightOut(5,!TKY_GetBacklightState(5));  PRINT("KEY_6_DOWN !\n");break;
       case KEY_5_LONG :   PRINT("KEY_6_LONG !\n");break;
       case KEY_5_UP   :   TKY_KeyBacklightOut(5,DISABLE); PRINT("KEY_6_UP   !\n");break;
       case KEY_6_DOWN :   TKY_KeyBacklightOut(6,!TKY_GetBacklightState(6));  PRINT("KEY_7_DOWN !\n");break;
       case KEY_6_LONG :   PRINT("KEY_7_LONG !\n");break;
       case KEY_6_UP   :   TKY_KeyBacklightOut(6,DISABLE); PRINT("KEY_7_UP   !\n");break;
       case KEY_7_DOWN :   TKY_KeyBacklightOut(7,!TKY_GetBacklightState(7));  PRINT("KEY_8_DOWN !\n");break;
       case KEY_7_LONG :   PRINT("KEY_8_LONG !\n");break;
       case KEY_7_UP   :   TKY_KeyBacklightOut(7,DISABLE); PRINT("KEY_8_UP   !\n");break;
       default : break;
    }
}

/*********************************************************************
 * @fn      Timer_Init
 *
 * @brief   Timer Init function
 *
 * @param   none
 *
 * @return  none
 */
void Timer_Init (void) {
    TMR0_TimerInit(FREQ_SYS/1000);               //定时周期为1ms
    TMR0_ITCfg(ENABLE, TMR0_3_IT_CYC_END);
    PFIC_EnableIRQ( TMR0_IRQn );
}

/*********************************************************************
 * @fn      TKY_Init
 *
 * @brief   触摸初始化函数（不使用tmos，需要设备开启定时器）
 *
 * @return  none
 */
void TKY_Init(void)
{
	TKY_PeripheralInit();       /* 初始化外设，例如背光和蜂鸣器等 */

	touch_Init(&touch_cfg);				/* 初始化触摸库  */

    Timer_Init();

    for(int i=0;i<40;i++){
        TKY_LoadAndRun();
        TKY_PollForFilter();
        TKY_SaveAndStop();
    }
    touch_ScanWakeUp();

    dg_log("Touch Key init Finish!\n");
}


/**********************
 *   STATIC FUNCTIONS
 **********************/

/*********************************************************************
 * @fn      TKY_PeripheralInit
 *
 * @brief   触摸相关外设初始化函数
 *
 * @return  none
 */
static void TKY_PeripheralInit(void)
{
    /*You code here*/
    TKY_BacklightInit();
    TKY_BeepInit();
}

/*********************************************************************
 * @fn      TMR0_IRQHandler
 *
 * @brief   定时器0中断服务函数
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
        timerFlag=1;
        recal_time++;
        if (led_scanflag)
        {
            /*led scan*/
            TKY_BacklightProcess();          // poll背光状态
        }
    }
}
