/********************************** (C) COPYRIGHT *******************************
 * File Name          : app_tmos.C
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2023/8/5
 * Description        : TouchKey application
 *******************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "Touch.h"
#include "touchkey.h"
/*********************
 *      DEFINES
 *********************/

/**********************
 *      VARIABLES
 **********************/
UINT8V timerFlag = 0;

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void TKY_PeripheralInit(void);

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

/*********************************************************************
 * @fn      touch_init
 *
 * @brief   TouchKey initialization
 *
 * @return  none
 */
void touch_init(void)
{
    /* Initialize peripherals, such as backlight and buzzer, etc */
	TKY_PeripheralInit();

	/* Touch library initialization */
	touch_InitKey();

    TKY_SetSleepStatusValue( ~tkyQueueAll );

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
}
