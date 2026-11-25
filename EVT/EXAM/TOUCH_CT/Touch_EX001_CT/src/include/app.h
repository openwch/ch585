#ifndef APP_H_
#define APP_H_

#include "CH58x_common.h"

#define TOUCH_DECIMAL_POINT_PRECISION       (100)

/************************TOUCH_KEY_DEFINE****************************/
#define TOUCH_KEY_ELEMENTS            	    (2)
#define TOUCH_KEY_CHS                       0,1

/************************WHEEL_SLIDER_DEFINE****************************/
#define TOUCH_WHEEL_ELEMENTS            	(3)
#define TOUCH_WHEEL_RESOLUTION              (120)
#define TOUCH_WHEEL_CHS                     2,3,4

/************************LINE_SLIDER_DEFINE****************************/
#define TOUCH_SLIDER_ELEMENTS            	(3)
#define TOUCH_SLIDER_RESOLUTION             (120)
#define TOUCH_SLIDER_CHS                    5,6,7



extern volatile uint8_t timerFlag;

void TKY_Init(void);
void TKY_dataProcess(void);

#endif
