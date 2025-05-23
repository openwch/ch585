#ifndef APP_H_
#define APP_H_

#include "CH58x_common.h"

/************************TOUCH_KEY_DEFINE****************************/
#define TOUCH_KEY_ELEMENTS            	    (TKY_MAX_QUEUE_NUM)
#define TOUCH_KEY_CHS                       0,1,2,3,4,5,6,7

extern volatile uint8_t timerFlag;

void TKY_Init(void);
void TKY_dataProcess(void);

#endif
