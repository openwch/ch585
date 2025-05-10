#ifndef APP_TMOS_H_
#define APP_TMOS_H_

#include "CH58x_common.h"

#define DEALDATA_EVT         0x0001
#define WAKEUP_DATA_DEAL_EVT 0x0002
#define DEBUG_PRINT_EVENT    0x0004
#define TKY_KEEPALIVE_EVENT  0x0008

/************************TOUCH_KEY_DEFINE****************************/
#define TOUCH_KEY_ELEMENTS                  (KEY_COUNT)
#define TOUCH_KEY_CHS                       {0x0101,0x0201,0x0401,0x0801,0x1001,0x2001,0x2010,0x1010,0x0810,0x0410,0x0210,0x0110,    \
                                             0x0102,0x0202,0x0402,0x0802,0x1002,0x2002,0x2020,0x1020,0x0820,0x0420,0x0220,0x0120,    \
                                             0x0104,0x0204,0x0404,0x0804,0x1004,0x2004,0x2040,0x1040,0x0840,0x0440,0x0240,0x0140,    \
                                             0x0108,0x0208,0x0408,0x0808,0x1008,0x2008,0x2080,0x1080,0x0880,0x0480,0x0280,0x0180}

void PeriodicDealData(void);
void touch_on_TMOS_init(void);
void tky_on_TMOS_dataProcess(void);
void tky_DealData_stop(void);
void tky_DealData_start(void);

#endif
