/********************************** (C) COPYRIGHT *******************************
* File Name          : RingMem.h
* Author             : WCH
* Version            : V1.0
* Date               : 2025/04/28
* Description        : 
            
*******************************************************************************/

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */

#ifndef RINGMEM_H
#define RINGMEM_H

#ifndef SUCCESS
#define SUCCESS	0
#endif

/*********************************************************************
 * TYPEDEFS
 */
 
typedef unsigned char   uint8_t;
typedef unsigned short  uint16_t;
typedef unsigned long   uint32_t;

//缓冲区结构体
typedef struct 
{
         uint8_t   *pData;		    //缓冲区首地址
         uint8_t 	 *pWrite;           //写指针
         uint8_t 	 *pRead;		    //读指针
         uint8_t   *pEnd;			    //缓冲区末地址
volatile uint32_t  RemanentLen;       //剩余空间大小
volatile uint32_t  CurrentLen;        //已用空间大小
volatile uint32_t  MaxLen;      	    //总空间大小
}RingMemParm;

/*********************************************************************
 * Global Variables
 */

/*********************************************************************
 * FUNCTIONS
 */

extern void RingMemInit( RingMemParm *Parm, uint8_t *StartAddr, uint32_t MaxLen );

extern uint8_t RingMemWrite( RingMemParm *Parm, uint8_t *pData, uint32_t len );

extern uint8_t RingMemRead( RingMemParm *Parm, uint8_t *pData, uint32_t len );

extern uint8_t RingMemCopy( RingMemParm *Parm, uint8_t *pData, uint32_t len );

extern uint8_t RingMemDelete( RingMemParm *Parm, uint32_t len );

extern uint8_t RingAddInStart( RingMemParm *Parm, uint8_t *pData, uint32_t len );

extern uint8_t RingReturnSingleData( RingMemParm *Parm, uint32_t Num );

extern int __wrap_memcmp (void *src, const void * dst, size_t size);

extern void * __wrap_memset (void *src, int value, size_t size);
/*********************************************************************
*********************************************************************/

#endif
