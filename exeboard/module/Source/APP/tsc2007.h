#ifndef _HAL_TSC2007_H_
#define _HAL_TSC2007_H_

#include "i2c.h"

/*****************************************************************************
 *  Global Macros & Definitions
 *****************************************************************************/

typedef enum
{
    TSC2007_MESSAGE_IRQ = 0,
    TSC2007_MESSAGE_DELAY_CHECK,
}TSC2007_MESSAGE_ENUM;

UINT8 TSC2007_ItfProcess(Message *pMsg);

void TSC2007_Init(void);

#endif
