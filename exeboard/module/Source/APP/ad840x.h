#ifndef _HAL_AD840X_H_
#define _HAL_AD840X_H_

#include "stm32_eval.h"

#define AD840X_MAX_NUM (2)

void AD840x_Init(void);
void AD840x_Write(uint8_t ucChl,uint8_t ucValue);
uint8_t AD840x_BufferedRead(uint8_t ucChl);

#endif
