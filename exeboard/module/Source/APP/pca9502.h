#ifndef _HAL_PCA9502_H_
#define _HAL_PCA9502_H_

#include "stm32_eval.h"

#define PAC9502_MAX_NUM (3)

#define PCA9502_REG_NUM (5)

typedef enum
{
   PCA9502_REG_IODir      = 0XA,
   PCA9502_REG_IOState    = 0xb,
   PCA9502_REG_IOIntEna   = 0Xc,
   PCA9502_REG_RSVD       = 0XD,
   PCA9502_REG_IOControl  = 0XE,
}PCA9502_REG_NAME_ENUM;

#define PCA9502_READ  (0X80)

void PCA9502_Init(void);
void PCA9502_SetGpio(uint8_t ucChl,uint8_t ucGpio,uint8_t ucValue);
uint8_t PCA9502_GetGpio(uint8_t ucChl,uint8_t ucGpio);
void PCA9502_UpdateGpios(uint8_t ucChl,uint8_t ucMask,uint8_t ucValue);
uint8_t PCA9502_GetBuffedGpio(uint8_t ucChl,uint8_t ucGpio);

#endif
