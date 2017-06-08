#ifndef _RELAY_H_
#define _RELAY_H_

#include "stm32_eval.h"

#include "DtypeStm32.h"

typedef enum
{
   STM32_RFID_ANT1_EN       = 0,  // 紫外灯开启信号，高有效，默认关闭
   STM32_RFID_ANT2_EN,
   STM32_RFID_ANT3_EN,
   STM32_RC663_PDOWN,
   STM32_PCA9502_nRST,
   RELAY_NUMBER,
}RELAY_ENUM;

typedef void (*RelayPulse_Cb)(void);


void InitRelays(void);
void RelayLogicCtrl(UINT8 ucChannel,UINT8 ucEnable);
UINT8 GetRelayLogicStatus(UINT8 ucChannel);
void RelayToggle(UINT8 ucChannel);
void RelayPulse(UINT8 ucChannel,uint32_t duration,RelayPulse_Cb cb);

#endif
