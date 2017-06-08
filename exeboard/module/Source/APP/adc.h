#ifndef _ADC_MEASUREMENT_H_
#define _ADC_MEASUREMENT_H_

typedef void (*multiplex_cb)(void *para);

#define CM_MAX_NUMBER (6)

typedef enum
{
    ADC_MESSAGE_SESSION_CMP = 0,
    ADC_MESSAGE_TIMER_EVENT,
        
}ADC_MESSAGE_ENUM;

UINT8 PidAdcProcess(Message *pMsg);

void ADC_Meas_Init(u32 Sampling_Period );

void AdcDmaHandler(void);

void ADC_SetMultiplexer ( uint8_t ucChl, multiplex_cb cb , void *para);

uint32_t GetAdcData(uint8_t ucChl);

#endif
