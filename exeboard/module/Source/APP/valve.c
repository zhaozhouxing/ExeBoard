/*! @file radio.c
 * @brief This file contains functions to interface with the radio chip.
 *
 * @b COPYRIGHT
 * @n Silicon Laboratories Confidential
 * @n Copyright 2012 Silicon Laboratories, Inc.
 * @n http://www.silabs.com
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "multiplex.h"

#include "task.h"

#include "ctype.h"

#include "sys_time.h "

#include "adc.h"

#include "valve.h"

#include "pca9502.h"

#include "ad840x.h"

#include "Display.h"


VALVE_STRU gValve;

static void ValveSwitchMeasChl(void *para)
{
    MULTIPLEX_STRU *pMult      = (MULTIPLEX_STRU *)para;
    MULTIPLEX_NODE_STRU *pNode = NULL;
    VALVE_NODE_STRU     *pValveNode;

    // 1. first get measured data

    if (list_empty(&pMult->head))
    {
        return ;
    }
    
    pNode = list_entry(pMult->head.next,MULTIPLEX_NODE_STRU,list);
    
    if (pNode->addData < VALVE_TOTAL_NUMBER) // just for sure
    {
        pValveNode = &gValve.aValves[pNode->addData];

        if (pValveNode->ucAdcChl4I < CM_MAX_NUMBER)
        {
            if (pValveNode->ucAdcData4IPtr < MAX_INPUT_REGISTERS)
            {
                Display.ausInputRegs[pValveNode->ucAdcData4IPtr] = GetAdcData(pValveNode->ucAdcChl4I);
				printf("\r\n%dCurrency is %d\r\n",pValveNode->ucAdcData4IPtr,Display.ausInputRegs[pValveNode->ucAdcData4IPtr]);
            }
        }

        if (pValveNode->ucAdcChl4V < CM_MAX_NUMBER)
        {
            if (pValveNode->ucAdcData4VPtr < MAX_INPUT_REGISTERS)
            {
                Display.ausInputRegs[pValveNode->ucAdcData4VPtr] = GetAdcData(pValveNode->ucAdcChl4V);
			   printf("\r\n%dVoltage is %d\r\n",pValveNode->ucAdcData4VPtr,Display.ausInputRegs[pValveNode->ucAdcData4VPtr]);
            }        
        }
    }

    // 2. then switch to next chl
    list_del_init(&pNode->list);

    list_add_tail(&pNode->list,&pMult->head);

    // 3. get header 
    pNode = list_entry(pMult->head.next,MULTIPLEX_NODE_STRU,list);

    // 4. switch
    if (pNode->addData < VALVE_TOTAL_NUMBER) // just for sure
    {
        pValveNode = &gValve.aValves[pNode->addData];

        PCA9502_UpdateGpios(pValveNode->ucPcaChl4IV,pValveNode->ucPacMask4IV,(1 << pValveNode->ucPcaSubChl4IV));
    }
    
}


void ValveInit(void)
{
    uint8_t ucLoop;

    uint8_t ucStart = 0;

    AD840x_Init();
    
    PCA9502_Init();

    memset(&gValve,0xFF,sizeof(VALVE_STRU));

    /* for magnetic gValve 1~8 */
    for (ucLoop = ucStart; ucLoop < ucStart + VALVE_MAGNETIC_NUM; ucLoop++)
    {
        gValve.aValves[ucLoop].ucType      = VALVE_TYPE_MAGNETIC;
        gValve.aValves[ucLoop].ucPcaChl    = 0;
        gValve.aValves[ucLoop].ucPcaSubChl = ucLoop;
    }

    ucStart += VALVE_MAGNETIC_NUM;
    
    /* for general pump 1~2 */
    for (ucLoop = ucStart; ucLoop < ucStart + VALVE_GENERAL_PUMP_NUM; ucLoop++)
    {
        gValve.aValves[ucLoop].ucType      = VALVE_TYPE_GENERAL_PUMP;
        gValve.aValves[ucLoop].ucPcaChl    = 2;
        gValve.aValves[ucLoop].ucPcaSubChl = ucLoop - ucStart;

        gValve.aValves[ucLoop].ucPcaChl4IV    = 2;
        gValve.aValves[ucLoop].ucPcaSubChl4IV = 4 + ucLoop - ucStart;
        gValve.aValves[ucLoop].ucPacMask4IV   = 0XF0;

        gValve.aValves[ucLoop].ucAdcChl4I     = 4;
        gValve.aValves[ucLoop].ucAdcSubChl4I  = ucLoop - ucStart;

        gValve.aValves[ucLoop].ucAdcData4IPtr = INPUT_REG_PUMP_OFFSET + ucLoop - ucStart;//  index into input register files

    }

    ucStart += VALVE_GENERAL_PUMP_NUM;

    /* for pressure  regulating pump 1~2 */
    for (ucLoop = ucStart; ucLoop < ucStart + VALVE_PRESSURE_REGULATING_PUMP_NUM; ucLoop++)
    {
        gValve.aValves[ucLoop].ucType         = VALVE_TYPE_PRESSURE_REGULATING_PUMP;
        gValve.aValves[ucLoop].ucPcaChl       = 2;
        gValve.aValves[ucLoop].ucPcaSubChl    = VALVE_GENERAL_PUMP_NUM + ucLoop - ucStart;

        gValve.aValves[ucLoop].ucPcaChl4IV    = 2;
        gValve.aValves[ucLoop].ucPcaSubChl4IV = 4 + VALVE_GENERAL_PUMP_NUM + ucLoop - ucStart;
        gValve.aValves[ucLoop].ucPacMask4IV   = 0XF0;

        gValve.aValves[ucLoop].ucAdcChl4I     = 4;
        gValve.aValves[ucLoop].ucAdcSubChl4I  = VALVE_GENERAL_PUMP_NUM + ucLoop - ucStart;
        gValve.aValves[ucLoop].ucAdcData4IPtr = INPUT_REG_REGPUMPI_OFFSET + ucLoop - ucStart;//  index into input register files

        gValve.aValves[ucLoop].ucAdcChl4V     = 5;
        gValve.aValves[ucLoop].ucAdcSubChl4V  = VALVE_GENERAL_PUMP_NUM + ucLoop - ucStart; // same as i channel
        gValve.aValves[ucLoop].ucAdcData4VPtr = INPUT_REG_REGPUMPV_OFFSET + ucLoop - ucStart;//  index into input register files
        

    }

    ucStart += VALVE_PRESSURE_REGULATING_PUMP_NUM;
    
    /* for rectifier 1~3 */
    for (ucLoop = ucStart; ucLoop < ucStart + VALVE_RECTIFIER_NUM; ucLoop++)
    {
        gValve.aValves[ucLoop].ucType         = VALVE_TYPE_RECTIFIER;
        gValve.aValves[ucLoop].ucPcaChl       = 1;
        gValve.aValves[ucLoop].ucPcaSubChl    = ucLoop - ucStart;

        gValve.aValves[ucLoop].ucPcaChl4IV    = 1;
        gValve.aValves[ucLoop].ucPcaSubChl4IV = 4 + ucLoop - ucStart;
        gValve.aValves[ucLoop].ucPacMask4IV   = 0XF0;

        gValve.aValves[ucLoop].ucAdcChl4I     = 3;
        gValve.aValves[ucLoop].ucAdcSubChl4I  = ucLoop - ucStart;        
        gValve.aValves[ucLoop].ucAdcData4IPtr = INPUT_REG_RECTIFIER_OFFSET + ucLoop - ucStart;//  index into input register files
    }

    ucStart += VALVE_RECTIFIER_NUM;
    
    /* for edi 1~1 */
    for (ucLoop = ucStart; ucLoop < ucStart + VALVE_EDI_NUM; ucLoop++)
    {
        gValve.aValves[ucLoop].ucType      = VALVE_TYPE_EDI;
        gValve.aValves[ucLoop].ucPcaChl    = 1;
        gValve.aValves[ucLoop].ucPcaSubChl = ucLoop - ucStart + VALVE_RECTIFIER_NUM;

        gValve.aValves[ucLoop].ucPcaChl4IV    = 1;
        gValve.aValves[ucLoop].ucPcaSubChl4IV = 4 + VALVE_RECTIFIER_NUM + ucLoop - ucStart;
        gValve.aValves[ucLoop].ucPacMask4IV   = 0XF0;

        gValve.aValves[ucLoop].ucAdcChl4I     = 3;
        gValve.aValves[ucLoop].ucAdcSubChl4I  = VALVE_RECTIFIER_NUM + ucLoop - ucStart;        
        gValve.aValves[ucLoop].ucAdcData4IPtr = INPUT_REG_EDI_OFFSET + ucLoop - ucStart;//  index into input register files
    }

    ADC_SetMultiplexer(3,ValveSwitchMeasChl,&gaMultiplex[0]);
    ADC_SetMultiplexer(4,ValveSwitchMeasChl,&gaMultiplex[1]);
    ADC_SetMultiplexer(5,ValveSwitchMeasChl,&gaMultiplex[2]);
    
}


void ValveCtrl(uint8_t ucChl,uint8_t ucEnable)
{
    PCA9502_SetGpio(gValve.aValves[ucChl].ucPcaChl,gValve.aValves[ucChl].ucPcaSubChl,ucEnable);
    printf("\r\nChl=%d,SubChl=%d,Enable=%d\r\n",gValve.aValves[ucChl].ucPcaChl,gValve.aValves[ucChl].ucPcaSubChl,ucEnable);
    // active / deactive measurement
    switch(gValve.aValves[ucChl].ucType)
    {
    case VALVE_TYPE_GENERAL_PUMP:
    case VALVE_TYPE_RECTIFIER:    
    case VALVE_TYPE_EDI: 
        if (ucEnable)
        {
            if (gValve.aValves[ucChl].ucAdcChl4I - 3 < MULTIPLEX_MAX_NUM)
            {
                 Mulitplex_Add(gValve.aValves[ucChl].ucAdcChl4I - 3,gValve.aValves[ucChl].ucAdcSubChl4I,ucChl);
            }
        }
        else
        {
            if (gValve.aValves[ucChl].ucAdcChl4I - 3 < MULTIPLEX_MAX_NUM)
            {
                 Mulitplex_Rmv(gValve.aValves[ucChl].ucAdcChl4I - 3,gValve.aValves[ucChl].ucAdcSubChl4I);
            }
        }
        break;
    case VALVE_TYPE_PRESSURE_REGULATING_PUMP:
        if (ucEnable)
        {
            if (gValve.aValves[ucChl].ucAdcChl4I - 3 < MULTIPLEX_MAX_NUM)
            {
                 Mulitplex_Add(gValve.aValves[ucChl].ucAdcChl4I - 3,gValve.aValves[ucChl].ucAdcSubChl4I,ucChl);
            }
            
            if (gValve.aValves[ucChl].ucAdcChl4V - 3 < MULTIPLEX_MAX_NUM)
            {
                 Mulitplex_Add(gValve.aValves[ucChl].ucAdcChl4V - 3,gValve.aValves[ucChl].ucAdcSubChl4V,0xff);
            }
        }
        else
        {
            if (gValve.aValves[ucChl].ucAdcChl4I - 3 < MULTIPLEX_MAX_NUM)
            {
                 Mulitplex_Rmv(gValve.aValves[ucChl].ucAdcChl4I - 3,gValve.aValves[ucChl].ucAdcSubChl4I);
            }
    
            if (gValve.aValves[ucChl].ucAdcChl4V - 3 < MULTIPLEX_MAX_NUM)
            {
                 Mulitplex_Rmv(gValve.aValves[ucChl].ucAdcChl4V - 3,gValve.aValves[ucChl].ucAdcSubChl4V);
            }
        }
        
        break;
    }
}

uint8_t ValveGetStatus(uint8_t ucChl)
{
    return PCA9502_GetBuffedGpio(gValve.aValves[ucChl].ucPcaChl,gValve.aValves[ucChl].ucPcaSubChl);
}

uint16_t ValveGetAllStatus(void)
{
    uint16_t usValue = 0;

    uint8_t ucLoop;

    for (ucLoop = 0; ucLoop < VALVE_TOTAL_NUMBER; ucLoop++)
    {
        if (PCA9502_GetBuffedGpio(gValve.aValves[ucLoop].ucPcaChl,gValve.aValves[ucLoop].ucPcaSubChl))
        {
            usValue |= 1 << ucLoop;
        }
    }

    return usValue;
}
