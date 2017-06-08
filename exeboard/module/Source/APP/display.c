#include "display.h"

#include "Relay.h"

#include "Dica.h"

#include <string.h>

#include <stdio.h>

#include <math.h>

#include <stdlib.h>

#include "RTC_Driver.h"

#include "LCD.h"

#include "keyboard.h"

#include "task.h"

#include "IWDG_Driver.h"

#include "font.h"

#include "lib_def.h"

#include "UartCmd.h"

#include "Sapp.h"


#include "hal_defs.h"

#include "Timer_driver.h"

#include "valve.h"

#include "adc.h"

#include "CanCmd.h"

#include "ad840x.h"

DISPLAY_STRU Display;

typedef union                                        
{
   float          f;
   unsigned char  auc[4];
   int            i;
   unsigned int   ul;
}un_data_type;

void Disp_EcoReport(void)
{
    uint8_t ucLoop;
    uint8_t ucPayLoad = 0;

    APP_PACKET_CLIENT_RPT_IND_STRU *pRpt = (APP_PACKET_CLIENT_RPT_IND_STRU *)Config_buff;
    APP_PACKET_RPT_ECO_STRU        *pFm  = (APP_PACKET_RPT_ECO_STRU *)pRpt->aucData;

    if (Display.ausHoldRegs[0])
    {
        pRpt->hdr.ucMsgType = APP_PACKET_CLIENT_REPORT;
        pRpt->hdr.ucDevType = APP_DEV_TYPE_EXE_BOARD;
        pRpt->ucRptType     = APP_PACKET_RPT_ECO;
        ucPayLoad++;
        for (ucLoop = 0; ucLoop < MAX_ECO_NUMBER; ucLoop++)
        {
            if (Display.ausHoldRegs[0] & (1 << ucLoop))
            {
                uint8_t ucDataPtr = 0x000D + ucLoop * 6;
 
                un_data_type ud = {0};
             
                pFm->ucId   = ucLoop;
 
                ud.auc[3] = (Display.ausInputRegs[ucDataPtr + 0] >> 8) & 0xff;
                ud.auc[2] = (Display.ausInputRegs[ucDataPtr + 0] >> 0) & 0xff;
                ud.auc[1] = (Display.ausInputRegs[ucDataPtr + 1] >> 8) & 0xff;
                ud.auc[0] = (Display.ausInputRegs[ucDataPtr + 1] >> 0) & 0xff;
                pFm->fValue = ud.f;
                pFm->usTemp = Display.ausInputRegs[ucDataPtr + 2];
                pFm++;
 
                ucPayLoad += sizeof(APP_PACKET_RPT_ECO_STRU);
            }
        }
        pRpt->hdr.ucLen = ucPayLoad;

        // broadcast
        CanSndSappCmd2(0XFF ,SAPP_CMD_DATA,(uint8_t *)pRpt,ucPayLoad + APP_PROTOL_HEADER_LEN); 
    }

}

void Disp_SecondTask(void)
{
    Disp_EcoReport();
}

void Disp_ModbusReadInputRegister(MODBUS_PACKET_COMM_STRU *pModReq,MODBUS_PACKET_COMM_STRU *pModRsp)
{
    int iOffset = 0;

    int iIdx = 1;
    
    uint16_t usRegNum;
    
    uint16_t usStartAdr = (pModReq->aucData[iOffset] << 8) | (pModReq->aucData[iOffset + 1] );

    iOffset += 2;

    usRegNum = (pModReq->aucData[iOffset] << 8) | (pModReq->aucData[iOffset + 1] );

    iOffset += 2;

    for (iOffset = usStartAdr; iOffset < usStartAdr + usRegNum; iOffset++)
    {
         pModRsp->aucData[iIdx + 0] = (Display.ausInputRegs[iOffset] >> 8) & 0XFF;
         pModRsp->aucData[iIdx + 1] = (Display.ausInputRegs[iOffset] >> 0) & 0XFF;

         iIdx += 2;

         if (usStartAdr >=  MAX_INPUT_REGISTERS)
         {
             break;
         }
         
    }
    
    pModRsp->aucData[0] = iIdx;

}

void Disp_ModbusReadInputStatus(MODBUS_PACKET_COMM_STRU *pModReq,MODBUS_PACKET_COMM_STRU *pModRsp)
{
    int iOffset = 0;

    uint16_t usRegNum;
    
    uint16_t usStartAdr = (pModReq->aucData[iOffset] << 8) | (pModReq->aucData[iOffset + 1] );

    iOffset += 2;

    usRegNum = (pModReq->aucData[iOffset] << 8) | (pModReq->aucData[iOffset + 1] );

    iOffset += 2;

    for (iOffset = usStartAdr; iOffset < usStartAdr + usRegNum; iOffset++)
    {
         uint8_t ucByte = (iOffset  - usStartAdr) / 8;
         uint8_t ucBits = (iOffset  - usStartAdr) % 8;

         if (iOffset >= INPUT_NUMBER)
         {
             break;
         }

         if (0 == ucBits)
         {
            pModRsp->aucData[1 + ucByte]  = GetInputLogicStatus(iOffset);
         }
         else
         {
            pModRsp->aucData[1 + ucByte] |= (GetInputLogicStatus(iOffset) << ucBits);
         }
    }
    
    pModRsp->aucData[0] = (iOffset -  usStartAdr + 7) / 8;

}

void Disp_ModbusForceSingleCoil(MODBUS_PACKET_COMM_STRU *pModReq,MODBUS_PACKET_COMM_STRU *pModRsp)
{
    int iOffset = 0;

    uint16_t usRegValue;
    
    uint16_t usStartAdr = (pModReq->aucData[iOffset] << 8) | (pModReq->aucData[iOffset + 1] );

    iOffset += 2;

    usRegValue = (pModReq->aucData[iOffset] << 8) | (pModReq->aucData[iOffset + 1] );

    iOffset += 2;

    if (usStartAdr < VALVE_TOTAL_NUMBER)
    {
         ValveCtrl(usStartAdr,!!usRegValue);
    }

    pModRsp->aucData[0] = pModReq->aucData[0];    
    pModRsp->aucData[1] = pModReq->aucData[1];    
    pModRsp->aucData[2] = pModReq->aucData[2];    
    pModRsp->aucData[3] = pModReq->aucData[3];    
    
}

void Disp_ModbusForceMultipleCoils(MODBUS_PACKET_COMM_STRU *pModReq,MODBUS_PACKET_COMM_STRU *pModRsp)
{
    int iOffset = 0;

    int iLoop;

    uint16_t usRegNum;
    
    uint16_t usStartAdr = (pModReq->aucData[iOffset] << 8) | (pModReq->aucData[iOffset + 1] );

    iOffset += 2;

    usRegNum = (pModReq->aucData[iOffset] << 8) | (pModReq->aucData[iOffset + 1] );

    iOffset += 2;

    iOffset += 1; // skip counts

    for (iLoop = usStartAdr; iLoop < usStartAdr + usRegNum; iLoop++)
    {
         uint8_t ucByte = (iLoop - usStartAdr) / 8;
         uint8_t ucBits = (iLoop - usStartAdr)% 8;

         if (iLoop >= VALVE_TOTAL_NUMBER)
         {
             break;
         }

         if (pModReq->aucData[iOffset + ucByte] & (1 << ucBits))
         {
             ValveCtrl(iLoop,1);
         }
         else
         {
             ValveCtrl(iLoop,0);
         }
    }
    
    pModRsp->aucData[0] = pModReq->aucData[0];    
    pModRsp->aucData[1] = pModReq->aucData[1];    
    pModRsp->aucData[2] = pModReq->aucData[2];    
    pModRsp->aucData[3] = pModReq->aucData[3];    

}

void Disp_ModbusReadCoilStatus(MODBUS_PACKET_COMM_STRU *pModReq,MODBUS_PACKET_COMM_STRU *pModRsp)
{
    int iOffset = 0;

    uint16_t usRegNum;
    
    uint16_t usStartAdr = (pModReq->aucData[iOffset] << 8) | (pModReq->aucData[iOffset + 1] );

    iOffset += 2;

    usRegNum = (pModReq->aucData[iOffset] << 8) | (pModReq->aucData[iOffset + 1] );

    iOffset += 2;

    for (iOffset = usStartAdr; iOffset < usStartAdr + usRegNum; iOffset++)
    {
         uint8_t ucByte = (iOffset - usStartAdr) / 8;
         uint8_t ucBits = (iOffset - usStartAdr) % 8;

         if (iOffset >= VALVE_TOTAL_NUMBER)
         {
             break;
         }

         if (0 == ucBits)
         {
            pModRsp->aucData[1 + ucByte]  = ValveGetStatus(iOffset);
         }
         else
         {
            pModRsp->aucData[1 + ucByte] |= (ValveGetStatus(iOffset) << ucBits);
         }
    }
    
    pModRsp->aucData[0] = (iOffset -  usStartAdr + 7) / 8;

}


void Disp_ModbusMaskWrite0XRegister(MODBUS_PACKET_COMM_STRU *pModReq,MODBUS_PACKET_COMM_STRU *pModRsp)
{
    int iOffset = 0;

    uint16_t usAndMask;

    uint16_t usOrMask;
    
    uint16_t usStartAdr = (pModReq->aucData[iOffset] << 8) | (pModReq->aucData[iOffset + 1] );

    iOffset += 2;

    usAndMask = (pModReq->aucData[iOffset] << 8) | (pModReq->aucData[iOffset + 1] );

    iOffset += 2;

    usOrMask = (pModReq->aucData[iOffset] << 8) | (pModReq->aucData[iOffset + 1] );

    iOffset += 2;

    if (usStartAdr == 0) // only one register
    {
         usStartAdr = ValveGetAllStatus();

         for (iOffset = 0; iOffset < VALVE_TOTAL_NUMBER; iOffset++)
         {
             if (usAndMask & (1 << iOffset))
             {
                 if (usOrMask & (1 << iOffset))
                 {
                     ValveCtrl(iOffset,1);
                 }
                 else
                 {
                     ValveCtrl(iOffset,0);
                 }
             }
         }

    }
    
    pModRsp->aucData[0] = pModReq->aucData[0];    
    pModRsp->aucData[1] = pModReq->aucData[1];    
    pModRsp->aucData[2] = pModReq->aucData[2];    
    pModRsp->aucData[3] = pModReq->aucData[3];    
    pModRsp->aucData[4] = pModReq->aucData[4];    
    pModRsp->aucData[5] = pModReq->aucData[5];    

}

void Disp_ModbusPresetSingleRegister(MODBUS_PACKET_COMM_STRU *pModReq,MODBUS_PACKET_COMM_STRU *pModRsp)
{
    int iOffset = 0;

    uint16_t usValue ;
    
    uint16_t usStartAdr = (pModReq->aucData[iOffset] << 8) | (pModReq->aucData[iOffset + 1] );

    iOffset += 2;

    usValue = (pModReq->aucData[iOffset] << 8) | (pModReq->aucData[iOffset + 1] );

    iOffset += 2;

    pModRsp->aucData[0] = pModReq->aucData[0];
    pModRsp->aucData[1] = pModReq->aucData[1];
    pModRsp->aucData[2] = pModReq->aucData[2];
    pModRsp->aucData[3] = pModReq->aucData[3];

    if (usStartAdr < MAX_HOLD_REGISTERS)
    {
        Display.ausHoldRegs[usStartAdr] = usValue;

        switch(usStartAdr)
        {
        case HOLD_REGS_NAME_PUMP1:
        case HOLD_REGS_NAME_PUMP2:
            AD840x_Write(usStartAdr - HOLD_REGS_NAME_PUMP1,usValue);
            break;
        default:
            break;
        }
    }

}

void Disp_ModbusPresetMultipleRegister(MODBUS_PACKET_COMM_STRU *pModReq,MODBUS_PACKET_COMM_STRU *pModRsp)
{
    int iOffset = 0;

    uint16_t usNums ;

    uint16_t usLoop;
    
    uint16_t usStartAdr = (pModReq->aucData[iOffset] << 8) | (pModReq->aucData[iOffset + 1] );

    uint16_t usValue;

    iOffset += 2;

    usNums = (pModReq->aucData[iOffset] << 8) | (pModReq->aucData[iOffset + 1] );

    iOffset += 2;

    iOffset++; // skip count 

    for (usLoop = 0; usLoop < usNums; usLoop++)
    {
        usValue = (pModReq->aucData[iOffset] << 8) | (pModReq->aucData[iOffset + 1] );
        
        if (usStartAdr + usLoop < MAX_HOLD_REGISTERS)
        {
            Display.ausHoldRegs[usStartAdr + usLoop] = usValue;

            switch(usStartAdr + usLoop)
            {
            case HOLD_REGS_NAME_PUMP1:
            case HOLD_REGS_NAME_PUMP2:
                AD840x_Write(usStartAdr - HOLD_REGS_NAME_PUMP1,usValue);
                break;
            default:
                break;
            }            
        }
  
        iOffset += 2;
    }

    pModRsp->aucData[0] = pModReq->aucData[0];
    pModRsp->aucData[1] = pModReq->aucData[1];
    pModRsp->aucData[2] = pModReq->aucData[2];
    pModRsp->aucData[3] = pModReq->aucData[3];

}


void Disp_ModbusMaskWrite4XRegister(MODBUS_PACKET_COMM_STRU *pModReq,MODBUS_PACKET_COMM_STRU *pModRsp)
{
    int iOffset = 0;

    uint16_t usAndMask;

    uint16_t usOrMask;
    
    uint16_t usStartAdr = (pModReq->aucData[iOffset] << 8) | (pModReq->aucData[iOffset + 1] );

    iOffset += 2;

    usAndMask = (pModReq->aucData[iOffset] << 8) | (pModReq->aucData[iOffset + 1] );

    iOffset += 2;

    usOrMask = (pModReq->aucData[iOffset] << 8) | (pModReq->aucData[iOffset + 1] );

    iOffset += 2;

    if (usStartAdr < MAX_HOLD_REGISTERS) // only one register
    {
         for (iOffset = 0; iOffset < 16; iOffset++)
         {
             if (usAndMask & (1 << iOffset))
             {
                 if (usOrMask & (1 << iOffset))
                 {
                     Display.ausHoldRegs[usStartAdr] |= (1 << iOffset);
                 }
                 else
                 {
                     Display.ausHoldRegs[usStartAdr] &= ~(1 << iOffset);
                 }
             }
         }

         switch(usStartAdr)
         {
         case HOLD_REGS_NAME_PUMP1:
         case HOLD_REGS_NAME_PUMP2:
             AD840x_Write(usStartAdr - HOLD_REGS_NAME_PUMP1,Display.ausHoldRegs[usStartAdr]);
             break;
         default:
             break;
         }            
         

    }
    
    pModRsp->aucData[0] = pModReq->aucData[0];    
    pModRsp->aucData[1] = pModReq->aucData[1];    
    pModRsp->aucData[2] = pModReq->aucData[2];    
    pModRsp->aucData[3] = pModReq->aucData[3];    
    pModRsp->aucData[4] = pModReq->aucData[4];    
    pModRsp->aucData[5] = pModReq->aucData[5];    

}


static void Disp_PressureAdcCb(void *para)
{ 
    int iChl = (int)para;

    uint8_t ucAdcChl      = ((iChl & 0XFF00) >> 8);
    uint8_t ucPressureChl = ((iChl & 0XFF) >> 0);

    Display.ausInputRegs[ucPressureChl] = GetAdcData(ucAdcChl);
}

void Disp_Init(void)
{
    memset(&Display,0,sizeof(DISPLAY_STRU));

    ADC_SetMultiplexer(0,Disp_PressureAdcCb,(void *)((0<< 8 )|0));
    ADC_SetMultiplexer(1,Disp_PressureAdcCb,(void *)((1<< 8 )|1));
    ADC_SetMultiplexer(2,Disp_PressureAdcCb,(void *)((2<< 8 )|2));

}

