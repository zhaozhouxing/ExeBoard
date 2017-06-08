#include    <ucos_ii.h>

#include    <string.h>

#include "stm32f10x.h"

#include "memory.h"
#include "task.h"
#include "timer.h"

#include "stm32_eval.h"

#include "serial_driver.h"

#include "Serial.h"

#include "relay.h"

#include "qmi.h"

#include "Beep.h"

#include "config.h"

#include "CanCmd.h"

#include "dica.h"

#include "Display.h"

#define MODBUS_ADR_POS (0)
#define MODBUS_CODE_POS (1)
#define MODBUS_CONT_POS (2)

#define MOSBUS_HEADER_LEN (2)

typedef enum
{
   MODBUS_ADDRESS_MODULE_QM        = 0XC1,        // Weight module
   
}MODBUS_ADDRESS_MODULE_ENUM;


typedef enum
{
    MODBUS_TARGET_QUERY_QM = 0,     //
    
    MODBUS_TARGET_DELAY,            // 

    MODBUS_TARGET_BUTT,
    
}MODBUS_ACTION_ENUM;


#define MODBUS_MAX_ACTIONS (20)

typedef void (*modbus_msg_cb)(void);

typedef void (*run_action_callback)(void *);


typedef struct
{
    UINT8 ucActionId;       // refer MODBUS_ACTION_ENUM
    run_action_callback cb; // callback for current buffer action done
    int para;               // para for action id
    int para1;              // para for cb
}RUN_ACTION_STRU;


typedef struct
{
    RUN_ACTION_STRU aActionBuf[MODBUS_MAX_ACTIONS]; // should be bigger enough to avoid overflow
    UINT8 ucActionCnt;
    UINT8 ucActionIdx;
    run_action_callback sac; // callback for all buffer action done

    sys_timeo to4ActionDelay;
    sys_timeo to4State;
    
}RUN_BUFFER_ACTION_STRU;


typedef struct
{
    uint8_t ucIdleTime;

    sys_timeo to4Schedule;

    RUN_BUFFER_ACTION_STRU RunBufAction;
    
}MODBUS_SCHEDULE_STRU;

#define MODBUS_IDLE_REPORT (5) // REPORT EVERY 5 SECOND

typedef struct
{
    MsgHead msgHead;
    void *para;
}MODBUS_MSG;

#define MODBUS_MSG_LENGHT (sizeof(MODBUS_MSG)-sizeof(MsgHead))


MODBUS_SCHEDULE_STRU ModbusSheduler;

uint8_t aucModbus_buff[32];


const unsigned char auchCRCHi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
    };

const unsigned char auchCRCLo[]={
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
    0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
    0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
    0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
    0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
    0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
    0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
    0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
    0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
    0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
    0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
    0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
    0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
    0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
    0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
    0x43, 0x83, 0x41, 0x81, 0x80, 0x40
    };

void Modbus_CommMove2Destination(void);

  
unsigned int calcrc16(unsigned char *puchMsgg,unsigned int usDataLen)
{
    unsigned char  uchCRCHi = 0xFF ; /* 高CRC字节初始化 */
    unsigned char  uchCRCLo = 0xFF ; /* 低CRC 字节初始化 */

    unsigned int uIndex ; /* CRC循环中的索引 */
    while (usDataLen--) /* 传输消息缓冲区 */
    {
        uIndex = uchCRCHi ^ *puchMsgg++ ; /* 计算CRC */
        uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex] ;
        uchCRCLo = auchCRCLo[uIndex] ;
    }
    return (uchCRCHi << 8 | uchCRCLo) ;
}


void Modbus_MakeMessage(unsigned char *pModbusMsg,unsigned char ucLength)
{
    unsigned short usCrc;

    usCrc = (unsigned short)calcrc16(pModbusMsg,ucLength); // six bytes for preset single register

    pModbusMsg[ucLength] = (usCrc >> 8) & 0xff;
    pModbusMsg[ucLength+1] = (usCrc >> 0) & 0xff;
}

void appQmi_config_cb(uint8_t ucPort)
{
    memset(&Serial[ucPort],0,offsetof(SERIAL_STRU,ucDriverType));

    Serial[ucPort].ucDriverType = MSG_DRIVER;
    Serial[ucPort].ucPortType   = RS485;
    Serial[ucPort].ucPortCtrl   = 0; // DONT CARE FOR RS232

    switch(ucPort)
    {
    case SERIAL_PORT1:
        Serial[ucPort].UsartDef = EVAL_COM2;
        Serial[ucPort].iIrq     = EVAL_COM2_IRQn;
        Serial[ucPort].iComIdx  = COM2;
        Serial[ucPort].ucPortCtrl = STM32F103_GPA(1);
        break;
    case SERIAL_PORT2:
        Serial[ucPort].UsartDef = EVAL_COM3;
        Serial[ucPort].iIrq     = EVAL_COM3_IRQn;
        Serial[ucPort].iComIdx  = COM3;
        Serial[ucPort].ucPortCtrl = STM32F103_GPA(13);
        break;
        
    }

    Serial_RetriveConfig(ucPort);

}

void Modbus_QmHandler(uint8_t *pData, uint8_t ucLen)
{
   // do something
   int iChl = pData[MODBUS_ADR_POS] - MODBUS_ADDRESS_MODULE_QM;

   uint8_t ucDataPtr = INPUT_REG_QMI_OFFSET + iChl * 3;

   Display.ausInputRegs[ucDataPtr + 0] = (pData[MODBUS_CONT_POS + 1] << 8) | (pData[MODBUS_CONT_POS + 2] << 0);
   Display.ausInputRegs[ucDataPtr + 1] = (pData[MODBUS_CONT_POS + 3] << 8) | (pData[MODBUS_CONT_POS + 4] << 0);
   Display.ausInputRegs[ucDataPtr + 2] = (pData[MODBUS_CONT_POS + 5] << 8) | (pData[MODBUS_CONT_POS + 6] << 0);

}

void Modbus_MsgHandler(uint8_t *pData, uint8_t ucLen)
{
    // for test purpose 
    MainAlarmWithDuration(1);
    
    // pre handle message
    sys_untimeout(&ModbusSheduler.RunBufAction.to4State);

    // process here (late implement)
    switch(pData[MODBUS_ADR_POS])
    {
    case MODBUS_ADDRESS_MODULE_QM + 0:
    case MODBUS_ADDRESS_MODULE_QM + 1:
    case MODBUS_ADDRESS_MODULE_QM + 2:
    case MODBUS_ADDRESS_MODULE_QM + 3:
    case MODBUS_ADDRESS_MODULE_QM + 4:
        {
            Modbus_QmHandler(pData,ucLen);
        }
        break;
    default:
        break;
    }

    Modbus_CommMove2Destination();

}



void appQmi_ItfProcess(Message *pMsg)
{


    // first parse message
    // 1. check modbus address
    UINT8   ucCnt = 0;
    UINT16  usCrc;
    UINT8   ucAdr;
    UINT8  *pModbus = (UINT8 *)pMsg->data;

#ifdef TESTER
        MainAlarmWithDuration(1);
#endif

    ucAdr = pModbus[MODBUS_ADR_POS];

    // 1. first parese message
    if (ucAdr < MODBUS_ADDRESS_MODULE_QM
        || ucAdr > MODBUS_ADDRESS_MODULE_QM + 4)
    {
        return ; 
    }
    // 2. check code

    switch (pModbus[MODBUS_CODE_POS])
    {
    case 0x2:
        ucCnt = pModbus[MODBUS_CONT_POS] + 1 + MOSBUS_HEADER_LEN; // 1bytes for rsplen info
        break;
    default :
        return; 
    }
    // check crc
    usCrc = calcrc16((UINT8 *)pMsg->data,ucCnt);

    if (((usCrc >> 8 ) & 0xff) != pMsg->data[ucCnt]
        || ((usCrc >> 0 ) & 0xff) != pMsg->data[ucCnt+1])
    {
        VOS_LOG(VOS_LOG_DEBUG, "mitf crc fail %x,%x",usCrc,(pMsg->data[ucCnt]<<8)|pMsg->data[ucCnt+1]);
        return;
    }

    Modbus_MsgHandler((uint8_t *)pMsg->data,ucCnt);
    
}

void appQmi_msg_Handler(Message *Msg)
{
    MODBUS_MSG *dmsg = (MODBUS_MSG *)Msg;

    if (dmsg->para)
    {
        ((modbus_msg_cb)dmsg->para)();
    }
}

// function for serialization
void Modbus_report(void *para)
{
   Message *Msg;
   Msg = MessageAlloc(PID_SELF,MODBUS_MSG_LENGHT);

   if (Msg)
   {
       MODBUS_MSG *dmsg = (MODBUS_MSG *)Msg;
       dmsg->msgHead.nMsgType = SELF_MSG_CODE_USER_QM;
       dmsg->para = para;
       MessageSend(Msg);
   }
}

void RunBuffActionInit(RUN_BUFFER_ACTION_STRU *ba)
{
    memset(ba,0,offsetof(RUN_BUFFER_ACTION_STRU,to4ActionDelay));
}


uint8 Modbus_GetCurrentAction(void)
{
    RUN_BUFFER_ACTION_STRU *ba = &ModbusSheduler.RunBufAction;

    if (ba->ucActionIdx <= ba->ucActionCnt
        && ba->ucActionCnt > 0 )
    {
        return ba->aActionBuf[ba->ucActionIdx-1].ucActionId;   
    }
    
    return MODBUS_TARGET_BUTT;
}

void Modbus_CommActionDelay_msg_cb(void)
{
    RUN_BUFFER_ACTION_STRU *ba = &ModbusSheduler.RunBufAction;

    if ((ba->ucActionIdx >= 1) 
       && (NULL != ba->aActionBuf[ba->ucActionIdx-1].cb))
    {
        ba->aActionBuf[ba->ucActionIdx-1].cb(NULL);    
    }
    
    Modbus_CommMove2Destination();

}


void Modbus_CommActionDelayTimeoutHandler(void *para)
{
    Modbus_report(Modbus_CommActionDelay_msg_cb);

}

void Modbus_CommActionDelay(int delay)
{
    sys_timeout(delay,SYS_TIMER_ONE_SHOT,delay,Modbus_CommActionDelayTimeoutHandler,NULL,&ModbusSheduler.RunBufAction.to4ActionDelay); 
}


void Modbus_CommStateTimeout_msg_cb(void)
{
    RUN_BUFFER_ACTION_STRU *ba = &ModbusSheduler.RunBufAction;

    //int ActionId = (int) (para);

    if ((ba->ucActionIdx >= 1) 
       && (NULL != ba->aActionBuf[ba->ucActionIdx-1].cb))
    {
        ba->aActionBuf[ba->ucActionIdx-1].cb(NULL);    
    }
    
    Modbus_CommMove2Destination();

}

void Modbus_CommStateTimeoutHandler(void *para)
{
    Modbus_report(Modbus_CommStateTimeout_msg_cb);

}

void Modbus_Action4Qm(int modadr,int addr)
{
    uint8_t ucIdx = 0;
    
    aucModbus_buff[ucIdx++] = modadr; // addr
    aucModbus_buff[ucIdx++] = 0x2; // func code

    aucModbus_buff[ucIdx++] = (addr >> 8) & 0xff; // reg code
    aucModbus_buff[ucIdx++] = (addr >> 0) & 0xff; // reg code

    aucModbus_buff[ucIdx++] = (6 >> 8) & 0xff; // reg numbers
    aucModbus_buff[ucIdx++] = (6 >> 0) & 0xff; // reg numbers

    Modbus_MakeMessage(aucModbus_buff,ucIdx);   

    Serial_FillSndBuf(SERIAL_PORT1,aucModbus_buff,ucIdx+2); // extra byte for RS485 

    sys_timeout(150,SYS_TIMER_ONE_SHOT,0,Modbus_CommStateTimeoutHandler,(void *)modadr,&ModbusSheduler.RunBufAction.to4State); 
}


void Modbus_CommMove2Destination(void)
{
    UINT8 ucActionIdx;

    RUN_BUFFER_ACTION_STRU *ba = &ModbusSheduler.RunBufAction;

    if (ba->ucActionIdx == ba->ucActionCnt) 
    {
        ba->ucActionIdx += 0X80; // prevent further action

        // end callback
        if (ba->sac)
        {
            ba->sac(NULL);
        }
        return;
    }

    if (ba->ucActionIdx < ba->ucActionCnt)
    {
        ucActionIdx = ba->ucActionIdx;
        ba->ucActionIdx++;
        switch(ba->aActionBuf[ucActionIdx].ucActionId)
        {
        case MODBUS_TARGET_QUERY_QM:
            Modbus_Action4Qm(ba->aActionBuf[ucActionIdx].para,ba->aActionBuf[ucActionIdx].para1);
            break;
        case MODBUS_TARGET_DELAY:
            Modbus_CommActionDelay(ba->aActionBuf[ucActionIdx].para);
            break;
        default:
            break;
        }   
        return ;
    }
}



void Modbus_Scheduler_msg_cb(void)
{
    uint8_t ucIdx = 0;

    sys_untimeout(&ModbusSheduler.RunBufAction.to4ActionDelay);

    sys_untimeout(&ModbusSheduler.RunBufAction.to4State);

    RunBuffActionInit(&ModbusSheduler.RunBufAction);

    // fill action buffer & start
    
    ModbusSheduler.RunBufAction.aActionBuf[ucIdx].ucActionId = MODBUS_TARGET_QUERY_QM;
    ModbusSheduler.RunBufAction.aActionBuf[ucIdx].para       = MODBUS_ADDRESS_MODULE_QM; // address
    ModbusSheduler.RunBufAction.aActionBuf[ucIdx].para1      = 0x0003; // address
    ucIdx++;
    
    ModbusSheduler.RunBufAction.aActionBuf[ucIdx].ucActionId = MODBUS_TARGET_DELAY;
    ModbusSheduler.RunBufAction.aActionBuf[ucIdx].para       = 20; // interval for modbus
    ucIdx++;

    ModbusSheduler.RunBufAction.aActionBuf[ucIdx].ucActionId = MODBUS_TARGET_QUERY_QM;
    ModbusSheduler.RunBufAction.aActionBuf[ucIdx].para       = MODBUS_ADDRESS_MODULE_QM + 1; // address
    ModbusSheduler.RunBufAction.aActionBuf[ucIdx].para1      = 0x0010; // address
    ucIdx++;
    
    ModbusSheduler.RunBufAction.aActionBuf[ucIdx].ucActionId = MODBUS_TARGET_DELAY;
    ModbusSheduler.RunBufAction.aActionBuf[ucIdx].para       = 20; // interval for modbus
    ucIdx++;

    ModbusSheduler.RunBufAction.aActionBuf[ucIdx].ucActionId = MODBUS_TARGET_QUERY_QM;
    ModbusSheduler.RunBufAction.aActionBuf[ucIdx].para       = MODBUS_ADDRESS_MODULE_QM + 2; // address
    ModbusSheduler.RunBufAction.aActionBuf[ucIdx].para1      = 0x001b; // address
    ucIdx++;
    
    ModbusSheduler.RunBufAction.aActionBuf[ucIdx].ucActionId = MODBUS_TARGET_DELAY;
    ModbusSheduler.RunBufAction.aActionBuf[ucIdx].para       = 20; // interval for modbus
    ucIdx++;

    ModbusSheduler.RunBufAction.aActionBuf[ucIdx].ucActionId = MODBUS_TARGET_QUERY_QM;
    ModbusSheduler.RunBufAction.aActionBuf[ucIdx].para       = MODBUS_ADDRESS_MODULE_QM + 3; // address
    ModbusSheduler.RunBufAction.aActionBuf[ucIdx].para1      = 0x0026; // address
    ucIdx++;
    
    ModbusSheduler.RunBufAction.aActionBuf[ucIdx].ucActionId = MODBUS_TARGET_DELAY;
    ModbusSheduler.RunBufAction.aActionBuf[ucIdx].para       = 20; // interval for modbus
    ucIdx++;

    ModbusSheduler.RunBufAction.aActionBuf[ucIdx].ucActionId = MODBUS_TARGET_QUERY_QM;
    ModbusSheduler.RunBufAction.aActionBuf[ucIdx].para       = MODBUS_ADDRESS_MODULE_QM + 4; // address
    ModbusSheduler.RunBufAction.aActionBuf[ucIdx].para1      = 0x0031; // address
    ucIdx++;
    
    ModbusSheduler.RunBufAction.aActionBuf[ucIdx].ucActionId = MODBUS_TARGET_DELAY;
    ModbusSheduler.RunBufAction.aActionBuf[ucIdx].para       = 20; // interval for modbus
    ucIdx++;

    ModbusSheduler.RunBufAction.ucActionCnt = ucIdx;

    Modbus_CommMove2Destination();
}



void Modbus_Scheduler_cb(void *para)
{
    Modbus_report(Modbus_Scheduler_msg_cb);
}


void appQmiInit(void)
{
   uint8_t ucPortIdx = SERIAL_PORT1;

   for (ucPortIdx = SERIAL_PORT1; ucPortIdx < SERIAL_PORT_NUM;ucPortIdx++)
   {
       Serial[ucPortIdx].sccb = appQmi_config_cb;
   
       Serial[ucPortIdx].mcb  = appQmi_ItfProcess;
   
       SerialInitPort(ucPortIdx);   
   }

   memset(&ModbusSheduler,0,sizeof(MODBUS_SCHEDULE_STRU));
   
   
   sys_timeout(1000,SYS_TIMER_PERIOD,1000,Modbus_Scheduler_cb,NULL,&ModbusSheduler.to4Schedule); 
  
}

UINT8  appQmi_FillSndBuf(UINT8 ucPort,UINT8 *pData,UINT16 usLength)
{
    return Serial_FillSndBuf(ucPort + SERIAL_PORT1,pData,usLength);
}
    
