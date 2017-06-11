#ifndef _CCB_H_
#define _CCB_H_

#include "SatIpc.h"
#include "threadpool.h"
#include "Errorcode.h"
#include "msgdef.h"
#include "msg.h"
#include "list.h"
#include "timer.h"
#include "rpc.h"
#include "sapp.h"
#include "datatype.h"
#include "inneripc.h"

#include "Display.h"
#include "cminterface.h"
#include "list.h"

#ifdef __cplusplus
 extern "C"
 {
#endif

 /* special address description flags for the CAN_ID */
#define CAN_EFF_FLAG 0x80000000U /* EFF/SFF is set in the MSB */
#define CAN_RTR_FLAG 0x40000000U /* remote transmission request */
#define CAN_ERR_FLAG 0x20000000U /* error frame */
 
 /* valid bits in CAN ID for frame formats */
#define CAN_SFF_MASK 0x000007FFU /* standard frame format (SFF) */
#define CAN_EFF_MASK 0x1FFFFFFFU /* extended frame format (EFF) */
#define CAN_ERR_MASK 0x1FFFFFFFU /* omit EFF, RTR, ERR flags */

#define MAX_CAN_MESSAGE_LEN 256

#define CAN_ADDRESS_MASK (0X3FF)

#define CAN_SRC_ADDRESS_POS (10) // BIT0~BIT9 in indentifer

#define CAN_SRC_ADDRESS(indenfier)((indenfier >> CAN_SRC_ADDRESS_POS) & CAN_ADDRESS_MASK)

#define CAN_DST_ADDRESS_POS (0) // BIT0~BIT9 in indentifer

#define CAN_DST_ADDRESS(indenfier)((indenfier >> CAN_DST_ADDRESS_POS) & CAN_ADDRESS_MASK)

#define CAN_BROAD_CAST_ADDRESS (0X3FF)

#define CAN_ADDRESS(indenfier)((indenfier) & CAN_ADDRESS_MASK)

#define CAN_BUILD_ADDRESS_IDENTIFIER(ulIdentifier,usSrcCanAdr,usDstCanAdr)(ulIdentifier=(usSrcCanAdr<<CAN_SRC_ADDRESS_POS)|(usDstCanAdr << CAN_DST_ADDRESS_POS)|CAN_EFF_FLAG)

#define CAN_INVALID_ADDRESS (0XFFFF)

#define INVALID_CAN_ADDRESS(usAddress) ((usAddress & CAN_BROAD_CAST_ADDRESS) == CAN_BROAD_CAST_ADDRESS)

#define TIMER_SECOND 0
#define TIMER_MINUTE 1

#define MAX_HANDLER_NUM (2)


#define MAX_MODBUS_BUFFER_SIZE (128)

typedef void (*modbus_call_back)(void *);

typedef void (*work_proc)(void *);

typedef struct
{
    list_t    list;
    work_proc Work;
    void      *Para;
}WORK_STRU;

typedef struct
{
    unsigned int ulEcoValidFlags;

    DISP_OBJ_STRU aValveObjs[APP_EXE_VALVE_NUM];
    DISP_OBJ_STRU aPMObjs[APP_EXE_PRESSURE_METER]; /* Pressure Meter */
    DISP_OBJ_STRU aGPumpObjs[APP_EXE_G_PUMP_NUM];
    DISP_OBJ_STRU aRPumpObjs[APP_EXE_R_PUMP_NUM];
    DISP_OBJ_STRU aRectObjs[APP_EXE_RECT_NUM];
    DISP_OBJ_STRU aEDIObjs[APP_EXE_EDI_NUM];
    DISP_OBJ_STRU aEcoObjs[APP_EXE_ECO_NUM];
    
}EXE_BOARD_STRU;


typedef struct
{
    unsigned int ulFmValidFlags;
    
    DISP_OBJ_STRU aFmObjs[APP_FM_FLOW_METER_NUM];
}FLOW_METER_STRU;


typedef struct
{
    int iDummy;
}HANDLER_STRU;

typedef struct
{
   
   unsigned int     ulRegisterMask;
   unsigned int     ulActiveMask;
   
   EXE_BOARD_STRU   ExeBrd;

   FLOW_METER_STRU  FlowMeter;
   
   HANDLER_STRU     aHandler[MAX_HANDLER_NUM];

   int              iHbtCnt;

   INNER_IPC_STRU   Ipc;
   modbus_call_back ModbusCb;

   int              iWorkState; // refer DISP_WORK_STATE_ENUM

   list_t           WorkList;


   unsigned  char   aucModbusBuffer[MAX_MODBUS_BUFFER_SIZE];
   
}CCB;

extern int MSG_QUEUE_ID_MAIN ;
extern int MSG_QUEUE_ID_CANITF;

extern tls_key_t  pthread_tno_key;
extern threadpool gThdPool;

extern CCB gCcb;
extern unsigned int gSecond;

#define MOTOR_MAGIC 'L'
#define FEED_DOG 	_IOW(MOTOR_MAGIC, 0,int)

#ifdef __cplusplus
}
#endif

#endif

