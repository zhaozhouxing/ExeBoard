#ifndef _DISPLAY_H_
#define _DISPLAY_H_

#include "datatype.h"
#include "cminterface.h"

#ifdef __cplusplus
 extern "C"
 {
#endif


typedef void * DISPHANDLE;

#define DISP_INVALID_HANDLE (NULL)

typedef enum
{
    DISP_WORK_STATE_IDLE = 0,
    DISP_WORK_STATE_RUN,
    DISP_WORK_STATE_LPP,     /* Low pressure protect state */
    
}DISP_WORK_STATE_ENUM;

typedef enum
{
   /* works availabe under IDLE State */

   
   DISP_WORK_SUB_IDLE    = 0,
   DISP_WORK_SUB_RO_WASH, 
   DISP_WORK_SUB_PH_WASH,

   /* works availabe under RUN State  */

   DISP_WORK_SUB_RUN_INIT,
   DISP_WORK_SUB_RUN_QTW,
   DISP_WORK_SUB_RUN_TW,
   
   
}DISP_WORK_SUB_STATE_ENUM;

typedef enum
{
   /* works availabe under IDLE State */

   
   DISP_CMD_IDLE    = 0,  /* return to idle state */

   DISP_CMD_RUN,

   DISP_CMD_RO_WASH,

   DISP_CMD_PH_WASH ,  
   
}DISP_CMD_ENUM;


typedef union
{
   unsigned int       ulV;
   float              fV;
   APP_ECO_VALUE_STRU eV;
}DISP_OBJ_VALUE;

typedef struct
{
    int emDispObjType; /* refer APP_OBJ_TYPE_ENUM */
    int iDispObjId;
    int iState;        /* 0: normal , 1: breakdown */
    int iActive;       /* 0: inactive ,1 active */
    int iVChoice;
    DISP_OBJ_VALUE Value;
}DISP_OBJ_STRU;

typedef struct
{
    unsigned int ulRoWashT1;
    unsigned int ulRoWashT2;
    unsigned int ulRoWashT3;

    unsigned int ulPhWashT1;
    unsigned int ulPhWashT2;
    unsigned int ulPhWashT3;
    unsigned int ulPhWashT4;
    unsigned int ulPhWashT5;
    
}DISP_PARAM_STRU;

#define DISP_MAKE_WORK_STATE(Main,Sub) (((Main)<<8)|(Sub))

#define DISP_GET_WORK_MAIN_STATE(State)(((State)>>8)&0xff)

#define DISP_GET_WORK_SUB_STATE(State) (((State)>>0)&0xff)

void DispIndicationEntry(unsigned char *pucData,int iLength);

void CcbInit(void);

DISPHANDLE DispCmdEntry(int iCmdId,unsigned char *pucData, int iLength);

int DispGetWorkState(void);

#ifdef __cplusplus
}
#endif

#endif

