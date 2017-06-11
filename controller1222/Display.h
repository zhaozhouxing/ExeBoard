#ifndef _DISPLAY_H_
#define _DISPLAY_H_

#include "datatype.h"
#include "cminterface.h"

#ifdef __cplusplus
 extern "C"
 {
#endif

typedef enum
{
     MACHINE_L_Genie = 0,
     
}MACHINE_TYPE_ENUM;


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

   DISP_CMD_CANCEL_WORK = 0x80,
   
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
    float SP1;//         0~1.6Mpa    B1进水压力下限
    float SP2;//          RO截留率下限,通过计算公式rejection=(I1b-I2)/I1b*100%
    float SP3;//          0~100?s/cm  RO产水电导率下限（I2测得）
    float SP4;//          0~18.2M .cm EDI产水电阻率下限（I3测得）
    float SP5;//          0~2.0m  设备恢复产水液位（B2）
    float SP6;//          0~2.0m  水箱低液位报警线（B2）
    float SP7;//          0~18.2M .cm UP产水电阻率下限（I5）
    float SP8;//          0~2.0m  源水箱补水液位（B3）
    float SP9;//          0~2.0m  原水箱低位系统保护设定点（B3）
    float SP10;//        0~30L/s RO产水流量下限（S2）
    float SP13;//        0~2000?s/cm 自来水电导率上限（I1a）
}DISP_MID_MACHINE_PARAM_STRU;

typedef struct
{
    float SP1;//          0~1.6Mpa    进水压力下限（B1）
    float SP2;//          RO截留率下限,通过计算公式rejection=(I1b-I2)/I1b*100%
    float SP3;//          0~100?s/cm  RO产水电导率下限（I2测得）
    float SP4;//          0~18.2M .cm EDI产水电阻率下限（I3测得）
    float SP5;//          0~2.0m  设备恢复产水液位（B2）
    float SP6;//          0~2.0m  水箱低液位报警线（B2）
    float SP7;//          0~18.2M .cm UP产水电阻率下限（I5）
    float SP11;//        0~18.2M .cm 水箱水水质上限（I4）
    float SP12;//        0~18.2M .cm 纯水取水水质下限（I4）
    float SP13;//        0~2000?s/cm 自来水电导率上限（I1a）

}DISP_DESK_MACHINE_PARAM_STRU;

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

    unsigned int ulInitRunT1;
    
    unsigned int ulNormRunT1;
    
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

