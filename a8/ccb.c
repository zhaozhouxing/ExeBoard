


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <dirent.h>
#include <libgen.h>
#include <signal.h>
#include <fcntl.h>
#include <errno.h>

#include "ccb.h"
#include "canItf.h"
#include "msg.h"
#include "messageitf.h"
#include "memory.h"
#include "cminterface.h"
#include "Interface.h"
#include "DefaultParams.h"
#include "log.h"


#ifdef __cplusplus
extern "C"
{
#endif

tls_key_t  pthread_tno_key;
threadpool gThdPool;

TIMER_PARAM_STRU gTps;

CCB gCcb;

int MSG_QUEUE_ID_MAIN ;
int MSG_QUEUE_ID_CANITF;

unsigned int gSecond = 0;

#define HEART_BEAT_PERIOD (20)

void WorkCommEntry(void *para);


void CcbDefaultModbusCallBack(int code,void *param)
{
    MODBUS_PACKET_COMM_STRU *pmg = (MODBUS_PACKET_COMM_STRU *)param;

    int iContLen = 0;

    if (ERROR_CODE_SUCC == code)
    {

        switch(pmg->ucFuncode)
        {
        case MODBUS_FUNC_CODE_Read_Coil_Status:
        case MODBUS_FUNC_CODE_Read_Input_Status:
        case MODBUS_FUNC_CODE_Read_Holding_Registers:
        case MODBUS_FUNC_CODE_Read_Input_Registers:
            iContLen = pmg->aucData[0] + 2;
            break;
        case MODBUS_FUNC_CODE_Force_Single_Coil:
        case MODBUS_FUNC_CODE_Preset_Single_Register:
        case MODBUS_FUNC_CODE_Force_Multiple_Coils:
        case MODBUS_FUNC_CODE_Preset_Multiple_Registers:
            iContLen = 5;
            break;
        case MODBUS_FUNC_CODE_Mask_Write_0X_Register:
        case MODBUS_FUNC_CODE_Mask_Write_4X_Register:
            iContLen = 7;
            break;
        }

        if (0 == iContLen)
        {
            return;
        }

        if (iContLen <= MAX_MODBUS_BUFFER_SIZE)
        {
            memcpy(gCcb.aucModbusBuffer,(unsigned char *)pmg,iContLen);
        }
    }
}

int CcbSndCanCmd(int iChl,unsigned int ulCanId,unsigned char ucCmd,unsigned char *data, int iPayLoadLen)
{
    int iRet;

    int iCanMsgLen = iPayLoadLen + RPC_FRAME_HDR_SZ + RPC_UART_FRAME_OVHD;

    unsigned char *sbBuf ;
    
    MAIN_CANITF_STRU *pMsg = (MAIN_CANITF_STRU *)SatAllocMsg(MAIN_CANITF_MSG_SIZE + iCanMsgLen);//malloc(sizeof(TIMER_MSG_STRU));
    if (pMsg)
    {
        unsigned char ucFcs = 0;
        int           iLen;
        int           iIdx;

        pMsg->msgHead.event = MAIN_CANITF_MSG;
        pMsg->ulCanId = ulCanId;
        pMsg->iCanChl = iChl;
        pMsg->iMsgLen = iCanMsgLen;
        pMsg->aucData[0] = RPC_UART_SOF;
        
        sbBuf = pMsg->aucData + 1;
        sbBuf[RPC_POS_LEN]  = iPayLoadLen; // iLen for data area (NOT INCLUDE CMD0&CMD1&LEN itself)
        sbBuf[RPC_POS_CMD0] = RPC_SYS_AF;
        sbBuf[RPC_POS_CMD1] = ucCmd;//SAPP_CMD_DATA;
        memcpy(&sbBuf[RPC_POS_DAT0],data,iPayLoadLen);
   
        iLen = sbBuf[RPC_POS_LEN] + RPC_FRAME_HDR_SZ;
        for ( iIdx = RPC_POS_LEN; iIdx < iLen; iIdx++)
        {
          ucFcs ^= sbBuf[iIdx];
        }
        sbBuf[iLen] = ucFcs;
       
        iRet = SndMsg(MSG_QUEUE_ID_CANITF,(SAT_MSG_HEAD *)pMsg);
        if (0 != iRet)
        {
            SatFreeMsg(pMsg);

            return -1;
        }

        return 0;
    }

    return -1;

}


void MainResetModulers(void)
{
     APP_PACKET_HOST_RESET_STRU Rst;

     unsigned int ulIdentifier;

     memset(&Rst,0,sizeof(APP_PACKET_HOST_RESET_STRU));

     Rst.hdr.ucDevType = APP_DEV_TYPE_HOST ;
     Rst.hdr.ucMsgType = APP_PACKET_COMM_HOST_RESET;

     CAN_BUILD_ADDRESS_IDENTIFIER(ulIdentifier,0x1,CAN_BROAD_CAST_ADDRESS);

     CcbSndCanCmd(0,ulIdentifier,SAPP_CMD_DATA,(unsigned char *)&Rst,sizeof(Rst));
     
}

void MainSndHeartBeat(void)
{
     APP_PACKET_HEART_BEAT_REQ_STRU Hbt;

     unsigned int ulIdentifier;

     memset(&Hbt,0,sizeof(APP_PACKET_HOST_RESET_STRU));

     Hbt.hdr.ucDevType = APP_DEV_TYPE_HOST ;
     Hbt.hdr.ucMsgType = APP_PACKET_COMM_HEART_BEAT;

     CAN_BUILD_ADDRESS_IDENTIFIER(ulIdentifier,0x1,CAN_BROAD_CAST_ADDRESS);

     CcbSndCanCmd(0,ulIdentifier,SAPP_CMD_DATA,(unsigned char *)&Hbt,sizeof(Hbt));
}

void MainInitInnerIpc(void)
{
    sp_thread_mutex_init(&gCcb.Ipc.mutex,NULL);
    sp_thread_cond_init( &gCcb.Ipc.cond, NULL );
}


void MainDeinitInnerIpc(void)
{
    sp_thread_mutex_destroy(&gCcb.Ipc.mutex);
    sp_thread_cond_destroy( &gCcb.Ipc.cond );
}

void MainInitMsg(void)
{
   int iLoop;
   memset(&gCcb,0,sizeof(gCcb));

   INIT_LIST_HEAD(&gCcb.WorkList);
   for (iLoop = 0; iLoop < APP_EXE_VALVE_NUM; iLoop++)
   {
       gCcb.ExeBrd.aValveObjs[iLoop].emDispObjType = APP_OBJ_VALVE;
       gCcb.ExeBrd.aValveObjs[iLoop].iDispObjId    = iLoop;
       gCcb.ExeBrd.aValveObjs[iLoop].iVChoice      = APP_OBJ_VALUE_U32;
   }

   for (iLoop = 0; iLoop < APP_EXE_PRESSURE_METER; iLoop++)
   {
       gCcb.ExeBrd.aPMObjs[iLoop].emDispObjType = APP_OBJ_B;
       gCcb.ExeBrd.aPMObjs[iLoop].iDispObjId    = iLoop;
       gCcb.ExeBrd.aPMObjs[iLoop].iVChoice      = APP_OBJ_VALUE_U32; /*ua*/
   }

   for (iLoop = 0; iLoop < APP_EXE_G_PUMP_NUM; iLoop++)
   {
       gCcb.ExeBrd.aGPumpObjs[iLoop].emDispObjType = APP_OBJ_N_PUMP;
       gCcb.ExeBrd.aGPumpObjs[iLoop].iDispObjId    = iLoop;
       gCcb.ExeBrd.aGPumpObjs[iLoop].iVChoice      = APP_OBJ_VALUE_U32; /*ua*/
   }

   for (iLoop = 0; iLoop < APP_EXE_R_PUMP_NUM; iLoop++)
   {
       gCcb.ExeBrd.aRPumpObjs[iLoop].emDispObjType = APP_OBJ_R_PUMP;
       gCcb.ExeBrd.aRPumpObjs[iLoop].iDispObjId    = iLoop;
       gCcb.ExeBrd.aRPumpObjs[iLoop].iVChoice      = APP_OBJ_VALUE_U32; 
   }

   for (iLoop = 0; iLoop < APP_EXE_RECT_NUM; iLoop++)
   {
       gCcb.ExeBrd.aRectObjs[iLoop].emDispObjType = APP_OBJ_RECT;
       gCcb.ExeBrd.aRectObjs[iLoop].iDispObjId    = iLoop;
       gCcb.ExeBrd.aRectObjs[iLoop].iVChoice      = APP_OBJ_VALUE_U32;
   }

   for (iLoop = 0; iLoop < APP_EXE_EDI_NUM; iLoop++)
   {
       gCcb.ExeBrd.aEDIObjs[iLoop].emDispObjType = APP_OBJ_EDI;
       gCcb.ExeBrd.aEDIObjs[iLoop].iDispObjId    = iLoop;
       gCcb.ExeBrd.aEDIObjs[iLoop].iVChoice      = APP_OBJ_VALUE_U32;
   }

   for (iLoop = 0; iLoop < APP_EXE_ECO_NUM; iLoop++)
   {
       gCcb.ExeBrd.aEcoObjs[iLoop].emDispObjType = APP_OBJ_I;
       gCcb.ExeBrd.aEcoObjs[iLoop].iDispObjId    = iLoop;
       gCcb.ExeBrd.aEcoObjs[iLoop].iVChoice      = APP_OBJ_VALUE_CUST;
   }

   for (iLoop = 0; iLoop < APP_FM_FLOW_METER_NUM; iLoop++)
   {
       gCcb.FlowMeter.aFmObjs[iLoop].emDispObjType = APP_OBJ_S;
       gCcb.FlowMeter.aFmObjs[iLoop].iDispObjId    = iLoop;
       gCcb.FlowMeter.aFmObjs[iLoop].iVChoice      = APP_OBJ_VALUE_U32;
   }
   
   // for ccb init
   gCcb.iMainWorkState = DISP_WORK_STATE_IDLE;
   gCcb.iSubWorkState  = DISP_WORK_SUB_IDLE;
   
   MainInitInnerIpc();

   sp_thread_sem_init(&gCcb.Sem4Delay,0,1); /* binary semphone */

   gCcb.Param.ulRoWashT1 = DEFAULT_RoWashT1;
   gCcb.Param.ulRoWashT2 = DEFAULT_RoWashT2;
   gCcb.Param.ulRoWashT3 = DEFAULT_RoWashT3;

   gCcb.Param.ulPhWashT1 = DEFAULT_PhWashT1;
   gCcb.Param.ulPhWashT2 = DEFAULT_PhWashT2;
   gCcb.Param.ulPhWashT3 = DEFAULT_PhWashT3;
   gCcb.Param.ulPhWashT4 = DEFAULT_PhWashT4;
   gCcb.Param.ulPhWashT5 = DEFAULT_PhWashT5;
   
   // set reset to all sub modulers
   MainResetModulers();
}

void MainDeInitMsg(void)
{
    MainDeinitInnerIpc();

    sp_thread_sem_destroy(&gCcb.Sem4Delay);

}


void Main_second_handler(void *arg)
{
    int id = (int )arg;
    int ret;
    TIMER_MSG_STRU *tm = (TIMER_MSG_STRU *)SatAllocMsg(TIMER_MSG_MSG_SIZE);//malloc(sizeof(TIMER_MSG_STRU));
    if (tm)
    {
        tm->id = id;
        tm->msgHead.event = TIMER_MSG;
        ret = SndMsg(MSG_QUEUE_ID_MAIN,(SAT_MSG_HEAD *)tm);
        if (0 != ret)
        {
            SatFreeMsg(tm);
        }
    }
}

void MainHeartBeatProc()
{
    int iLoop;
    if (gCcb.ulRegisterMask)
    {
       gCcb.iHbtCnt = (gCcb.iHbtCnt + 1) % (HEART_BEAT_PERIOD - 5);

       if (0 == gCcb.iHbtCnt)
       {
          for (iLoop = 0; iLoop < 20; iLoop++)
          {
              if ((1 << iLoop) & (gCcb.ulRegisterMask))
              {
                  if ((1 << iLoop) & (gCcb.ulActiveMask))
                  {
                      gCcb.ulActiveMask &= ~(1 << iLoop);
                      continue;
                  }
                  else
                  {
                      /* lost heart beat , raise alarm !*/
                  }
              }
          }
          MainSndHeartBeat();
       }
    } 
}

void MainSecondTask()
{
	gSecond++;

    //printf("MainSecondTask \r\n");
    MainHeartBeatProc();
}


void MainProcTimerMsg(char *pMsg, int nMsgLen)
{
    TIMER_MSG_STRU *pTimerMsg = (TIMER_MSG_STRU *)(pMsg);
    nMsgLen = nMsgLen; // mute complier's complain

    switch(pTimerMsg->id)
    {
    case TIMER_SECOND:
        MainSecondTask();
        break;
    default:
        break;
    }
}

int CanCcbAfModbusMsg(MAIN_CANITF_STRU *pCanItfMsg)
{
    MODBUS_PACKET_COMM_STRU *pmg = (MODBUS_PACKET_COMM_STRU *)&pCanItfMsg->aucData[1 + RPC_POS_DAT0]; 

    if (gCcb.ModbusCb)
    {
        (gCcb.ModbusCb)(0,pmg);
        sp_thread_mutex_lock  ( &gCcb.Ipc.mutex );
        sp_thread_cond_signal ( &gCcb.Ipc.cond  );// ylf: all thread killed
        sp_thread_mutex_unlock( &gCcb.Ipc.mutex );
    }
    
    return 0;
}


int CanCcbAfDataOnLineNotiIndMsg(MAIN_CANITF_STRU *pCanItfMsg)
{

    APP_PACKET_ONLINE_NOTI_IND_STRU *pmg = (APP_PACKET_ONLINE_NOTI_IND_STRU *)&pCanItfMsg->aucData[1 + RPC_POS_DAT0];

    int iSrcAdr = CAN_SRC_ADDRESS(pCanItfMsg->ulCanId);

    printf("OnLine Ind 0x%x\r\n",pCanItfMsg->ulCanId);
    
    switch(pmg->hdr.ucDevType)
    {
    case APP_DEV_TYPE_EXE_BOARD:
    case APP_DEV_TYPE_FLOW_METER:
        if (iSrcAdr != pmg->hdr.ucDevType)
        {
            return -1;
        }
        break;
    case APP_DEV_TYPE_HAND_SET:
        if (iSrcAdr < APP_HAND_SET_BEGIN_ADDRESS)
        {
            return -1;
        }
        break;
    default:
        printf("unknow dev type %x\r\n",pmg->hdr.ucDevType);
        return -1;
    }
    
    gCcb.ulRegisterMask |= (1 << iSrcAdr);
    gCcb.ulActiveMask   |= (1 << iSrcAdr);

    /* Send response message */
    {
        APP_PACKET_ONLINE_NOTI_CONF_STRU NotiCnf;

        unsigned int ulIdentifier ;

        NotiCnf.hdr.ucLen     = APP_POROTOL_PACKET_ONLINE_NOTI_CNF_PAYLOAD_LENGTH;
        NotiCnf.hdr.ucTransId = pmg->hdr.ucTransId;
        NotiCnf.hdr.ucDevType = APP_DEV_TYPE_MAIN_CTRL;
        NotiCnf.hdr.ucMsgType = pmg->hdr.ucMsgType|0X80;
        NotiCnf.usHeartBeatPeriod = HEART_BEAT_PERIOD*1000;

        CAN_BUILD_ADDRESS_IDENTIFIER(ulIdentifier,APP_DEV_TYPE_MAIN_CTRL,iSrcAdr);

        CcbSndCanCmd(0,ulIdentifier,SAPP_CMD_DATA,(unsigned char *)&NotiCnf,sizeof(NotiCnf));

    }

    return 0;
}

int CanCcbAfDataHeartBeatRspMsg(MAIN_CANITF_STRU *pCanItfMsg)
{
    APP_PACKET_ONLINE_NOTI_CONF_STRU *pmg = (APP_PACKET_ONLINE_NOTI_CONF_STRU *)&pCanItfMsg->aucData[1 + RPC_POS_DAT0]; 

    int iSrcAdr = CAN_SRC_ADDRESS(pCanItfMsg->ulCanId);

    printf("Hbt response 0x%x\r\n",pCanItfMsg->ulCanId);
    
    switch(pmg->hdr.ucDevType)
    {
    case APP_DEV_TYPE_EXE_BOARD:
    case APP_DEV_TYPE_FLOW_METER:
        if (iSrcAdr != pmg->hdr.ucDevType)
        {
            return -1;
        }
        break;
    case APP_DEV_TYPE_HAND_SET:
        if (iSrcAdr < APP_HAND_SET_BEGIN_ADDRESS)
        {
            return -1;
        }
        break;
    default:
        printf("unknow dev type %x\r\n",pmg->hdr.ucDevType);
        return -1;
    }
    
    gCcb.ulActiveMask   |= (1 << iSrcAdr);

    return 0;
}


int CanCcbAfDataClientRpt4ExeBoard(MAIN_CANITF_STRU *pCanItfMsg)
{
    APP_PACKET_CLIENT_RPT_IND_STRU *pmg = (APP_PACKET_CLIENT_RPT_IND_STRU *)&pCanItfMsg->aucData[1 + RPC_POS_DAT0];

    switch(pmg->ucRptType)
    {
    case APP_PACKET_RPT_ECO:
        {
            int iPackets = (pmg->hdr.ucLen - 1) / sizeof (APP_PACKET_RPT_ECO_STRU);
            APP_PACKET_RPT_ECO_STRU *pEco = (APP_PACKET_RPT_ECO_STRU *)pmg->aucData;
            int iLoop;
            
            for (iLoop = 0; iLoop < iPackets; iLoop++,pEco++ )
            {
                if (pEco->ucId < APP_EXE_ECO_NUM)
                {
                    gCcb.ExeBrd.aEcoObjs[pEco->ucId].Value.eV.fWaterQ = pEco->ev.fWaterQ;
                    gCcb.ExeBrd.aEcoObjs[pEco->ucId].Value.eV.usTemp = pEco->ev.usTemp;
                    gCcb.ExeBrd.ulEcoValidFlags |= 1 << iLoop;
                }
            }
        }
        break;
    case APP_PACKET_MT_STATUS:
        break;
    }
    return 0;
}



int CanCcbAfDataClientRpt4FlowMeter(MAIN_CANITF_STRU *pCanItfMsg)
{
    APP_PACKET_CLIENT_RPT_IND_STRU *pmg = (APP_PACKET_CLIENT_RPT_IND_STRU *)&pCanItfMsg->aucData[1 + RPC_POS_DAT0];

    switch(pmg->ucRptType)
    {
    case APP_PACKET_RPT_FM:
         {
            int iPackets = (pmg->hdr.ucLen - 1) / sizeof (APP_PACKET_RPT_FM_STRU);
            APP_PACKET_RPT_FM_STRU *pFM = (APP_PACKET_RPT_FM_STRU *)pmg->aucData;
            int iLoop;
            
            for (iLoop = 0; iLoop < iPackets; iLoop++,pFM++ )
            {
                if (pFM->ucId < APP_FM_FLOW_METER_NUM)
                {
                    gCcb.FlowMeter.aFmObjs[pFM->ucId].Value.ulV = pFM->ulValue;
                    gCcb.FlowMeter.ulFmValidFlags |= 1 << iLoop;
                }
            }
        }
        break;
    default:
        break;
    }

    return 0;
}


int CanCcbAfDataClientRptMsg(MAIN_CANITF_STRU *pCanItfMsg)
{

    APP_PACKET_CLIENT_RPT_IND_STRU *pmg = (APP_PACKET_CLIENT_RPT_IND_STRU *)&pCanItfMsg->aucData[1 + RPC_POS_DAT0];

    int iSrcAdr = CAN_SRC_ADDRESS(pCanItfMsg->ulCanId);

    /* message validation  */
    switch(pmg->hdr.ucDevType)
    {
    case APP_DEV_TYPE_EXE_BOARD:
    case APP_DEV_TYPE_FLOW_METER:
        if (iSrcAdr != pmg->hdr.ucDevType)
        {
            return -1;
        }
        break;
    default:
        printf("unknow dev type %x\r\n",pmg->hdr.ucDevType);
        return -1;
    }

    switch(pmg->hdr.ucDevType)
    {
    case APP_DEV_TYPE_EXE_BOARD:
        return CanCcbAfDataClientRpt4ExeBoard(pCanItfMsg);
    case APP_DEV_TYPE_FLOW_METER:
        return CanCcbAfDataClientRpt4FlowMeter(pCanItfMsg);
    default:
        break;
    }

    return 0;
}

int CanCcbAfDataHOCirReqMsg(MAIN_CANITF_STRU *pCanItfMsg)
{
    APP_PACKET_HO_STRU *pmg = (APP_PACKET_HO_STRU *)&pCanItfMsg->aucData[1 + RPC_POS_DAT0]; 

    APP_PACKET_HO_CIR_REQ_STRU *pCirReq = (APP_PACKET_HO_CIR_REQ_STRU *)pmg->aucData;

    uint8_t ucResult = 0;

    switch(pCirReq->ucAction)
    {
    case APP_PACKET_HO_ACTION_START:
        {
            /* late implement */
        }
        break;
    default:
        ucResult = 1; // failed
        break;
    }

    {
        /* send response */
        unsigned char buf[32];

        unsigned int ulCanId;

        int iPayLoad;

        APP_PACKET_HO_STRU *pHoRsp = (APP_PACKET_HO_STRU *)buf;
        APP_PACKET_HO_CIR_RSP_STRU *pCirRsp = (APP_PACKET_HO_CIR_RSP_STRU *)pHoRsp->aucData;

        pHoRsp->hdr = pmg->hdr;

        pHoRsp->hdr.ucLen     = sizeof(APP_PACKET_HO_CIR_RSP_STRU) + 1; // 1 for ops type
        pHoRsp->hdr.ucDevType = APP_DEV_TYPE_MAIN_CTRL;
        pHoRsp->hdr.ucMsgType |= 0x80;
        pHoRsp->ucOpsType     = pmg->ucOpsType;

        pCirRsp->ucAction     = pCirReq->ucAction;
        pCirRsp->ucResult     = ucResult;

        iPayLoad = APP_POROTOL_PACKET_HO_COMMON_LENGTH + sizeof(APP_PACKET_HO_CIR_RSP_STRU);

        CAN_BUILD_ADDRESS_IDENTIFIER(ulCanId,0x1,CAN_SRC_ADDRESS(pCanItfMsg->ulCanId));

        CcbSndCanCmd(pCanItfMsg->iCanChl,ulCanId,SAPP_CMD_DATA,buf,iPayLoad);

    }
    return 0;
}
int CanCcbAfDataHOCirRspMsg(MAIN_CANITF_STRU *pCanItfMsg)
{
    /* do nothing here */
    pCanItfMsg = pCanItfMsg;
    return 0;
}

int CanCcbAfDataHOQtwReqMsg(MAIN_CANITF_STRU *pCanItfMsg)
{

    APP_PACKET_HO_STRU *pmg = (APP_PACKET_HO_STRU *)&pCanItfMsg->aucData[1 + RPC_POS_DAT0]; 

    APP_PACKET_HO_QTW_REQ_STRU *pQtwReq = (APP_PACKET_HO_QTW_REQ_STRU *)pmg->aucData;

    uint8_t ucResult = 0;

    switch(pQtwReq->ucAction)
    {
    case APP_PACKET_HO_ACTION_START:
        {
            /* late implement */
        }
        break;
    case APP_PACKET_HO_ACTION_STOP:
        {
            /* late implement */
        }
        break;
    default:
        ucResult = 1; // failed
        break;
    }

    {
        /* send response */
        unsigned char buf[32];

        unsigned int ulCanId;
        int iPayLoad;

        APP_PACKET_HO_STRU *pHoRsp = (APP_PACKET_HO_STRU *)buf;
        APP_PACKET_HO_QTW_RSP_STRU *pQtwRsp = (APP_PACKET_HO_QTW_RSP_STRU *)pHoRsp->aucData;

        pHoRsp->hdr = pmg->hdr;

        pHoRsp->hdr.ucLen     = sizeof(APP_PACKET_HO_QTW_RSP_STRU) + 1; // 1 for ops type
        pHoRsp->hdr.ucDevType = APP_DEV_TYPE_MAIN_CTRL;
        pHoRsp->hdr.ucMsgType |= 0x80;
        pHoRsp->ucOpsType     = pmg->ucOpsType;

        pQtwRsp->ucAction     = pQtwReq->ucAction;
        pQtwRsp->ucResult     = ucResult;

        iPayLoad = APP_POROTOL_PACKET_HO_COMMON_LENGTH + sizeof(APP_PACKET_HO_QTW_RSP_STRU);

        CAN_BUILD_ADDRESS_IDENTIFIER(ulCanId,0x1,CAN_SRC_ADDRESS(pCanItfMsg->ulCanId));

        CcbSndCanCmd(pCanItfMsg->iCanChl,ulCanId,SAPP_CMD_DATA,buf,iPayLoad);

    }

    return 0;
}
int CanCcbAfDataHOQtwRspMsg(MAIN_CANITF_STRU *pCanItfMsg)
{

    pCanItfMsg = pCanItfMsg;

    /* do nothing here */

    return 0;
}


int CanCcbAfDataHandleOpsMsg(MAIN_CANITF_STRU *pCanItfMsg)
{
    APP_PACKET_HO_STRU *pmg = (APP_PACKET_HO_STRU *)&pCanItfMsg->aucData[1 + RPC_POS_DAT0]; 

    switch(pmg->ucOpsType)
    {
    case APP_PACKET_HO_CIR:
        if (!(pmg->hdr.ucMsgType & 0x80))
        {
            return CanCcbAfDataHOCirReqMsg(pCanItfMsg);
        }
        return CanCcbAfDataHOCirRspMsg(pCanItfMsg);
    case APP_PACKET_HO_QTW:
        if (!(pmg->hdr.ucMsgType & 0x80))
        {
            return CanCcbAfDataHOQtwReqMsg(pCanItfMsg);
        }
        return CanCcbAfDataHOQtwRspMsg(pCanItfMsg);
    }
    return 0;
}



int CanCcbAfDataStateIndMsg(MAIN_CANITF_STRU *pCanItfMsg)
{
    APP_PACKET_STATE_IND_STRU *pmg = (APP_PACKET_STATE_IND_STRU *)&pCanItfMsg->aucData[1 + RPC_POS_DAT0]; 

    {
        int iPackets = (pmg->hdr.ucLen ) / sizeof (APP_PACKET_STATE_STRU);
        APP_PACKET_STATE_STRU *pState = (APP_PACKET_STATE_STRU *)pmg->aucData;
        int iLoop;
        
        for (iLoop = 0; iLoop < iPackets; iLoop++,pState++ )
        {
            switch(pState->ucType)
            {
            case APP_OBJ_N_PUMP:   
                if (pState->ucId < APP_EXE_G_PUMP_NUM)
                {
                    gCcb.ExeBrd.aGPumpObjs[pState->ucId].iState = !!pState->ucState;
                }
                break;
            case APP_OBJ_R_PUMP:   
                if (pState->ucId < APP_EXE_R_PUMP_NUM)
                {
                    gCcb.ExeBrd.aRPumpObjs[pState->ucId].iState = !!pState->ucState;
                }
                break;
            }
        }
    }

    return 0;
}


int CanCcbAfDataMsg(MAIN_CANITF_STRU *pCanItfMsg)
{
    APP_PACKET_COMM_STRU *pmg = (APP_PACKET_COMM_STRU *)&pCanItfMsg->aucData[1 + RPC_POS_DAT0]; 
    switch((pmg->ucMsgType & 0x7f))
    {
    case APP_PAKCET_COMM_ONLINE_NOTI:
        if (!(pmg->ucMsgType & 0x80))
        {
            return CanCcbAfDataOnLineNotiIndMsg(pCanItfMsg);
        }
        break;
    case APP_PACKET_COMM_HEART_BEAT:
        if ((pmg->ucMsgType & 0x80))
        {
            return CanCcbAfDataHeartBeatRspMsg(pCanItfMsg);
        }
        break;
    case APP_PACKET_CLIENT_REPORT:
        CanCcbAfDataClientRptMsg(pCanItfMsg);
        break;
    case APP_PACKET_HAND_OPERATION:
        CanCcbAfDataHandleOpsMsg(pCanItfMsg);
        break;
    case APP_PACKET_STATE_INDICATION:
        CanCcbAfDataStateIndMsg(pCanItfMsg);
        break;
    default: // yet to be implemented
        break;
    }

    return 0;
}


void CanCcbAfProc(MAIN_CANITF_STRU *pCanItfMsg)
{
    switch((pCanItfMsg->aucData[1 + RPC_POS_CMD1] & 0X7F))
    {
    case SAPP_CMD_DATA:
        CanCcbAfDataMsg(pCanItfMsg);
        break;
    case SAPP_CMD_MODBUS:
        CanCcbAfModbusMsg(pCanItfMsg);
        break;
    }
}

void MainProcCanItfMsg(char *pucMsg, int nMsgLen)
{
    MAIN_CANITF_STRU *pCanItfMsg = (MAIN_CANITF_STRU *)pucMsg;

    nMsgLen = nMsgLen; // mute complier's complain

    printf("can msg 0x%x: CM0=%x&CM1=%x\r\n",pCanItfMsg->ulCanId,pCanItfMsg->aucData[1 + RPC_POS_CMD0],pCanItfMsg->aucData[1 + RPC_POS_CMD1]);

    // do something, later implement
    switch(pCanItfMsg->aucData[1 + RPC_POS_CMD0])
    {
    case RPC_SYS_AF:
        CanCcbAfProc(pCanItfMsg);
        break;
    default:
        break;
    }
}


void MainMsgProc(char *pMsg, int nMsgLen, int nEvent)
{
    switch(nEvent)
    {
    case INIT_ALL_THREAD_EVENT:
        MainInitMsg();
        break;
    case TIMER_MSG:
        MainProcTimerMsg(pMsg,nMsgLen);
        break;
    case CANITF_MAIN_MSG:
        MainProcCanItfMsg(pMsg,nMsgLen);
        break;
    case QUIT_ALL_THREAD_EVENT:
        MainDeInitMsg();
        break;
    default:
        break;
   }
}

void MainProc(void* lpParam)
{
    int         recvlen;
    int         minSize = sizeof(SAT_MSG_HEAD);

    SAT_MSG_HEAD        *pMsgHead;

    tls_setkey(&pthread_tno_key,(int)lpParam);

    for(;;)
    {
        recvlen = msg_recv(MSG_QUEUE_ID_MAIN, (char**)&pMsgHead, NULL);
        if (recvlen < minSize) 
        {
            continue;
        }
        
        MainMsgProc((char*)pMsgHead, pMsgHead->len, pMsgHead->event);

        if (QUIT_ALL_THREAD_EVENT == pMsgHead->event)
        {
            SatFreeMsg(pMsgHead);    
            break;
        }
        else
        {
            SatFreeMsg(pMsgHead);
        }

    }

}

int CcbModbusWorkEntry(int iChl,unsigned int ulCanId,unsigned char *pucModbus,int iPayLoad,int iTime,modbus_call_back cb)
{
    int iRet;
    
    struct timeval now;
    struct timespec outtime;
    
    sp_thread_mutex_lock( &gCcb.Ipc.mutex );
    
    iRet = CcbSndCanCmd(iChl,ulCanId,SAPP_CMD_MODBUS,pucModbus,iPayLoad);
    if (0 != iRet)
    {
        sp_thread_mutex_unlock( &gCcb.Ipc.mutex );
        
        return -1;
    }

    gettimeofday(&now, NULL);
    outtime.tv_sec  = now.tv_sec + iTime/1000;
    outtime.tv_nsec = (now.tv_usec + (iTime  % 1000)*1000)* 1000;

    gCcb.ModbusCb = cb;

    iRet = sp_thread_cond_timedwait( &gCcb.Ipc.cond, &gCcb.Ipc.mutex ,&outtime);//ylf: thread sleep here
    if(ETIMEDOUT == iRet)
    {
        sp_thread_mutex_unlock( &gCcb.Ipc.mutex );
        if (gCcb.ModbusCb )(gCcb.ModbusCb)(iRet,NULL);
        gCcb.ModbusCb = NULL;
        return -1;
    }

    gCcb.ModbusCb = NULL;
    sp_thread_mutex_unlock( &gCcb.Ipc.mutex );  
    
    return 0; // success
}


void CcbDelayCallBack(void *para)
{
    CCB *pCcb = (CCB *)para;

    sp_thread_sem_post(&pCcb->Sem4Delay);

}

int CcbWorkDelayEntry(unsigned int ulTime,sys_timeout_handler cb)
{
    gCcb.to4Delay = timer_add(ulTime,TIMER_ONE_SHOT,cb,&gCcb);

    if (gCcb.to4Delay)
    {
        sp_thread_sem_pend(&gCcb.Sem4Delay);
        
        return 0;
    }

    return -1; // success
}


// dispatch_threadpool(gThdPool,Batch4TestImgProc,NULL);

void work_init_run(void *para)
{
    CCB *pCcb = (CCB *)para;

    /* do something here */
}

void work_idle(void *para)
{
    CCB *pCcb = (CCB *)para;

    unsigned char aucModbusBuf[32];

    int iIdx = 0;

    int iTmp;

    unsigned int ulIdentifier;

    MODBUS_PACKET_COMM_STRU *pModBus = (MODBUS_PACKET_COMM_STRU *)aucModbusBuf;

    /* notify ui (late implemnt) */

    /* shutdown everything */
    pModBus->ucFuncode = MODBUS_FUNC_CODE_Force_Multiple_Coils;

    /* 1.Start Address */
    pModBus->aucData[iIdx++] = (0 >> 8) & 0XFF; 
    pModBus->aucData[iIdx++] = (0 >> 0) & 0XFF;

   /* 2.Reg Numbers */
   iTmp = APP_EXE_VALVE_NUM + APP_EXE_G_PUMP_NUM + APP_EXE_R_PUMP_NUM + APP_EXE_EDI_NUM + APP_EXE_RECT_NUM;
   pModBus->aucData[iIdx++] = (iTmp >> 8) & 0XFF;
   pModBus->aucData[iIdx++] = (iTmp >> 0) & 0XFF;

   /* 3. data counts */
   pModBus->aucData[iIdx++] = 2;

   /* close all valves */
   iTmp = 0;
   pModBus->aucData[iIdx++] = (iTmp >> 8) & 0XFF;
   pModBus->aucData[iIdx++] = (iTmp >> 0) & 0XFF;

   CAN_BUILD_ADDRESS_IDENTIFIER(ulIdentifier,APP_PAKCET_ADDRESS_MC,APP_PAKCET_ADDRESS_EXE);

   CcbModbusWorkEntry(0,ulIdentifier,aucModbusBuf,iIdx + 1,2000,CcbDefaultModbusCallBack); // don care modbus exe result

    /* disable all reports from exe */
    iIdx = 0;
    pModBus->ucFuncode = MODBUS_FUNC_CODE_Preset_Single_Register;
   
    /* 1.Start Address */
    pModBus->aucData[iIdx++] = (0 >> 8) & 0XFF; 
    pModBus->aucData[iIdx++] = (0 >> 0) & 0XFF;
   
  
   /* close all report */
   iTmp = 0;
   pModBus->aucData[iIdx++] = (iTmp >> 8) & 0XFF;
   pModBus->aucData[iIdx++] = (iTmp >> 0) & 0XFF;
   
   CAN_BUILD_ADDRESS_IDENTIFIER(ulIdentifier,APP_PAKCET_ADDRESS_MC,APP_PAKCET_ADDRESS_EXE);
   
   CcbModbusWorkEntry(0,ulIdentifier,aucModbusBuf,iIdx + 1,2000,CcbDefaultModbusCallBack); // don care modbus exe result
   
    /* disable all reports from fm */
    iIdx = 0;
    pModBus->ucFuncode = MODBUS_FUNC_CODE_Preset_Single_Register;
   
    /* 1.Start Address */
    pModBus->aucData[iIdx++] = (0 >> 8) & 0XFF; 
    pModBus->aucData[iIdx++] = (0 >> 0) & 0XFF;
   
  
   /* close all report */
   iTmp = 0;
   pModBus->aucData[iIdx++] = (iTmp >> 8) & 0XFF;
   pModBus->aucData[iIdx++] = (iTmp >> 0) & 0XFF;
   
   CAN_BUILD_ADDRESS_IDENTIFIER(ulIdentifier,APP_PAKCET_ADDRESS_MC,APP_PAKCET_ADDRESS_FM);
   
   CcbModbusWorkEntry(0,ulIdentifier,aucModbusBuf,iIdx + 1,2000,CcbDefaultModbusCallBack); // don care modbus exe result
   
   /* notify hand set (late implement) */     

   /* notify ui (late implemnt) */
      
}


void work_idle_rowash(void *para)
{
    CCB *pCcb = (CCB *)para;

    unsigned char aucModbusBuf[32];

    int iIdx = 0;

    int iTmp;

    int iRet;

    unsigned int ulIdentifier;

    MODBUS_PACKET_COMM_STRU *pModBus = (MODBUS_PACKET_COMM_STRU *)aucModbusBuf;

    /* notify ui (late implemnt) */

     /* enable S4 reports from fm */
     iIdx = 0;
     pModBus->ucFuncode = MODBUS_FUNC_CODE_Preset_Single_Register;
    
     /* 1.Start Address */
     pModBus->aucData[iIdx++] = (0 >> 8) & 0XFF; 
     pModBus->aucData[iIdx++] = (0 >> 0) & 0XFF;
    
    
    /* enable S4 report */
    iTmp = 1 << APP_FM_FM4_NO;
    pModBus->aucData[iIdx++] = (iTmp >> 8) & 0XFF;
    pModBus->aucData[iIdx++] = (iTmp >> 0) & 0XFF;
    
    CAN_BUILD_ADDRESS_IDENTIFIER(ulIdentifier,APP_PAKCET_ADDRESS_MC,APP_PAKCET_ADDRESS_FM);

    
    iRet = CcbModbusWorkEntry(0,ulIdentifier,aucModbusBuf,iIdx + 1,2000,CcbDefaultModbusCallBack); // don care modbus exe result
    
    if (iRet )
    {
        VOS_LOG(VOS_LOG_WARNING,"CcbModbusWorkEntry Fail %d",iRet);    
        /* notify ui (late implemnt) */
        return ;
    }
    
    /* enable E1 、E2 、E9、C3 ON everything */
    pModBus->ucFuncode = MODBUS_FUNC_CODE_Force_Multiple_Coils;

    /* 1.Start Address */
    pModBus->aucData[iIdx++] = (0 >> 8) & 0XFF; 
    pModBus->aucData[iIdx++] = (0 >> 0) & 0XFF;

   /* 2.Reg Numbers */
   iTmp = APP_EXE_VALVE_NUM + APP_EXE_G_PUMP_NUM + APP_EXE_R_PUMP_NUM + APP_EXE_EDI_NUM + APP_EXE_RECT_NUM;
   pModBus->aucData[iIdx++] = (iTmp >> 8) & 0XFF;
   pModBus->aucData[iIdx++] = (iTmp >> 0) & 0XFF;

   /* 3.data counts */
   pModBus->aucData[iIdx++] = 2;

   /* 4.valves */
   iTmp = (1 << APP_EXE_E1_NO)|(1<<APP_EXE_E2_NO)|(1<<APP_EXE_E9_NO)|(1<<APP_EXE_C3_NO);
   pModBus->aucData[iIdx++] = (iTmp >> 8) & 0XFF;
   pModBus->aucData[iIdx++] = (iTmp >> 0) & 0XFF;

   CAN_BUILD_ADDRESS_IDENTIFIER(ulIdentifier,APP_PAKCET_ADDRESS_MC,APP_PAKCET_ADDRESS_EXE);

   iRet = CcbModbusWorkEntry(0,ulIdentifier,aucModbusBuf,iIdx + 1,2000,CcbDefaultModbusCallBack); // don care modbus exe result
   if (iRet )
   {
       VOS_LOG(VOS_LOG_WARNING,"CcbModbusWorkEntry Fail %d",iRet);    
   
       /* notify ui (late implemnt) */
       return ;
   }

   iRet = CcbWorkDelayEntry(gCcb.Param.ulRoWashT1,CcbDelayCallBack);
   if (iRet )
   {
       VOS_LOG(VOS_LOG_WARNING,"CcbWorkDelayEntry Fail %d",iRet);    
   
       /* notify ui (late implemnt) */
       return ;
   }

   /* enable E1��E2、E9、C1、C3 ON */
   pModBus->ucFuncode = MODBUS_FUNC_CODE_Force_Multiple_Coils;
   
   /* 1.Start Address */
   pModBus->aucData[iIdx++] = (0 >> 8) & 0XFF; 
   pModBus->aucData[iIdx++] = (0 >> 0) & 0XFF;
   
   /* 2.Reg Numbers */
   iTmp = APP_EXE_VALVE_NUM + APP_EXE_G_PUMP_NUM + APP_EXE_R_PUMP_NUM + APP_EXE_EDI_NUM + APP_EXE_RECT_NUM;
   pModBus->aucData[iIdx++] = (iTmp >> 8) & 0XFF;
   pModBus->aucData[iIdx++] = (iTmp >> 0) & 0XFF;
   
   /* 3.data counts */
   pModBus->aucData[iIdx++] = 2;
   
   /* 4.valves */
   iTmp = (1 << APP_EXE_E1_NO)|(1<<APP_EXE_E9_NO)|(1<<APP_EXE_C3_NO)|(1<<APP_EXE_C1_NO);
   pModBus->aucData[iIdx++] = (iTmp >> 8) & 0XFF;
   pModBus->aucData[iIdx++] = (iTmp >> 0) & 0XFF;
   
   CAN_BUILD_ADDRESS_IDENTIFIER(ulIdentifier,APP_PAKCET_ADDRESS_MC,APP_PAKCET_ADDRESS_EXE);
   
   iRet = CcbModbusWorkEntry(0,ulIdentifier,aucModbusBuf,iIdx + 1,2000,CcbDefaultModbusCallBack); // don care modbus exe result
   if (iRet )
   {
       VOS_LOG(VOS_LOG_WARNING,"CcbModbusWorkEntry Fail %d",iRet);    
   
      /* notify ui (late implemnt) */
      return ;
   }
   
   iRet = CcbWorkDelayEntry(gCcb.Param.ulRoWashT2,CcbDelayCallBack);
   if (iRet )
   {
       VOS_LOG(VOS_LOG_WARNING,"CcbModbusWorkEntry Fail %d",iRet);    
       /* notify ui (late implemnt) */
       return ;
   }

   /* enable E1、E9、C1、C3 ON
   ON */
   pModBus->ucFuncode = MODBUS_FUNC_CODE_Force_Multiple_Coils;
   
   /* 1.Start Address */
   pModBus->aucData[iIdx++] = (0 >> 8) & 0XFF; 
   pModBus->aucData[iIdx++] = (0 >> 0) & 0XFF;
   
   /* 2.Reg Numbers */
   iTmp = APP_EXE_VALVE_NUM + APP_EXE_G_PUMP_NUM + APP_EXE_R_PUMP_NUM + APP_EXE_EDI_NUM + APP_EXE_RECT_NUM;
   pModBus->aucData[iIdx++] = (iTmp >> 8) & 0XFF;
   pModBus->aucData[iIdx++] = (iTmp >> 0) & 0XFF;
   
   /* 3.data counts */
   pModBus->aucData[iIdx++] = 2;
   
   /* 4.valves */
   iTmp = (1 << APP_EXE_E1_NO)|(1<<APP_EXE_E9_NO)|(1<<APP_EXE_C3_NO);
   pModBus->aucData[iIdx++] = (iTmp >> 8) & 0XFF;
   pModBus->aucData[iIdx++] = (iTmp >> 0) & 0XFF;
   
   CAN_BUILD_ADDRESS_IDENTIFIER(ulIdentifier,APP_PAKCET_ADDRESS_MC,APP_PAKCET_ADDRESS_EXE);
   
   iRet = CcbModbusWorkEntry(0,ulIdentifier,aucModbusBuf,iIdx + 1,2000,CcbDefaultModbusCallBack); // don care modbus exe result
   if (iRet )
   {
       VOS_LOG(VOS_LOG_WARNING,"CcbModbusWorkEntry Fail %d",iRet);    
      /* notify ui (late implemnt) */
      return ;
   }
   
   iRet = CcbWorkDelayEntry(gCcb.Param.ulRoWashT3,CcbDelayCallBack);
   if (iRet )
   {
       VOS_LOG(VOS_LOG_WARNING,"CcbWorkDelayEntry Fail %d",iRet);    
       /* notify ui (late implemnt) */
       return ;
   }

   /* go to idle */
   /* shutdown everything */
   pModBus->ucFuncode = MODBUS_FUNC_CODE_Force_Multiple_Coils;
   
   /* 1.Start Address */
   pModBus->aucData[iIdx++] = (0 >> 8) & 0XFF; 
   pModBus->aucData[iIdx++] = (0 >> 0) & 0XFF;
   
   /* 2.Reg Numbers */
   iTmp = APP_EXE_VALVE_NUM + APP_EXE_G_PUMP_NUM + APP_EXE_R_PUMP_NUM + APP_EXE_EDI_NUM + APP_EXE_RECT_NUM;
   pModBus->aucData[iIdx++] = (iTmp >> 8) & 0XFF;
   pModBus->aucData[iIdx++] = (iTmp >> 0) & 0XFF;
   
   /* 3. data counts */
   pModBus->aucData[iIdx++] = 2;
   
   /* close all valves */
   iTmp = 0;
   pModBus->aucData[iIdx++] = (iTmp >> 8) & 0XFF;
   pModBus->aucData[iIdx++] = (iTmp >> 0) & 0XFF;
   
   CAN_BUILD_ADDRESS_IDENTIFIER(ulIdentifier,APP_PAKCET_ADDRESS_MC,APP_PAKCET_ADDRESS_EXE);
   
   CcbModbusWorkEntry(0,ulIdentifier,aucModbusBuf,iIdx + 1,2000,CcbDefaultModbusCallBack); // don care modbus exe result

   /* notify ui (late implement) */
      
}

void work_idle_phwash(void *para)
{
    CCB *pCcb = (CCB *)para;

    unsigned char aucModbusBuf[32];

    int iIdx = 0;

    int iTmp;

    int iRet;

    int iLoop;

    unsigned int ulIdentifier;

    MODBUS_PACKET_COMM_STRU *pModBus = (MODBUS_PACKET_COMM_STRU *)aucModbusBuf;

    /* notify ui (late implemnt) */

     /* enable S4 reports from fm */
     iIdx = 0;
     pModBus->ucFuncode = MODBUS_FUNC_CODE_Preset_Single_Register;
    
     /* 1.Start Address */
     pModBus->aucData[iIdx++] = (0 >> 8) & 0XFF; 
     pModBus->aucData[iIdx++] = (0 >> 0) & 0XFF;
    
    
    /* enable S4 report */
    iTmp = 1 << APP_FM_FM4_NO;
    pModBus->aucData[iIdx++] = (iTmp >> 8) & 0XFF;
    pModBus->aucData[iIdx++] = (iTmp >> 0) & 0XFF;
    
    CAN_BUILD_ADDRESS_IDENTIFIER(ulIdentifier,APP_PAKCET_ADDRESS_MC,APP_PAKCET_ADDRESS_FM);

    
    iRet = CcbModbusWorkEntry(0,ulIdentifier,aucModbusBuf,iIdx + 1,2000,CcbDefaultModbusCallBack); // don care modbus exe result
    
    if (iRet )
    {
        VOS_LOG(VOS_LOG_WARNING,"CcbModbusWorkEntry Fail %d",iRet);    
        /* notify ui (late implemnt) */
        return ;
    }
    
    /* enable E1 、E2 、E9、C3 ON  */
    pModBus->ucFuncode = MODBUS_FUNC_CODE_Force_Multiple_Coils;

    /* 1.Start Address */
    pModBus->aucData[iIdx++] = (0 >> 8) & 0XFF; 
    pModBus->aucData[iIdx++] = (0 >> 0) & 0XFF;

   /* 2.Reg Numbers */
   iTmp = APP_EXE_VALVE_NUM + APP_EXE_G_PUMP_NUM + APP_EXE_R_PUMP_NUM + APP_EXE_EDI_NUM + APP_EXE_RECT_NUM;
   pModBus->aucData[iIdx++] = (iTmp >> 8) & 0XFF;
   pModBus->aucData[iIdx++] = (iTmp >> 0) & 0XFF;

   /* 3.data counts */
   pModBus->aucData[iIdx++] = 2;

   /* 4.valves */
   iTmp = (1 << APP_EXE_E1_NO)|(1<<APP_EXE_E2_NO)|(1<<APP_EXE_E9_NO)|(1<<APP_EXE_C3_NO);
   pModBus->aucData[iIdx++] = (iTmp >> 8) & 0XFF;
   pModBus->aucData[iIdx++] = (iTmp >> 0) & 0XFF;

   CAN_BUILD_ADDRESS_IDENTIFIER(ulIdentifier,APP_PAKCET_ADDRESS_MC,APP_PAKCET_ADDRESS_EXE);

   iRet = CcbModbusWorkEntry(0,ulIdentifier,aucModbusBuf,iIdx + 1,2000,CcbDefaultModbusCallBack); // don care modbus exe result
   if (iRet )
   {
       VOS_LOG(VOS_LOG_WARNING,"CcbModbusWorkEntry Fail %d",iRet);    
       /* notify ui (late implemnt) */
       return ;
   }

   iRet = CcbWorkDelayEntry(gCcb.Param.ulPhWashT1,CcbDelayCallBack);
   if (iRet )
   {
       VOS_LOG(VOS_LOG_WARNING,"CcbWorkDelayEntry Fail %d",iRet);    
       /* notify ui (late implemnt) */
       return ;
   }

   pModBus->ucFuncode = MODBUS_FUNC_CODE_Force_Multiple_Coils;
   
   /* 1.Start Address */
   pModBus->aucData[iIdx++] = (0 >> 8) & 0XFF; 
   pModBus->aucData[iIdx++] = (0 >> 0) & 0XFF;
   
   /* 2.Reg Numbers */
   iTmp = APP_EXE_VALVE_NUM + APP_EXE_G_PUMP_NUM + APP_EXE_R_PUMP_NUM + APP_EXE_EDI_NUM + APP_EXE_RECT_NUM;
   pModBus->aucData[iIdx++] = (iTmp >> 8) & 0XFF;
   pModBus->aucData[iIdx++] = (iTmp >> 0) & 0XFF;
   
   /* 3. data counts */
   pModBus->aucData[iIdx++] = 2;
   
   /* close all valves */
   iTmp = 0;
   pModBus->aucData[iIdx++] = (iTmp >> 8) & 0XFF;
   pModBus->aucData[iIdx++] = (iTmp >> 0) & 0XFF;
   
   CAN_BUILD_ADDRESS_IDENTIFIER(ulIdentifier,APP_PAKCET_ADDRESS_MC,APP_PAKCET_ADDRESS_EXE);
   
   iRet = CcbModbusWorkEntry(0,ulIdentifier,aucModbusBuf,iIdx + 1,2000,CcbDefaultModbusCallBack); // don care modbus exe result
   if (iRet )
   {
       VOS_LOG(VOS_LOG_WARNING,"CcbModbusWorkEntry Fail %d",iRet);    
       /* notify ui (late implemnt) */
       return ;
   }

   iRet = CcbWorkDelayEntry(gCcb.Param.ulPhWashT2,CcbDelayCallBack);
   if (iRet )
   {
       VOS_LOG(VOS_LOG_WARNING,"CcbWorkDelayEntry Fail %d",iRet);    
       /* notify ui (late implemnt) */
       return ;
   }


   for (iLoop = 0; iLoop < 5; iLoop++)
   {

       /* enable E1,E2,E9,C3 ON */
       pModBus->ucFuncode = MODBUS_FUNC_CODE_Force_Multiple_Coils;
       
       /* 1.Start Address */
       pModBus->aucData[iIdx++] = (0 >> 8) & 0XFF; 
       pModBus->aucData[iIdx++] = (0 >> 0) & 0XFF;
       
       /* 2.Reg Numbers */
       iTmp = APP_EXE_VALVE_NUM + APP_EXE_G_PUMP_NUM + APP_EXE_R_PUMP_NUM + APP_EXE_EDI_NUM + APP_EXE_RECT_NUM;
       pModBus->aucData[iIdx++] = (iTmp >> 8) & 0XFF;
       pModBus->aucData[iIdx++] = (iTmp >> 0) & 0XFF;
       
       /* 3.data counts */
       pModBus->aucData[iIdx++] = 2;
       
       /* 4.valves */
       iTmp = (1 << APP_EXE_E1_NO)|(1<<APP_EXE_E9_NO)|(1<<APP_EXE_C3_NO);
       pModBus->aucData[iIdx++] = (iTmp >> 8) & 0XFF;
       pModBus->aucData[iIdx++] = (iTmp >> 0) & 0XFF;
       
       CAN_BUILD_ADDRESS_IDENTIFIER(ulIdentifier,APP_PAKCET_ADDRESS_MC,APP_PAKCET_ADDRESS_EXE);
       
       iRet = CcbModbusWorkEntry(0,ulIdentifier,aucModbusBuf,iIdx + 1,2000,CcbDefaultModbusCallBack); // don care modbus exe result
       if (iRet )
       {
           VOS_LOG(VOS_LOG_WARNING,"CcbModbusWorkEntry Fail %d",iRet);    
          /* notify ui (late implemnt) */
          return ;
       }
       
       iRet = CcbWorkDelayEntry(gCcb.Param.ulPhWashT3,CcbDelayCallBack);
       if (iRet )
       {
           VOS_LOG(VOS_LOG_WARNING,"CcbWorkDelayEntry Fail %d",iRet);    
           /* notify ui (late implemnt) */
           return ;
       }
       pModBus->ucFuncode = MODBUS_FUNC_CODE_Force_Multiple_Coils;
       
       /* 1.Start Address */
       pModBus->aucData[iIdx++] = (0 >> 8) & 0XFF; 
       pModBus->aucData[iIdx++] = (0 >> 0) & 0XFF;
       
       /* 2.Reg Numbers */
       iTmp = APP_EXE_VALVE_NUM + APP_EXE_G_PUMP_NUM + APP_EXE_R_PUMP_NUM + APP_EXE_EDI_NUM + APP_EXE_RECT_NUM;
       pModBus->aucData[iIdx++] = (iTmp >> 8) & 0XFF;
       pModBus->aucData[iIdx++] = (iTmp >> 0) & 0XFF;
       
       /* 3. data counts */
       pModBus->aucData[iIdx++] = 2;
       
       /* close all valves */
       iTmp = 0;
       pModBus->aucData[iIdx++] = (iTmp >> 8) & 0XFF;
       pModBus->aucData[iIdx++] = (iTmp >> 0) & 0XFF;
       
       CAN_BUILD_ADDRESS_IDENTIFIER(ulIdentifier,APP_PAKCET_ADDRESS_MC,APP_PAKCET_ADDRESS_EXE);
       
       iRet = CcbModbusWorkEntry(0,ulIdentifier,aucModbusBuf,iIdx + 1,2000,CcbDefaultModbusCallBack); // don care modbus exe result
       if (iRet )
       {
           VOS_LOG(VOS_LOG_WARNING,"CcbModbusWorkEntry Fail %d",iRet);    
           /* notify ui (late implemnt) */
           return ;
       }

       iRet = CcbWorkDelayEntry(gCcb.Param.ulPhWashT4,CcbDelayCallBack);
       if (iRet )
       {
           VOS_LOG(VOS_LOG_WARNING,"CcbWorkDelayEntry Fail %d",iRet);    
           /* notify ui (late implemnt) */
           return ;
       }   
   }
   /* enable E1、E9、C1、C3 ON
   ON */
   pModBus->ucFuncode = MODBUS_FUNC_CODE_Force_Multiple_Coils;
   
   /* 1.Start Address */
   pModBus->aucData[iIdx++] = (0 >> 8) & 0XFF; 
   pModBus->aucData[iIdx++] = (0 >> 0) & 0XFF;
   
   /* 2.Reg Numbers */
   iTmp = APP_EXE_VALVE_NUM + APP_EXE_G_PUMP_NUM + APP_EXE_R_PUMP_NUM + APP_EXE_EDI_NUM + APP_EXE_RECT_NUM;
   pModBus->aucData[iIdx++] = (iTmp >> 8) & 0XFF;
   pModBus->aucData[iIdx++] = (iTmp >> 0) & 0XFF;
   
   /* 3.data counts */
   pModBus->aucData[iIdx++] = 2;
   
   /* 4.valves */
   iTmp = (1 << APP_EXE_E1_NO)|(1<<APP_EXE_E9_NO)|(1<<APP_EXE_C3_NO);
   pModBus->aucData[iIdx++] = (iTmp >> 8) & 0XFF;
   pModBus->aucData[iIdx++] = (iTmp >> 0) & 0XFF;
   
   CAN_BUILD_ADDRESS_IDENTIFIER(ulIdentifier,APP_PAKCET_ADDRESS_MC,APP_PAKCET_ADDRESS_EXE);
   
   iRet = CcbModbusWorkEntry(0,ulIdentifier,aucModbusBuf,iIdx + 1,2000,CcbDefaultModbusCallBack); // don care modbus exe result
   if (iRet )
   {
       VOS_LOG(VOS_LOG_WARNING,"CcbModbusWorkEntry Fail %d",iRet);    
      /* notify ui (late implemnt) */
      return ;
   }
   
   iRet = CcbWorkDelayEntry(gCcb.Param.ulPhWashT5,CcbDelayCallBack);
   if (iRet )
   {
       VOS_LOG(VOS_LOG_WARNING,"CcbWorkDelayEntry Fail %d",iRet);    
       /* notify ui (late implemnt) */
       return ;
   }

   /* go to idle */
   /* shutdown everything */
   pModBus->ucFuncode = MODBUS_FUNC_CODE_Force_Multiple_Coils;
   
   /* 1.Start Address */
   pModBus->aucData[iIdx++] = (0 >> 8) & 0XFF; 
   pModBus->aucData[iIdx++] = (0 >> 0) & 0XFF;
   
   /* 2.Reg Numbers */
   iTmp = APP_EXE_VALVE_NUM + APP_EXE_G_PUMP_NUM + APP_EXE_R_PUMP_NUM + APP_EXE_EDI_NUM + APP_EXE_RECT_NUM;
   pModBus->aucData[iIdx++] = (iTmp >> 8) & 0XFF;
   pModBus->aucData[iIdx++] = (iTmp >> 0) & 0XFF;
   
   /* 3. data counts */
   pModBus->aucData[iIdx++] = 2;
   
   /* close all valves */
   iTmp = 0;
   pModBus->aucData[iIdx++] = (iTmp >> 8) & 0XFF;
   pModBus->aucData[iIdx++] = (iTmp >> 0) & 0XFF;
   
   CAN_BUILD_ADDRESS_IDENTIFIER(ulIdentifier,APP_PAKCET_ADDRESS_MC,APP_PAKCET_ADDRESS_EXE);
   
   CcbModbusWorkEntry(0,ulIdentifier,aucModbusBuf,iIdx + 1,2000,CcbDefaultModbusCallBack); // don care modbus exe result

   /* notify ui (late implement) */
      
}


WORK_ITEM_STRU *CcbAllocWorkItem(void)
{
   WORK_ITEM_STRU *pWorkItem = (WORK_ITEM_STRU *)malloc(sizeof(WORK_ITEM_STRU));

   if (!pWorkItem) return NULL;

   INIT_LIST_HEAD(&pWorkItem->list);

   pWorkItem->para = NULL;
   
   pWorkItem->proc = NULL;

   return pWorkItem;
}


void CcbScheduleWork(void)
{
     if (gCcb.iBusyWork)
     {
         return;
     }

     if (!list_empty(&gCcb.WorkList))
     {
         WORK_ITEM_STRU *pWorkItem = list_entry(gCcb.WorkList.next,WORK_ITEM_STRU,list) ;

         dispatch_threadpool(gThdPool,WorkCommEntry,pWorkItem);

         return;
     }

     /* notify ui (late implement) */
}

void CcbAddWork(WORK_ITEM_STRU *pWorkItem)
{
     list_add_tail(&pWorkItem->list,&gCcb.WorkList);
     
     CcbScheduleWork();
     
}

void WorkCommEntry(void *para)
{
    WORK_ITEM_STRU *pWorkItem = (WORK_ITEM_STRU *)para;

    gCcb.iBusyWork = TRUE;

    pWorkItem->proc(pWorkItem->para);

    gCcb.iBusyWork = FALSE;

    list_del_init(&pWorkItem->list);

    free(pWorkItem);

    CcbScheduleWork();
}


DISPHANDLE DispCmdIdleProc(void)
{
    if (DISP_WORK_STATE_IDLE == gCcb.iMainWorkState
        && DISP_WORK_SUB_IDLE == gCcb.iSubWorkState)
    {
        return DISP_INVALID_HANDLE;
    }

    /* just put into work queue */
    {
        WORK_ITEM_STRU *pWorkItem = CcbAllocWorkItem();

        if (!pWorkItem)
        {
            return DISP_INVALID_HANDLE;
        }

        pWorkItem->proc = work_idle;
        pWorkItem->para = &gCcb;

        CcbAddWork(pWorkItem);

        return (DISPHANDLE)pWorkItem;
    }
}

DISPHANDLE DispCmdROWashProc(void)
{
    if (DISP_WORK_STATE_IDLE != gCcb.iMainWorkState
        || DISP_WORK_SUB_IDLE != gCcb.iSubWorkState)
    {
        return DISP_INVALID_HANDLE;
    }

    /* just put into work queue */
    {
        WORK_ITEM_STRU *pWorkItem = CcbAllocWorkItem();

        if (!pWorkItem)
        {
            return DISP_INVALID_HANDLE;
        }

        pWorkItem->proc = work_idle_rowash;
        pWorkItem->para = &gCcb;

        CcbAddWork(pWorkItem);

        return (DISPHANDLE)pWorkItem;
    }
}

DISPHANDLE DispCmdPHWashProc(void)
{
    if (DISP_WORK_STATE_IDLE != gCcb.iMainWorkState
        || DISP_WORK_SUB_IDLE != gCcb.iSubWorkState)
    {
        return DISP_INVALID_HANDLE;
    }

    /* just put into work queue */
    {
        WORK_ITEM_STRU *pWorkItem = CcbAllocWorkItem();

        if (!pWorkItem)
        {
            return DISP_INVALID_HANDLE;
        }

        pWorkItem->proc = work_idle_phwash;
        pWorkItem->para = &gCcb;

        CcbAddWork(pWorkItem);

        return (DISPHANDLE)pWorkItem;
    }
}


DISPHANDLE DispCmdEntry(int iCmdId,unsigned char *pucData, int iLength)
{
    switch(iCmdId)
    {
    case DISP_CMD_IDLE:
        return DispCmdIdleProc();
        break;
    case DISP_CMD_RUN:
        break;
    case DISP_CMD_RO_WASH:
        return DispCmdROWashProc();
    case DISP_CMD_PH_WASH:
        return DispCmdPHWashProc();
    }
    return DISP_INVALID_HANDLE;
}

int DispGetWorkState(void)
{
    return gCcb.iMainWorkState; 
}

void CcbInit(void)
{
    //int err = 0;

    VOS_SetLogLevel(VOS_LOG_DEBUG);

    // init thread local storage
    tls_init(&pthread_tno_key);

    mem_init();

    init_msgQueue();

    gThdPool = create_threadpool(20); 

    MSG_QUEUE_ID_MAIN      = MsgQueueAlloc();
    MSG_QUEUE_ID_CANITF    = MsgQueueAlloc();

    TimerInit();

    // start period check timer
    timer_add(1000,TIMER_PERIOD,Main_second_handler,TIMER_SECOND);

    dispatch_threadpool(gThdPool,MainProc,&gCcb);

    dispatch_threadpool(gThdPool,CanItfProc,&gCanItf[0]);

    dispatch_threadpool(gThdPool,TimerProc,&gTps);

    // send init message to Main Proc
    SndMsg2(MSG_QUEUE_ID_MAIN    ,INIT_ALL_THREAD_EVENT,0,NULL);
    SndMsg2(MSG_QUEUE_ID_CANITF  ,INIT_ALL_THREAD_EVENT,0,NULL);
}

#ifdef __cplusplus
}
#endif

