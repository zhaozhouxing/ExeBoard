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

void CcbDefaultModbusCallBack(void *param)
{
    MODBUS_PACKET_COMM_STRU *pmg = (MODBUS_PACKET_COMM_STRU *)param;

    int iContLen = 0;

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
   

   MainInitInnerIpc();
   
   // set reset to all sub modulers
   MainResetModulers();
}

void MainDeInitMsg(void)
{
    MainDeinitInnerIpc();

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
    MODBUS_PACKET_COMM_STRU *pmg = (MODBUS_PACKET_COMM_STRU *)&pCanItfMsg->aucData[RPC_POS_DAT0]; 

    if (gCcb.ModbusCb)
    {
        (gCcb.ModbusCb)(pmg);
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
                if (pEco->ucId < MAX_ECO_NUM)
                {
                    gCcb.ExeBrd.aEco[pEco->ucId].fValue = pEco->fValue;
                    gCcb.ExeBrd.aEco[pEco->ucId].usTemp = pEco->usTemp;
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
                    gCcb.FlowMeter.aulInputRegs[pFM->ucId] = pFM->ulValue;
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
        char buf[32];

        unsigned int ulCanId;

        int iPayLoad;

        APP_PACKET_HO_STRU *pHoRsp = (APP_PACKET_HO_STRU *)buf;
        APP_PACKET_HO_CIR_RSP_STRU *pCirRsp = pHoRsp->aucData;

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
        char buf[32];

        unsigned int ulCanId;
        int iPayLoad;

        APP_PACKET_HO_STRU *pHoRsp = (APP_PACKET_HO_STRU *)buf;
        APP_PACKET_HO_QTW_RSP_STRU *pQtwRsp = pHoRsp->aucData;

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
    switch(pCanItfMsg->aucData[1 + RPC_POS_CMD1])
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

    printf("can msg 0x%x\r\n",pCanItfMsg->ulCanId);

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
        gCcb.ModbusCb = NULL;
        return -1;
    }

    gCcb.ModbusCb = NULL;
    sp_thread_mutex_unlock( &gCcb.Ipc.mutex );  
    
    return 0; // success
}

// dispatch_threadpool(gThdPool,Batch4TestImgProc,NULL);


DISPHANDLE DispCmdEntry(int iCmdId,unsigned char *pucData, int iLength)
{
    switch(iCmdId)
    {
    case DISP_CMD_QRY_FM:
        break;
    case DISP_CMD_QRY_QM:
        break;
    }
    return DISP_INVALID_HANDLE;
}

int DispGetWorkState(void)
{
    return gCcb.iWorkState; 
}

void CcbInit(void)
{
    //int err = 0;

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

