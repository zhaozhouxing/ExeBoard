#ifndef _APP_QMI_H
#define _APP_QMI_H

#define RS485_MAX_NUMBER 2


void   appQmiInit(void);
UINT8  appQmi_FillSndBuf(UINT8 ucPort,UINT8 *pData,UINT16 usLength);
void appQmi_msg_Handler(Message *Msg);

#endif
