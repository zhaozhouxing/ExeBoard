#ifndef _MSG_APP_ITF_H____
#define _MSG_APP_ITF_H____

#include "msg.h"


#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
    SAT_MSG_HEAD msgHead;
    int   id;
}TIMER_MSG_STRU;


typedef struct
{
    SAT_MSG_HEAD msgHead;
    int iCanChl;
    unsigned int ulCanId;
    int iMsgLen;
    unsigned char aucData[1];
}MAIN_CANITF_STRU;

#define MAIN_CANITF_MSG_SIZE (sizeof(MAIN_CANITF_STRU) - sizeof(SAT_MSG_HEAD) - 1)

#define TIMER_MSG_MSG_SIZE   (sizeof(TIMER_MSG_STRU) - sizeof(SAT_MSG_HEAD))

#define TIMER_MSG_LEN        (sizeof(TIMER_MSG_STRU) - sizeof(SAT_MSG_HEAD))

#ifdef __cplusplus
}
#endif

#endif
