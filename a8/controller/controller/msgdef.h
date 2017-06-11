#ifndef _DASERVICE_EVENT_H
#define _DASERVICE_EVENT_H

#define	SAT_OS_EVENT						(unsigned short int)1000
#define	INIT_ALL_THREAD_EVENT			    SAT_OS_EVENT+(unsigned short int)1
#define	QUIT_ALL_THREAD_EVENT			    SAT_OS_EVENT+(unsigned short int)2

#define	USER_MESSAGE_EVENT				    (unsigned short int)2001

#define MAIN_CANITF_MSG                      (USER_MESSAGE_EVENT+1)

#define CANITF_MAIN_MSG                      (USER_MESSAGE_EVENT+2)

#define TIMER_MSG                            (USER_MESSAGE_EVENT+3)


#define TCP_CLIENT_SEND_DATA				(USER_MESSAGE_EVENT + 0x100)

#define ASTCP_CLIENT_RECEIVED				(TCP_CLIENT_SEND_DATA + 1)


#endif

