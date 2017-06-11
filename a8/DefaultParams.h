#ifndef _DEFAULT_PARAMS_H_
#define _DEFAULT_PARAMS_H_

#define SYSTEM_TEST

#ifdef SYSTEM_TEST
#define DEFAULT_RoWashT1  (10*1000)
#define DEFAULT_RoWashT2  (10*1000)
#define DEFAULT_RoWashT3  (10*1000)

#define DEFAULT_PhWashT1  (3*1000)
#define DEFAULT_PhWashT2  (10*1000)
#define DEFAULT_PhWashT3  (10*1000)
#define DEFAULT_PhWashT4  (10*1000)
#define DEFAULT_PhWashT5  (10*1000)
#else

#define DEFAULT_RoWashT1  (13*60*1000)
#define DEFAULT_RoWashT2  (5*60*1000)
#define DEFAULT_RoWashT3  (2*60*1000)

#define DEFAULT_PhWashT1  (3*1000)
#define DEFAULT_PhWashT2  (30*60*1000)
#define DEFAULT_PhWashT3  (5*60*1000)
#define DEFAULT_PhWashT4  (5*60*1000)
#define DEFAULT_PhWashT5  (10*60*1000)
#endif

#endif

