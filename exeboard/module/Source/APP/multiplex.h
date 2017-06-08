#ifndef _MULTIPLEX_H_
#define _MULTIPLEX_H_

#include "stm32_eval.h"

#include "list.h"

#define MULTIPLEX_MAX_NUM   (3)  //  adc channels which are multiplexed
#define MULTIPLEX_NODES_NUM (4)

typedef struct
{
    list_t list;
    
    uint8_t ucChl;

    int     addData;
    
}MULTIPLEX_NODE_STRU;


typedef struct
{
 list_t head;

 MULTIPLEX_NODE_STRU  nodes[MULTIPLEX_NODES_NUM];
 
}MULTIPLEX_STRU;

extern MULTIPLEX_STRU gaMultiplex[MULTIPLEX_MAX_NUM];

void Mulitplex_Add(uint8_t ucMult,uint8_t ucNode,int addData);
void Mulitplex_Rmv(uint8_t ucMult,uint8_t ucNode);
void MultiplexInit(void);

#endif
