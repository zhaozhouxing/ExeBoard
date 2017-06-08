#include <string.h>

#include "stm32_eval.h"

#include "task.h"

#include "multiplex.h"


MULTIPLEX_STRU gaMultiplex[MULTIPLEX_MAX_NUM];


void MultiplexInit(void)
{
    uint8_t i,j;
    memset(gaMultiplex,0,sizeof(gaMultiplex));

    for (i = 0; i < MULTIPLEX_MAX_NUM; i++)
    {
        INIT_LIST_HEAD(&gaMultiplex[i].head);

        for (j = 0; j < MULTIPLEX_NODES_NUM; j++)
        {
            INIT_LIST_HEAD(&gaMultiplex[i].nodes[j].list);

            gaMultiplex[i].nodes[j].ucChl = j;
        }
    }
}

void Mulitplex_Add(uint8_t ucMult,uint8_t ucNode,int addData)
{
    gaMultiplex[ucMult].nodes[ucNode].addData = addData;

    list_del_init(&gaMultiplex[ucMult].nodes[ucNode].list);
    list_add_tail(&gaMultiplex[ucMult].nodes[ucNode].list,&gaMultiplex[ucMult].head);
}

void Mulitplex_Rmv(uint8_t ucMult,uint8_t ucNode)
{
    list_del_init(&gaMultiplex[ucMult].nodes[ucNode].list);
}



