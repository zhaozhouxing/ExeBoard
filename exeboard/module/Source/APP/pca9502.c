/*! @file radio.c
 * @brief This file contains functions to interface with the radio chip.
 *
 * @b COPYRIGHT
 * @n Silicon Laboratories Confidential
 * @n Copyright 2012 Silicon Laboratories, Inc.
 * @n http://www.silabs.com
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "pca9502.h"

#include "task.h"

#include "ctype.h"

#include "sys_time.h "

#include "hal_spi_driver.h"
#include "Relay.h"


static const uint8_t spi_cs[PAC9502_MAX_NUM] = {
    STM32F103_GPA(4),     
    STM32F103_GPB(12),     
    STM32F103_GPA(5),     
};

const uint8_t aPca9502Defaults[PAC9502_MAX_NUM] = 
{
    0,0,0,
};


typedef struct
{
    
    uint8_t aucCache[PCA9502_REG_NUM];
    
    struct spi_adpater *pSpiAdpter;
    
}PCA9502_STRU;


static PCA9502_STRU sPca9502[PAC9502_MAX_NUM];




void PCA9502_Write(uint8_t ucChl,uint8_t ucReg,uint8_t ucValue)
{
    //printf("ucChl = %d\r\n",ucChl);
    stm32_gpio_set_value(spi_cs[ucChl],0); // ylf:  inactive low
    sPca9502[ucChl].pSpiAdpter->send((ucReg<<3));
    sPca9502[ucChl].pSpiAdpter->send(ucValue);
    sPca9502[ucChl].aucCache[ucReg - PCA9502_REG_IODir]  = ucValue;
    stm32_gpio_set_value(spi_cs[ucChl],1); // ylf:  inactive low
    
}

uint8_t PCA9502_Read(uint8_t ucChl,uint8_t ucReg)
{
    uint8_t ucValue;
    stm32_gpio_set_value(spi_cs[ucChl],0); // ylf:  inactive low
    sPca9502[ucChl].pSpiAdpter->send((ucReg<<3)|PCA9502_READ);
    ucValue = sPca9502[ucChl].pSpiAdpter->read();
    stm32_gpio_set_value(spi_cs[ucChl],1); // ylf:  inactive low

    return ucValue;
}

void PCA9502_Init(void)
{
    int iLoop;
//    uint8_t ucRet;

    struct spi_adpater *spi = hal_spi_Init(HAL_SPI1,SPI_CPOL_Low,SPI_CPHA_1Edge);

    memset(sPca9502,0,sizeof(sPca9502));

    for (iLoop = 0; iLoop < PAC9502_MAX_NUM; iLoop++)
    {
        stm32_gpio_cfgpin(spi_cs[iLoop],MAKE_PIN_CFG(GPIO_Speed_50MHz,GPIO_Mode_Out_PP)); 
        
        stm32_gpio_set_value(spi_cs[iLoop],1); // ylf:  active low

        sPca9502[iLoop].pSpiAdpter = spi;
    }

    // set to output mode
    
    for (iLoop = 0; iLoop < PAC9502_MAX_NUM; iLoop++)
    {
  
    
        PCA9502_Write(iLoop,PCA9502_REG_IOState,aPca9502Defaults[iLoop]);   
    
        PCA9502_Write(iLoop,PCA9502_REG_IODir,0XFF);   

        //ucRet = PCA9502_Read(iLoop,PCA9502_REG_IODir);

        //printf("IODir %d\r\n",ucRet);
    }

}

void PCA9502_SetGpio(uint8_t ucChl,uint8_t ucGpio,uint8_t ucValue)
{
    uint8_t ucTmp = sPca9502[ucChl].aucCache[PCA9502_REG_IOState - PCA9502_REG_IODir];

    if (ucValue)
    {
        ucTmp |= (1 << ucGpio);
    }
    else
    {
        ucTmp &= ~(1 << ucGpio);
    }
    
    PCA9502_Write(ucChl,PCA9502_REG_IOState,ucTmp);
    
}

uint8_t PCA9502_GetGpio(uint8_t ucChl,uint8_t ucGpio)
{
    
    uint8_t ucTmp = PCA9502_Read(ucChl,PCA9502_REG_IOState);

    return !!(ucTmp & (1 << ucGpio));
    
}

void PCA9502_UpdateGpios(uint8_t ucChl,uint8_t ucMask,uint8_t ucValue)
{
    uint8_t ucTmp = sPca9502[ucChl].aucCache[PCA9502_REG_IOState - PCA9502_REG_IODir];

    ucTmp &= ~ucMask;

    ucValue &= ucMask;

    ucTmp |= ucValue;

    PCA9502_Write(ucChl,PCA9502_REG_IOState,ucTmp);
}

uint8_t PCA9502_GetBuffedGpio(uint8_t ucChl,uint8_t ucGpio)
{
    
    uint8_t ucTmp = sPca9502[ucChl].aucCache[PCA9502_REG_IOState - PCA9502_REG_IODir];

    return !!(ucTmp & (1 << ucGpio));
    
}


