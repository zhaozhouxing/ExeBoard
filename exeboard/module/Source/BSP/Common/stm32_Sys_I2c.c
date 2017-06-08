/**
  ******************************************************************************
  * @file    stm32_eval_i2c_ee.c
  * @author  MCD Application Team
  * @version V4.2.0
  * @date    04/16/2010
  * @brief   This file provides a set of functions needed to manage the I2C M24CXX 
  *          EEPROM memory mounted on STM32xx-EVAL board (refer to stm32_eval.h
  *          to know about the boards supporting this memory). 
  *          It implements a high level communication layer for read and write 
  *          from/to this memory. The needed STM32 hardware resources (I2C and 
  *          GPIO) are defined in stm32xx_eval.h file, and the initialization is 
  *          performed in sEE_LowLevel_Init() function declared in stm32xx_eval.c 
  *          file.
  *          You can easily tailor this driver to any other development board, 
  *          by just adapting the defines for hardware resources and 
  *          sEE_LowLevel_Init() function. 
  *        
  *          @note In this driver, basic read and write functions (sEE_ReadBuffer() 
  *                and sEE_WritePage()) use the DMA to perform the data transfer 
  *                to/from EEPROM memory (except when number of requested data is
  *                equal to 1). Thus, after calling these two functions, user 
  *                application may perform other tasks while DMA is transferring
  *                data. The application should then monitor the variable holding 
  *                the number of data in order to determine when the transfer is
  *                completed (variable decremented to 0). Stopping transfer tasks
  *                are performed into DMA interrupt handlers (which are integrated
  *                into this driver).
  *            
  *     +-----------------------------------------------------------------+
  *     |                        Pin assignment                           |                 
  *     +---------------------------------------+-----------+-------------+
  *     |  STM32 I2C Pins                       |   sEE     |   Pin       |
  *     +---------------------------------------+-----------+-------------+
  *     | .                                     |   E0(GND) |    1  (0V)  |
  *     | .                                     |   E1(GND) |    2  (0V)  |
  *     | .                                     |   E2(GND) |    3  (0V)  |
  *     | .                                     |   E0(VSS) |    4  (0V)  |
  *     | HAL_I2C_SDA_PIN/ SDA                  |   SDA     |    5        |
  *     | HAL_I2C_SCL_PIN/ SCL                  |   SCL     |    6        |
  *     | .                                     |   /WC(VDD)|    7 (3.3V) |
  *     | .                                     |   VDD     |    8 (3.3V) |
  *     +---------------------------------------+-----------+-------------+  
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */ 

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>

#include    <ucos_ii.h>

#include "stm32_i2c_hal.h"

#ifdef I2C_DBG_INFO
#define dev_dbg(fmt,arg...) printf(fmt,## arg)
#else
#define dev_dbg(fmt,arg...)
#endif

/**
  * @}
  */

/** @addtogroup STM3210C_EVAL_LOW_LEVEL_I2C_EE
  * @{
  */
/**
  * @brief  I2C EEPROM Interface pins
  */  
#define HAL_I2C                          I2C1
#define HAL_I2C_CLK                      RCC_APB1Periph_I2C1
#define HAL_I2C_SCL_PIN                  GPIO_Pin_6                  /* PB.06 */
#define HAL_I2C_SCL_GPIO_PORT            GPIOB                       /* GPIOB */
#define HAL_I2C_SCL_GPIO_CLK             RCC_APB2Periph_GPIOB
#define HAL_I2C_SDA_PIN                  GPIO_Pin_7                  /* PB.07 */
#define HAL_I2C_SDA_GPIO_PORT            GPIOB                       /* GPIOB */
#define HAL_I2C_SDA_GPIO_CLK             RCC_APB2Periph_GPIOB
#define sEE_M24C64_32

#define HAL_I2C_DMA                      DMA1   
#define HAL_I2C_DMA_CHANNEL_TX           DMA1_Channel6
#define HAL_I2C_DMA_CHANNEL_RX           DMA1_Channel7 
#define HAL_I2C_DMA_FLAG_TX_TC           DMA1_IT_TC6   
#define HAL_I2C_DMA_FLAG_TX_GL           DMA1_IT_GL6 
#define HAL_I2C_DMA_FLAG_RX_TC           DMA1_IT_TC7 
#define HAL_I2C_DMA_FLAG_RX_GL           DMA1_IT_GL7    
#define HAL_I2C_DMA_CLK                  RCC_AHBPeriph_DMA1
#define HAL_I2C_DR_Address               ((uint32_t)0x40005410)
   
#define HAL_I2C_DMA_TX_IRQn              DMA1_Channel6_IRQn
#define HAL_I2C_DMA_RX_IRQn              DMA1_Channel7_IRQn
#define HAL_I2C_DMA_TX_IRQHandler        DMA1_Channel6_IRQHandler
#define HAL_I2C_DMA_RX_IRQHandler        DMA1_Channel7_IRQHandler   
#define HAL_I2C_DMA_PREPRIO              0
#define HAL_I2C_DMA_SUBPRIO              0   
   
#define HAL_I2C_DIRECTION_TX             0
#define HAL_I2C_DIRECTION_RX             1   

DMA_InitTypeDef                          sEEDMA_InitStructure; 

#define I2C_SPEED                       100000
#define I2C_SLAVE_ADDRESS7              0x35

#define HAL_I2C_ACCESS_DMA              0

#define HAL_IICSTAT_LASTBIT             (1 << 0)


enum hal_i2c_state {
    STATE_IDLE,
    STATE_START,
    STATE_READ,
    STATE_WRITE,
    STATE_STOP
};


struct hal_i2c {

    OS_EVENT            *wait;
    struct i2c_msg      *msg;
    unsigned int        msg_num;
    unsigned int        msg_idx;
    unsigned int        msg_ptr;
    int                 state ; 
    int                 tx_setup;
    struct i2c_adapter  adap;

};

static struct hal_i2c sI2C;

void halI2C_Delay(int tick)
{
    int tend = ((int)jiffies) + tick;
    while(((int)jiffies) < tend);
}


/**
  * @brief  DeInitializes peripherals used by the I2C EEPROM driver.
  * @param  None
  * @retval None
  */

void halI2C_LowLevel_DeInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure; 
  NVIC_InitTypeDef NVIC_InitStructure;    
   
  /* HAL_I2C Peripheral Disable */
  I2C_Cmd(HAL_I2C, DISABLE);
 
  /* HAL_I2C DeInit */
  I2C_DeInit(HAL_I2C);

  /*!< HAL_I2C Periph clock disable */
  RCC_APB1PeriphClockCmd(HAL_I2C_CLK, DISABLE);
    
  /*!< GPIO configuration */  
  /*!< Configure HAL_I2C pins: SCL */
  GPIO_InitStructure.GPIO_Pin = HAL_I2C_SCL_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(HAL_I2C_SCL_GPIO_PORT, &GPIO_InitStructure);

  /*!< Configure HAL_I2C pins: SDA */
  GPIO_InitStructure.GPIO_Pin = HAL_I2C_SDA_PIN;
  GPIO_Init(HAL_I2C_SDA_GPIO_PORT, &GPIO_InitStructure);
  
  /* Configure and enable I2C DMA TX Channel interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = HAL_I2C_DMA_TX_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = HAL_I2C_DMA_PREPRIO;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = HAL_I2C_DMA_SUBPRIO;
  NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Configure and enable I2C DMA RX Channel interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = HAL_I2C_DMA_RX_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = HAL_I2C_DMA_PREPRIO;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = HAL_I2C_DMA_SUBPRIO;
  NVIC_Init(&NVIC_InitStructure);   
  
  /* Disable and Deinitialize the DMA channels */
  DMA_Cmd(HAL_I2C_DMA_CHANNEL_TX, DISABLE);
  DMA_Cmd(HAL_I2C_DMA_CHANNEL_RX, DISABLE);
  DMA_DeInit(HAL_I2C_DMA_CHANNEL_TX);
  DMA_DeInit(HAL_I2C_DMA_CHANNEL_RX);
}


/**
  * @brief  Initializes DMA channel used by the I2C EEPROM driver.
  * @param  None
  * @retval None
  */
void halI2C_LowLevel_DMAConfig(uint32_t pBuffer, uint32_t BufferSize, uint32_t Direction)
{ 
  /* Initialize the DMA with the new parameters */
  if (Direction == HAL_I2C_DIRECTION_TX)
  {
    /* Configure the DMA Tx Channel with the buffer address and the buffer size */
    sEEDMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)pBuffer;
    sEEDMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;    
    sEEDMA_InitStructure.DMA_BufferSize = (uint32_t)BufferSize;  
    DMA_Init(HAL_I2C_DMA_CHANNEL_TX, &sEEDMA_InitStructure);  
  }
  else
  { 
    /* Configure the DMA Rx Channel with the buffer address and the buffer size */
    sEEDMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)pBuffer;
    sEEDMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    sEEDMA_InitStructure.DMA_BufferSize = (uint32_t)BufferSize;      
    DMA_Init(HAL_I2C_DMA_CHANNEL_RX, &sEEDMA_InitStructure);    
  }
}


/**
  * @brief  Initializes peripherals used by the I2C EEPROM driver.
  * @param  None
  * @retval None
  */
void halI2C_LowLevel_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;  

  /*!< HAL_I2C_SCL_GPIO_CLK and HAL_I2C_SDA_GPIO_CLK Periph clock enable */
  RCC_APB2PeriphClockCmd(HAL_I2C_SCL_GPIO_CLK | HAL_I2C_SDA_GPIO_CLK, ENABLE);

  /*!< HAL_I2C Periph clock enable */
  RCC_APB1PeriphClockCmd(HAL_I2C_CLK, ENABLE);
    
  /*!< GPIO configuration */  
  /*!< Configure HAL_I2C pins: SCL */
  GPIO_InitStructure.GPIO_Pin = HAL_I2C_SCL_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(HAL_I2C_SCL_GPIO_PORT, &GPIO_InitStructure);

  /*!< Configure HAL_I2C pins: SDA */
  GPIO_InitStructure.GPIO_Pin = HAL_I2C_SDA_PIN;
  GPIO_Init(HAL_I2C_SDA_GPIO_PORT, &GPIO_InitStructure); 

  /* Configure and enable I2C DMA TX Channel interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = HAL_I2C_DMA_TX_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = HAL_I2C_DMA_PREPRIO;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = HAL_I2C_DMA_SUBPRIO;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Configure and enable I2C DMA RX Channel interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = HAL_I2C_DMA_RX_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = HAL_I2C_DMA_PREPRIO;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = HAL_I2C_DMA_SUBPRIO;
  NVIC_Init(&NVIC_InitStructure);  
  
  /*!< I2C DMA TX and RX channels configuration */
  /* Enable the DMA clock */
  RCC_AHBPeriphClockCmd(HAL_I2C_DMA_CLK, ENABLE);

  /* I2C TX DMA Channel configuration */
  DMA_DeInit(HAL_I2C_DMA_CHANNEL_TX);
  sEEDMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)HAL_I2C_DR_Address;
  sEEDMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)0;   /* This parameter will be configured durig communication */
  sEEDMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;    /* This parameter will be configured durig communication */
  sEEDMA_InitStructure.DMA_BufferSize = 0xFFFF;            /* This parameter will be configured durig communication */
  sEEDMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  sEEDMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  sEEDMA_InitStructure.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte;
  sEEDMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  sEEDMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  sEEDMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  sEEDMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(HAL_I2C_DMA_CHANNEL_TX, &sEEDMA_InitStructure);  
  
  /* I2C RX DMA Channel configuration */
  DMA_DeInit(HAL_I2C_DMA_CHANNEL_RX);
  DMA_Init(HAL_I2C_DMA_CHANNEL_RX, &sEEDMA_InitStructure);  
  
  /* Enable the DMA Channels Interrupts */
  DMA_ITConfig(HAL_I2C_DMA_CHANNEL_TX, DMA_IT_TC, ENABLE);
  DMA_ITConfig(HAL_I2C_DMA_CHANNEL_RX, DMA_IT_TC, ENABLE);    
}


/* irq enable/disable functions */

static void halI2C_disable_irq(struct hal_i2c *i2c)
{
    I2C_ITConfig(HAL_I2C, I2C_IT_EVT | I2C_IT_BUF, DISABLE); /* use interrupt mode */

}

static void halI2C_enable_irq(struct hal_i2c *i2c)
{
    I2C_ITConfig(HAL_I2C, I2C_IT_EVT | I2C_IT_BUF, ENABLE); /* use interrupt mode */
}

static void ndelay(int delay)
{
    while(delay > 0) delay--;
}


static  int is_lastmsg(struct hal_i2c *i2c)
{
    return i2c->msg_idx >= (i2c->msg_num - 1);
}

/* is_msglast
 *
 * returns TRUE if we this is the last byte in the current message
*/

static  int is_msglast(struct hal_i2c *i2c)
{
    /* msg->len is always 1 for the first byte of smbus block read.
     * Actual length will be read from slave. More bytes will be
     * read according to the length then. */
    if (i2c->msg->flags & I2C_M_RECV_LEN && i2c->msg->len == 1)
        return 0;

    return i2c->msg_ptr == i2c->msg->len-1;
}

/* is_msgend
 *
 * returns TRUE if we reached the end of the current message
*/

static  int is_msgend(struct hal_i2c *i2c)
{
    return i2c->msg_ptr >= i2c->msg->len;
}

static  void halI2C_enable_ack(struct hal_i2c *i2c)
{
    I2C_AcknowledgeConfig(HAL_I2C, ENABLE);
}

static  void halI2C_enable_pos(struct hal_i2c *i2c)
{
    I2C_PECPositionConfig(HAL_I2C, I2C_PECPosition_Next);
}

//static  void halI2C_disable_pos(struct hal_i2c *i2c)
//{
//    I2C_PECPositionConfig(HAL_I2C, 0);
//}



static  void halI2C_disable_ack(struct hal_i2c *i2c)
{
    I2C_AcknowledgeConfig(HAL_I2C, DISABLE);
}



/* halI2C_message_start
 *
 * put the start of a message onto the bus
*/

static void halI2C_message_start(struct hal_i2c *i2c,
                      struct i2c_msg *msg)
{
    i2c->state = STATE_START;

    /* todo - check for whether ack wanted or not */
    halI2C_enable_ack(i2c);

    ndelay(sI2C.tx_setup);

    /*!< Send START condition */
    I2C_GenerateSTART(HAL_I2C, ENABLE);
}

static void halI2C_master_complete(struct hal_i2c *i2c, int ret)
{
    // dev_dbg("master_complete %d\r\n", ret);

    i2c->msg_ptr = 0;
    i2c->msg = NULL;
    i2c->msg_idx++;
    i2c->msg_num = 0;
    if (ret)
        i2c->msg_idx = ret;

    OSMboxPost(i2c->wait,(void *)1);
}



static void halI2C_stop(struct hal_i2c *i2c,int state)
{

    I2C_GenerateSTOP(HAL_I2C, ENABLE);

    i2c->state = STATE_STOP;

    halI2C_master_complete(i2c, state);
    
    halI2C_disable_irq(i2c);    

}



/* hal_i2c_irq_nextbyte
 *
 * this starts an i2c transfer
*/

static int hal_i2c_irq_nextbyte(struct hal_i2c *i2c, unsigned long iicstat,unsigned long event)
{
    unsigned char byte;
    int ret = 0;

    switch (i2c->state) {

    case STATE_IDLE:
        goto out;

    case STATE_STOP:
        halI2C_disable_irq(i2c);
        goto out_ack;
    case STATE_START:
        /* last thing we did was send a start condition on the
         * bus, or started a new i2c message
         */

        if (iicstat & HAL_IICSTAT_LASTBIT &&
            !(i2c->msg->flags & I2C_M_IGNORE_NAK)) {
            /* ack was not received... */

            halI2C_stop(i2c, -ENXIO);
            goto out_ack;
        }

        if (i2c->msg->flags & I2C_M_RD)
            i2c->state = STATE_READ;
        else
            i2c->state = STATE_WRITE;

        /* terminate the transfer if there is nothing to do
         * as this is used by the i2c probe to find devices. */

        if (is_lastmsg(i2c) && i2c->msg->len == 0) {
            halI2C_stop(i2c, 0);
            goto out_ack;
        }

        if (i2c->state == STATE_READ)
        { 
            if (2 == i2c->msg->len)
            {
                halI2C_enable_pos(i2c);
            }
            goto prepare_read;
        }

        /* fall through to the write state, as we will need to
         * send a byte as well */

    case STATE_WRITE:
        /* we are writing data to the device... check for the
         * end of the message, and if so, work out what to do
         */

        if (!(i2c->msg->flags & I2C_M_IGNORE_NAK)) {
            if (iicstat & HAL_IICSTAT_LASTBIT) {

                halI2C_stop(i2c, -ECONNREFUSED);
                goto out_ack;
            }
        }

 retry_write:

        if (!is_msgend(i2c)) {
            byte = i2c->msg->buf[i2c->msg_ptr++];

            I2C_SendData(HAL_I2C, byte); 

            /* delay after writing the byte to allow the
             * data setup time on the bus, as writing the
             * data to the register causes the first bit
             * to appear on SDA, and SCL will change as
             * soon as the interrupt is acknowledged */

            ndelay(i2c->tx_setup);

        } else if (!is_lastmsg(i2c)) {
            /* we need to go to the next i2c message */

            i2c->msg_ptr = 0;
            i2c->msg_idx++;
            i2c->msg++;

            /* check to see if we need to do another message */
            if (i2c->msg->flags & I2C_M_NOSTART) { // YLF: CHECK NEXT MESSAGE

                if (i2c->msg->flags & I2C_M_RD) {
                    /* cannot do this, the controller
                     * forces us to send a new START
                     * when we change direction */

                    halI2C_stop(i2c, -EINVALID);
                }

                goto retry_write;
            } else {
                /* send the new start */
                halI2C_message_start(i2c, i2c->msg);
                
                i2c->state = STATE_START;
            }

        } else {
            /* send stop */

            halI2C_stop(i2c, 0);
        }
        break;

    case STATE_READ:
        /* we have a byte of data in the data register, do
         * something with it, and then work out whether we are
         * going to do any more read/write
         */

        if (2 == i2c->msg->len && ((event & 0x44) == 0x44)) // RXNE 
        {
            I2C_GenerateSTOP(HAL_I2C, ENABLE);

            byte = I2C_ReceiveData(HAL_I2C);

            i2c->msg->buf[i2c->msg_ptr++] = byte;
            
            ndelay(i2c->tx_setup);

            byte = I2C_ReceiveData(HAL_I2C);
            
            i2c->msg->buf[i2c->msg_ptr++] = byte;
        }
        else if (1 == i2c->msg->len && (event & 0x40))
        {
            byte = I2C_ReceiveData(HAL_I2C);

            i2c->msg->buf[i2c->msg_ptr++] = byte;
        }
        else if ( 3 <= i2c->msg->len)
        {
            int delta = i2c->msg->len - i2c->msg_ptr;
           
            if (delta == 3)
            {
                if (!(event & 0x4))
                {
                    break;
                }
                halI2C_disable_ack(i2c);

                byte = I2C_ReceiveData(HAL_I2C);
                i2c->msg->buf[i2c->msg_ptr++] = byte;


                I2C_GenerateSTOP(HAL_I2C, ENABLE);
                
                ndelay(i2c->tx_setup);

                byte = I2C_ReceiveData(HAL_I2C);
                i2c->msg->buf[i2c->msg_ptr++] = byte;
            }
            else 
            {
                if ((event & 0x40))
                {
                    byte = I2C_ReceiveData(HAL_I2C);
                    i2c->msg->buf[i2c->msg_ptr++] = byte;
                }
            }
        }
        
 prepare_read:

        if ((event & I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) == I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)
        {
            if (2 == i2c->msg->len) 
            {
                halI2C_disable_ack(i2c);
            }
            else if (1 == i2c->msg->len)
            {
                halI2C_disable_ack(i2c);

                I2C_GenerateSTOP(HAL_I2C, ENABLE);
            }
        }
    
        if (is_msglast(i2c)) { /* ylf: last byte of buffer */
            /* last byte of buffer */

            if (is_lastmsg(i2c))
                halI2C_disable_ack(i2c);

        } else if (is_msgend(i2c)) {
            /* ok, we've read the entire buffer, see if there
             * is anything else we need to do */

            if (is_lastmsg(i2c)) {
                /* last message, send stop and complete */
                halI2C_stop(i2c, 0);
            } else {
                /* go to the next transfer */
                i2c->msg_ptr = 0;
                i2c->msg_idx++;
                i2c->msg++;
            }
        }

        break;
    }

    /* acknowlegde the IRQ and get back on with the work */

 out_ack:
 out:
    
    return ret;
}

#ifdef HAL_I2C_DEBUG
static uint32_t teststack[8];
static uint32_t statcksize;
#endif
/**
  * @brief  irq for i2c.
  * @param  None
  * @retval None
  */
void halI2C_EV_IRQHandler(I2C_TypeDef * i2c)
{
    uint32_t event = I2C_GetLastEvent(i2c);
    {

        //==================I2C1 Master Mode Transmitter Related state===================
        if((event & I2C_EVENT_MASTER_MODE_SELECT) == I2C_EVENT_MASTER_MODE_SELECT)                      
        {

            struct i2c_msg      *msg = sI2C.msg;

            unsigned int addr = (msg->addr & 0x7f) << 1;
        
            if (msg->flags & I2C_M_RD) {
                addr |= 1;
            } 
        
            if (msg->flags & I2C_M_REV_DIR_ADDR)
                addr ^= 1; 
            
            // send slave address
            I2C_SendData(i2c, addr);

            ndelay(sI2C.tx_setup);

#ifdef HAL_I2C_DEBUG
            statcksize = 0;
#endif                      

        }
        else
        if((event & I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)                      
        {
            hal_i2c_irq_nextbyte(&sI2C,0,event);
        }
        else
        if((event & I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) == I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)                      
        {
            hal_i2c_irq_nextbyte(&sI2C,0,event);
        }
        else
        if((event & I2C_EVENT_MASTER_BYTE_TRANSMITTED) == I2C_EVENT_MASTER_BYTE_TRANSMITTED)                      
        {
            hal_i2c_irq_nextbyte(&sI2C,0,event);
        }
        //else
        //if((event & I2C_EVENT_MASTER_BYTE_TRANSMITTING) == I2C_EVENT_MASTER_BYTE_TRANSMITTING)                      
        //{
        //    hal_i2c_irq_nextbyte(&sI2C,0,event);
        //}
        else
        if((event & I2C_EVENT_MASTER_BYTE_RECEIVED) == I2C_EVENT_MASTER_BYTE_RECEIVED
            || ((STATE_READ == sI2C.state) && ((event & 0x40) == 0x40)))                      
        {
            hal_i2c_irq_nextbyte(&sI2C,0,event);

        }
        else
        {
            //hal_i2c_irq_nextbyte(&sI2C,0,event);

            //if (STATE_READ == sI2C.state)
                //printf("ie:%x&%d\r\n",event,sI2C.state);
            
        }
        //printf("ie:%x&%d\r\n",event,sI2C.state);

        
#ifdef HAL_I2C_DEBUG
        if (STATE_READ == sI2C.state)
        {
            /printf("ie:%x&%d\r\n",event,sI2C.state);

            if((event & I2C_EVENT_MASTER_BYTE_TRANSMITTED) == I2C_EVENT_MASTER_BYTE_TRANSMITTED)  
            //if((event & I2C_EVENT_MASTER_BYTE_TRANSMITTING) == I2C_EVENT_MASTER_BYTE_TRANSMITTING)  
            {
                int iLoop = 0;
                for (iLoop = 0; iLoop < statcksize; iLoop++)
                {
                    printf("ie:-%x\r\n",teststack[iLoop]);
                
                }
            }
        }

        if (event != I2C_EVENT_MASTER_BYTE_TRANSMITTING)
        {
            teststack[statcksize++] = event;
            statcksize %= 8;
        }
#endif            
    }
}

/**
  * @brief  DeInitializes peripherals used by the I2C EEPROM driver.
  * @param  None
  * @retval None
  */
void halI2C_DeInit(void)
{
  halI2C_LowLevel_DeInit(); 
}

/**
  * @brief  Initializes peripherals used by the I2C EEPROM driver.
  * @param  None
  * @retval None
  */
void halI2C_Init(void)
{ 
  I2C_InitTypeDef  I2C_InitStructure;
  
  halI2C_LowLevel_Init();

  I2C_DeInit(HAL_I2C);
  
  /*!< I2C configuration */
  /* HAL_I2C configuration */
#ifdef NO_SMBUS_HOST  
  I2C_InitStructure.I2C_Mode                = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle           = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1         = I2C_SLAVE_ADDRESS7;
  I2C_InitStructure.I2C_Ack                 = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed          = I2C_SPEED;
#else
  I2C_InitStructure.I2C_Mode                = I2C_Mode_SMBusHost;
  I2C_InitStructure.I2C_DutyCycle           = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1         = I2C_SLAVE_ADDRESS7;
  I2C_InitStructure.I2C_Ack                 = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed          = I2C_SPEED;
#endif

  I2C_Init(HAL_I2C, &I2C_InitStructure);


  /* Configure and enable interrupt */
  {
      NVIC_InitTypeDef NVIC_InitStructure;  
      NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;
      NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = HAL_I2C_DMA_PREPRIO;
      NVIC_InitStructure.NVIC_IRQChannelSubPriority = HAL_I2C_DMA_SUBPRIO;
      NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
      NVIC_Init(&NVIC_InitStructure);
  }
  
  I2C_Cmd(HAL_I2C, ENABLE);

  /* Enable the HAL_I2C peripheral DMA requests */
  // I2C_DMACmd(HAL_I2C, ENABLE);
  
}


/* s3c24xx_i2c_set_master
 *
 * get the i2c bus for a master transaction
*/

static int halI2C_set_master(struct hal_i2c *i2c)
{
    int timeout = 10;

    while (timeout-- > 0) {
        
        if (!I2C_GetFlagStatus(HAL_I2C, I2C_FLAG_BUSY))
            return 0;

        halI2C_Delay(1);
    }

    return -ETIMEDOUT;
}


/* s3c24xx_i2c_wait_idle
 *
 * wait for the i2c bus to become idle.
*/

static void halI2C_wait_idle(struct hal_i2c *i2c)
{

}

int wait_event_timeout(OS_EVENT  *wait , int condition, int timeout)            \
{
    if (condition)
    {
        return 0;
    }

    {
        INT16U usTo = timeout;
        INT8U  ucErr = 0;
        OSMboxPend(wait,usTo,&ucErr);

        if (ucErr)
        {
            return ucErr;
        }

        return 0;
    }
}


/* hal_i2c_doxfer
 *
 * this starts an i2c transfer
*/

static int hal_i2c_doxfer(struct hal_i2c *i2c,
                  struct i2c_msg *msgs, int num)
{
    unsigned long timeout;
    int           ret;

    ret = halI2C_set_master(i2c);
    if (ret != 0) {
        
        dev_dbg("cannot get bus (error %d)\n", ret);
        ret = -EAGAIN;
        goto out;
    }

    i2c->msg     = msgs;
    i2c->msg_num = num;
    i2c->msg_ptr = 0;
    i2c->msg_idx = 0;
    i2c->state   = STATE_START;
    
    halI2C_enable_irq(i2c);

    halI2C_message_start(i2c, msgs);

    timeout = wait_event_timeout(i2c->wait, i2c->msg_num == 0, 2000);

    ret = i2c->msg_idx;

    /* having these next two as dev_err() makes life very
     * noisy when doing an i2cdetect */

    if (timeout)
    {
        dev_dbg("timeout\n");
    }
    else if (ret != num)
    {
        dev_dbg("incomplete xfer (%d)\n", ret);
    }

    halI2C_wait_idle(i2c);
 out:

    return ret;
}




/* hal_i2c_xfer
 *
 * first port of call from the i2c bus code when an message needs
 * transferring across the i2c bus.
*/

int hal_i2c_xfer(struct i2c_adapter *adap,
            struct i2c_msg *msgs, int num)
{
    struct hal_i2c *i2c = (struct hal_i2c *)adap->algo_data;
    int retry;
    int ret;

    for (retry = 0; retry < adap->retries; retry++) {

        ret = hal_i2c_doxfer(i2c, msgs, num);

        if (ret != -EAGAIN) {
            return ret;
        }
        
        halI2C_Delay(2);
    }

    return -EREMOTEIO;
}


/* i2c bus registration info */

static const struct i2c_algorithm hal_i2c_algorithm = {
     hal_i2c_xfer,
     0,
};




struct i2c_adapter *hal_i2c_Init(void)
{
    memset(&sI2C,0,sizeof(sI2C));
    sI2C.adap.algo      = &hal_i2c_algorithm;
    sI2C.adap.timeout   = 2;
    sI2C.adap.algo_data = &sI2C;
    sI2C.adap.retries   = 2;
    sI2C.tx_setup       = 10;
    sI2C.wait           = OSMboxCreate(0); 

    halI2C_Init();

    return &sI2C.adap;
}

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
