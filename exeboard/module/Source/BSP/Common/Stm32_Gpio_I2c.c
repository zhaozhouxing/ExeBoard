#include <stdio.h>
#include <string.h>

#include <ucos_ii.h>

#include "Stm32_Gpio_I2c.h"


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

//IIC所有操作函数
void Gpio_IIC_Init(void);                //初始化IIC的IO口				 
bool Gpio_IIC_Start(struct hal_i2c *i2c);				//发送IIC开始信号
void Gpio_IIC_Stop(struct hal_i2c *i2c);	  			//发送IIC停止信号
void Gpio_IIC_Send_Byte(u8 txd);			//IIC发送一个字节
u8 Gpio_Gpio_IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
u8 Gpio_IIC_Wait_Ack(struct hal_i2c *i2c); 				//IIC等待ACK信号
void Gpio_IIC_Ack(void);					//IIC发送ACK信号
void Gpio_IIC_NAck(void);				//IIC不发送ACK信号

void Gpio_IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 Gpio_IIC_Read_One_Byte(u8 daddr,u8 addr);	  

void delay_us(u16 time)
{    
   u16 i=0;  
   while(time--)
   {
      i=10;  
      while(i--) ;    
   }
}  




void Gpio_IIC_Init(void)
{					     
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );	
	   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB,GPIO_Pin_6|GPIO_Pin_7); 	
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


bool Gpio_IIC_Start(struct hal_i2c *i2c)
{
	
	SDA_OUT();     //sda线输出
	Gpio_IIC_SDA=1;	  	  
	Gpio_IIC_SCL=1;
	delay_us(4);

	if(!Gpio_READ_SDA)	//SDA线为低电平则总线忙
		{
			return FALSE;
		}
	
 	Gpio_IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);

	if(Gpio_READ_SDA)	//SDA线为高电平则总线出错，退出
		return FALSE;

	i2c->state = STATE_START;
	
	Gpio_IIC_SCL=0;//准备发送或接收数据

	return TRUE;
}	


void Gpio_IIC_Stop(struct hal_i2c *i2c)
{
	SDA_OUT();//sda线输出
	Gpio_IIC_SCL=0;
	Gpio_IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	Gpio_IIC_SCL=1; 
	Gpio_IIC_SDA=1;//发送I2C总线结束信号

	i2c->state = STATE_STOP;
	
	delay_us(4);							   	
}


u8 Gpio_IIC_Wait_Ack(struct hal_i2c *i2c)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA设置为输入  
	Gpio_IIC_SDA=1;delay_us(1);	   
	Gpio_IIC_SCL=1;delay_us(1);	 
	while(Gpio_READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			Gpio_IIC_Stop(i2c);
			return 1;
		}
	}
	Gpio_IIC_SCL=0;//时钟输出0 	   
	return 0;  
} 


void Gpio_IIC_Ack(void)
{
	Gpio_IIC_SCL=0;
	SDA_OUT();
	Gpio_IIC_SDA=0;
	delay_us(2);
	Gpio_IIC_SCL=1;
	delay_us(2);
	Gpio_IIC_SCL=0;
}


void Gpio_IIC_NAck(void)
{
	Gpio_IIC_SCL=0;
	SDA_OUT();
	Gpio_IIC_SDA=1;
	delay_us(2);
	Gpio_IIC_SCL=1;
	delay_us(2);
	Gpio_IIC_SCL=0;
}	


void Gpio_IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    Gpio_IIC_SCL=0;//拉低时钟开始数据传输

    for(t=0;t<8;t++)
    {              
		if((txd&0x80)>>7)
			Gpio_IIC_SDA=1;
		else
			Gpio_IIC_SDA=0;
		txd<<=1; 	  
		delay_us(2);   
		Gpio_IIC_SCL=1;
		delay_us(2);
		Gpio_IIC_SCL=0;	
		delay_us(2);
    }	 
} 	 

u8 Gpio_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        Gpio_IIC_SCL=0; 
        delay_us(2);
		Gpio_IIC_SCL=1;
        receive<<=1;
        if(Gpio_READ_SDA)receive++;   
		delay_us(1); 
    }					 
    if (!ack)
        Gpio_IIC_NAck();//发送nACK
    else
        Gpio_IIC_Ack(); //发送ACK   
    return receive;
}


static int hal_Gpio_i2c_xfer(struct hal_i2c *i2c,
                  struct i2c_msg *msgs, int num)
{
	 unsigned char byte;
	 int ret = 0;
	 i2c->msg     = msgs;
     i2c->msg_num = num;
     i2c->msg_ptr = 0;
     i2c->msg_idx = 0;

	 if(!Gpio_IIC_Start(i2c))
	     return 0;
	 else
		{
			unsigned int addr = (msgs->addr & 0x7f) << 1;
			
			if (i2c->msg->flags & I2C_M_RD){  
				addr |= 1;	
			}
			if (i2c->msg->flags & I2C_M_REV_DIR_ADDR){
				addr ^= 1;
			}

			Gpio_IIC_Send_Byte(addr); 

			if(Gpio_IIC_Wait_Ack(i2c))
				{
					Gpio_IIC_Stop(i2c);
					return 0;
				}
		}

	 for (;!ret;)
	 {
		 switch(i2c->state)
		 {
		 case STATE_START:
			 if (i2c->msg->flags & I2C_M_RD){  
				i2c->state = STATE_READ;
			 }
			 else{
				i2c->state = STATE_WRITE;
			 }
			 if (is_lastmsg(i2c) && i2c->msg->len == 0) {
				Gpio_IIC_Stop(i2c);
				 goto out_ack;
			 }
			 
			 if (i2c->state == STATE_READ){
				goto prepare_read;
			 } 
		 case STATE_WRITE:
	  retry_write:
	  
			 for (;;)
			 {
				 if (!is_msgend(i2c)) {
					 byte = i2c->msg->buf[i2c->msg_ptr++];
					
					 Gpio_IIC_Send_Byte(byte);

					 if(Gpio_IIC_Wait_Ack(i2c))
						{
							Gpio_IIC_Stop(i2c);
							return 0;
						}
		  
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
		  
							 Gpio_IIC_Stop(i2c);
						 }
		  
						 goto retry_write;
					 } else {
						 /* send the new start */
						 unsigned int addr = (msgs->addr & 0x7f) << 1;

						 Gpio_IIC_Stop(i2c);

						 if(!Gpio_IIC_Start(i2c))
						 	printf("start false\r\n");
						 

						 if (i2c->msg->flags & I2C_M_RD){
							 addr |= 1;	
						 }
						 if (i2c->msg->flags & I2C_M_REV_DIR_ADDR)
							 addr ^= 1;

						Gpio_IIC_Send_Byte(addr); 

						if(Gpio_IIC_Wait_Ack(i2c))
						{
							Gpio_IIC_Stop(i2c);
							return 0;
						}

						i2c->state = STATE_START;
						break;
					 }
		  
				 } else {
					 /* send stop */
					 Gpio_IIC_Stop(i2c);
					 break;
				 }
			 }
			 break;
		case STATE_READ:
			for (;;)
			{

				 if (2 == i2c->msg->len){
					byte = Gpio_IIC_Read_Byte(1);

					i2c->msg->buf[i2c->msg_ptr++] = byte;

					byte = Gpio_IIC_Read_Byte(1);

					i2c->msg->buf[i2c->msg_ptr++] = byte;

				 }
	   prepare_read:
			  if (is_msglast(i2c)) { /* ylf: last byte of buffer */
				  /* last byte of buffer */
	   
				  if (is_lastmsg(i2c))
				   // halI2C_disable_ack(i2c); 
				   ret = i2c->msg_idx;
				   return ret;
			  } else if (is_msgend(i2c)) {
				  /* ok, we've read the entire buffer, see if there
				   * is anything else we need to do */
				  if (is_lastmsg(i2c)) {
					  /* last message, send stop and complete */
					  Gpio_IIC_Stop(i2c);
					  ret = 1;
				  } 
				  else {
					  /* go to the next transfer */
	   
					  i2c->msg_ptr = 0;
					  i2c->msg_idx++;
					  i2c->msg++;
				  }
			}
			  break;
		   }	
		 }
	 }
  out_ack:
	return ret;
}


int hal_i2c_xfer(struct i2c_adapter *adap,   
            struct i2c_msg *msgs, int num)
{
    struct hal_i2c *i2c = (struct hal_i2c *)adap->algo_data;
    int retry;
    int ret;
	
    for (retry = 0; retry < adap->retries; retry++) {

		ret = hal_Gpio_i2c_xfer(i2c, msgs, num);

        if (ret != -EAGAIN) {
            return ret;
        }
        
        delay_us(2);
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

	Gpio_IIC_Init();
	
    return &sI2C.adap;
}



