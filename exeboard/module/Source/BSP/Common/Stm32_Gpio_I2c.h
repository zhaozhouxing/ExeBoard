#include "sys.h"

#include "stm32_eval.h"

/* Includes ------------------------------------------------------------------*/
#include "i2c.h"


#define Gpio_I2c 1


//IO方向设置
//#define SDA_IN()  {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=8<<12;}
//#define SDA_OUT() {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=3<<12;}

#define SDA_IN()  
#define SDA_OUT() 

//IO操作函数	 
#define Gpio_IIC_SCL    PBout(6) //SCL
#define Gpio_IIC_SDA    PBout(7) //SDA	 
#define Gpio_READ_SDA   PBin(7)  //输入SDA 





