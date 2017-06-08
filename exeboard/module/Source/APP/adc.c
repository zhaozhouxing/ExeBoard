#include "stm32_eval.h"

#include "Timer_Driver.h"

#include "Adc_Driver.h"

#include "task.h"

#include "adc.h"

#include "stdio.h"


typedef struct
{
    multiplex_cb MultiCb;
    void *pPara;
    u32   ulADCResult;  
}ADC_STRU;

#define ADC_REFER_VOLT (3440)


#define CM_INPUT_MASK ((1 << 6)-1)

#define CM_MAX_INPUT_NUMBER (CM_MAX_NUMBER)

#define ADC_Oversampling_NUM  (16)       /*pow(4, ADC_Additional_Bits)*/

#define ADC_Additional_Bits 4   // should satisfy : 2^ADC_Additional_Bits =  ADC_Oversampling_NUM

#define ADC_Oversampling_Factor  (CM_MAX_NUMBER*ADC_Oversampling_NUM)       /*pow(4, ADC_Additional_Bits)*/

extern u16 ADC_ConvertedValue[ADC_Oversampling_NUM][CM_MAX_INPUT_NUMBER];

u16 ADC_ConvertedValue[ADC_Oversampling_NUM][CM_MAX_NUMBER];

static ADC_STRU saAdc[CM_MAX_NUMBER];


/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configures Vector Table base location.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void ADC_NVICConfiguration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Enable DMA channel1 IRQ Channel -----------------------------------------*/
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;//DMA1_Channel1_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0xf;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
}

void ADC_Timer_Msg(void)
{
    /* Update the Number of DMA transfer */ 
    DMA1_Channel1->CNDTR = ADC_Oversampling_Factor;
    
    /* Update the Destination Memory of the DMA pointer */ 
    DMA1_Channel1->CMAR = (u32)ADC_ConvertedValue;  
    
    DMA1_Channel1->CCR |= 0x00000001;

    ADC_Cmd(ADC1, ENABLE);
    
    ADC_SoftwareStartConvCmd(ADC1,ENABLE);

}


static int ADC_TIM2Handler(int Tim,int event,void *para)
{
    Message *Msg;

    Msg = MessageAlloc(PID_ADC,0);
    
    if (Msg)
    {
        Msg->msgHead.nMsgType = ADC_MESSAGE_TIMER_EVENT;
        Msg->msgHead.AddData = event;
        MessageSend(Msg);
    }
    return 0;
}

/*******************************************************************************
* Function Name  : TIM2_Configuration
* Description    : Configures the TIM2 to generate an interrupt after each 
*                     sampling period                  
* Input          : The required oversampling period in us
* Output         : None
* Return         : None
*******************************************************************************/
static void ADC_NextMultiplexing ( void )
{  
    u32 index = 0;
    
    for (index = 0; index < CM_MAX_NUMBER; index++)
    {
        if (saAdc[index].MultiCb) 
		(saAdc[index].MultiCb)(saAdc[index].pPara);
		printf("\r\nADC_NextMultiplexing\r\n");
    }
    
}  


/*******************************************************************************
* Function Name  : TIM2_Configuration
* Description    : Configures the TIM2 to generate an interrupt after each 
*                     sampling period                  
* Input          : The required oversampling period in us
* Output         : None
* Return         : None
*******************************************************************************/
static void ADC_TIM2Configuration ( u32 Sampling_Period )
{  
    int freq = 1000000/Sampling_Period;

    TIM_Init_General(TIMER1,freq);

    TIM_InstallHandler(TIMER1,TIM_IT_Update,ADC_TIM2Handler,NULL);

}  

/*******************************************************************************
* Function Name  : ADC_SetMultiplexer
* Description    : Configures the Multiplex for each adc channel 
*                                    
* Input          : callback
* Output         : None
* Return         : None
*******************************************************************************/
void ADC_SetMultiplexer ( uint8_t ucChl, multiplex_cb cb , void *para)
{  
    saAdc[ucChl].MultiCb = cb;
    saAdc[ucChl].pPara   = para;
}  


/*******************************************************************************
* Function Name  : CurrentMeas_Init
* Description    : Configures the Init current measurement 
* Input          : 
*       @param Sampling_Period: sampling period ,unit 100us
* Output         : None
* Return         : None
*******************************************************************************/
void ADC_Meas_Init(u32 Sampling_Period )
{
  /* Peripherals InitStructure define -----------------------------------------*/
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOC, ENABLE);

  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
  GPIO_InitStructure.GPIO_Speed = (GPIOSpeed_TypeDef)0;
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  ADC_NVICConfiguration();

  DMA_ADC_Configuration((u32)ADC_ConvertedValue,ADC_Oversampling_Factor,CM_MAX_NUMBER,ADC_SampleTime_7Cycles5);

  ADC_TIM2Configuration(Sampling_Period);
  
  ADC_SoftwareStartConvCmd(ADC1,ENABLE);

  
}

/*******************************************************************************
* Function Name  : u32 Oversampling_GetConversion
* Description    : Gives the ADC oversampled conversion result  
* Input          : None 
* Output         : Oversampled Result
* Return         : None
*******************************************************************************/
static void Oversampling_GetConversion ( void )
{
  u32 index = 0;
  u32 chloop = 0;

  for (index = 0; index < CM_MAX_NUMBER; index++)
  {
      saAdc[index].ulADCResult = 0;
  }
  
  for( index = 0; index < ADC_Oversampling_NUM ; index++)
  {
    for (chloop = 0; chloop < CM_MAX_NUMBER;chloop++)
    {
        saAdc[chloop].ulADCResult += ADC_ConvertedValue[index][chloop];
    }
  }

  for (index = 0; index < CM_MAX_NUMBER; index++)
  {
      saAdc[index].ulADCResult  >>=  ADC_Additional_Bits;
  }
    
}  

/*******************************************************************************
* Function Name  : GetAdcData
* Description    : get the current measurement 
* Input          : 
*       @param ucChl: channel number ,in range [0~2]
* Output         : None
* Return         : None
*******************************************************************************/
uint32_t GetAdcData(uint8_t ucChl)
{
    if (ucChl >= CM_MAX_NUMBER)
    {
        return 0;
    }
    return saAdc[ucChl].ulADCResult ;
}

void AdcDmaHandler(void)
{
    /* Clear the DMA Global interrupt bit DMA_IT_GL1*/
    DMA1->IFCR = DMA1_IT_GL1;
    
    /* Disable DMA channel1 */
    DMA1_Channel1->CCR &= 0xFFFFFFFE;
    
    /* Disable ADC */
    ADC_Cmd(ADC1, DISABLE);
    
    {
       // send Message
       Message *msg;
    
       msg = MessageAlloc(PID_ADC,0);
       if (msg)
       {
         msg->msgHead.nMsgType = ADC_MESSAGE_SESSION_CMP;
         msg->msgHead.MsgSeq = 0;
         MessageSend(msg);
       }
    }     

}

/*******************************************************************************
* Function Name  : PidAdcProcess
* Description    : PID proc for adc driver module 
* Input          : 
*       @param pMsg: message pointer
* Output         : None
* Return         : None
*******************************************************************************/
UINT8 PidAdcProcess(Message *pMsg)
{

    switch(pMsg->msgHead.nMsgType)
    {
    case ADC_MESSAGE_TIMER_EVENT:
        {
           // do something here
           ADC_Timer_Msg();
        }
        break;
    case ADC_MESSAGE_SESSION_CMP:
        {
            //uint16_t usValue ;
           // do something here
           /* Compute the oversampled result */
           Oversampling_GetConversion();

           //usValue = (GetAdcData(0)* ADC_REFER_VOLT /4096);

           // switch channel if any
           ADC_NextMultiplexing();           
        }
        break;
    default:
        break;
    }
    return 0;
}


