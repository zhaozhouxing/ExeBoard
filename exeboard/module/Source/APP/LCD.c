
#include    <ucos_ii.h>

#include    <string.h>

#include "stm32f10x.h"

#include "stm32_eval.h"

#include "LCD.h"

#include "font.h"

#define LCD_Pin_RS      GPIO_Pin_1
#define LCD_PORT_RS     GPIOA
#define LCD_CLK_RS      RCC_APB2Periph_GPIOA

#define LCD_Pin_WR      GPIO_Pin_0
#define LCD_PORT_WR     GPIOB
#define LCD_CLK_WR      RCC_APB2Periph_GPIOB

#define LCD_Pin_RD      GPIO_Pin_1
#define LCD_PORT_RD     GPIOB
#define LCD_CLK_RD      RCC_APB2Periph_GPIOB

#define LCD_Pin_RESET   GPIO_Pin_2
#define LCD_PORT_RESET  GPIOB
#define LCD_CLK_RESET   RCC_APB2Periph_GPIOB

#define LCD_Pin_CS      GPIO_Pin_2
#define LCD_PORT_CS     GPIOD
#define LCD_CLK_CS      RCC_APB2Periph_GPIOD

#define LCD_Pin_BL      GPIO_Pin_3
#define LCD_PORT_BL     GPIOB
#define LCD_CLK_BL      RCC_APB2Periph_GPIOB

#define LCD_DELAY_NUM 1

#define LCD_DELAY(tm) do {int i= 0; while (i++ < tm);}while(0);


#define Set_nCs()            do {GPIO_HIGH(LCD_PORT_CS, LCD_Pin_CS);LCD_DELAY(LCD_DELAY_NUM);}while(0);
#define Clr_nCs()            do {GPIO_LOW(LCD_PORT_CS, LCD_Pin_CS);LCD_DELAY(LCD_DELAY_NUM);}while(0);
 

/* Serves as a write signal and writes data at the rising edge*/
#define Set_nWr()            do {GPIO_HIGH(LCD_PORT_WR, LCD_Pin_WR);LCD_DELAY(LCD_DELAY_NUM);}while(0);
#define Clr_nWr()            do {GPIO_LOW(LCD_PORT_WR, LCD_Pin_WR);LCD_DELAY(LCD_DELAY_NUM);}while(0);

/*Data or Command , 1 data , 0 register */
#define Set_Rs()             do {GPIO_HIGH(LCD_PORT_RS, LCD_Pin_RS);LCD_DELAY(LCD_DELAY_NUM);}while(0);
#define Clr_Rs()             do {GPIO_LOW(LCD_PORT_RS, LCD_Pin_RS);LCD_DELAY(LCD_DELAY_NUM);}while(0);

/* Serves as a read signal and MCU read data at the rising edge*/
#define Set_nRd()            do {GPIO_HIGH(LCD_PORT_RD, LCD_Pin_RD);LCD_DELAY(LCD_DELAY_NUM);}while(0);
#define Clr_nRd()            do {GPIO_LOW(LCD_PORT_RD, LCD_Pin_RD);LCD_DELAY(LCD_DELAY_NUM);}while(0);

#define Set_nRst()           do {GPIO_HIGH(LCD_PORT_RESET, LCD_Pin_RESET);LCD_DELAY(LCD_DELAY_NUM);}while(0);
#define Clr_nRst()           do {GPIO_LOW(LCD_PORT_RESET, LCD_Pin_RESET);LCD_DELAY(LCD_DELAY_NUM);}while(0);

#define Set_nBL()            do {GPIO_HIGH(LCD_PORT_BL, LCD_Pin_BL);LCD_DELAY(LCD_DELAY_NUM);}while(0);
#define Clr_nBL()            do {GPIO_LOW(LCD_PORT_BL, LCD_Pin_BL);LCD_DELAY(LCD_DELAY_NUM);}while(0);
#define Get_nBL()            GPIO_STATE_LOW(LCD_PORT_BL, LCD_Pin_BL)


//#define LCD_Write(LCD_DATA)  do {GPIO_Write(GPIOB, LCD_DATA);LCD_DELAY(LCD_DELAY_NUM);}while(0);
//#define LCD_Read()           GPIO_ReadInputData(GPIOB);

#define LCD_Status(LCD_DATA)  (GPIOB->ODR)

#define LCD_DATA_LOW_BIT_POS (1)
#define LCD_DATA_LOW_BITS    (8)
#define LCD_DATA_LOW_MASK    (((1<<LCD_DATA_LOW_BITS)-1)<<LCD_DATA_LOW_BIT_POS)
#define LCD_DATA_LOW_NMASK   (~LCD_DATA_LOW_MASK)

#define LCD_DATA_HIGH0_BIT_POS  (4)
#define LCD_DATA_HIGH0_BITS     (2)
#define LCD_DATA_HIGH0_MASK     (((1<<LCD_DATA_HIGH0_BITS)-1)<<LCD_DATA_HIGH0_BIT_POS)
#define LCD_DATA_HIGH0_NMASK    (~LCD_DATA_HIGH0_MASK)

#define LCD_DATA_HIGH1_BIT_POS (10)
#define LCD_DATA_HIGH1_BITS    (6)
#define LCD_DATA_HIGH1_MASK    (((1<<LCD_DATA_HIGH1_BITS)-1)<<LCD_DATA_HIGH1_BIT_POS)
#define LCD_DATA_HIGH1_NMASK   (~LCD_DATA_HIGH1_MASK)

#define LCD_DATA_HIGH_MASK     (LCD_DATA_HIGH0_MASK|LCD_DATA_HIGH1_MASK)
#define LCD_DATA_HIGH_NMASK    (~LCD_DATA_HIGH_MASK)

#ifndef  Bus_8    //��������-16λ����ģʽ   
void LCD_Writ_Bus(uint16_t data)   //��������д�뺯��
{
    uint16_t tmp = GPIOC->ODR;

    // low 8bits
    tmp &= LCD_DATA_LOW_NMASK;
    tmp |= (data << LCD_DATA_LOW_BIT_POS) & LCD_DATA_LOW_MASK;
    GPIOC->ODR = tmp;

    // high 8bits
    tmp = GPIOB->ODR;
    tmp &= (LCD_DATA_HIGH_NMASK);
    tmp |= ((data >> 8) << LCD_DATA_HIGH0_BIT_POS) & LCD_DATA_HIGH0_MASK;
    tmp |= ((data >> (8+LCD_DATA_HIGH0_BITS)) << LCD_DATA_HIGH1_BIT_POS) & LCD_DATA_HIGH1_MASK;
    GPIOB->ODR = tmp;
    
    Clr_nWr();
    //__nop();
    Set_nWr(); 
    //__nop();
}
#else           //��������-8λ����ģʽ 
void LCD_Writ_Bus(uint16_t data)   //��������д�뺯��
{   
    uint16_t tmp = GPIOC->ODR;

    tmp &= LCD_DATA_LOW_NMASK;
    tmp |= (data << LCD_DATA_LOW_BIT_POS) & LCD_DATA_LOW_MASK;
    GPIOC->ODR = tmp;
    tmp = GPIOC->ODR;

    Clr_nWr();
    Set_nWr();
    
    tmp &= LCD_DATA_LOW_NMASK;
    tmp |= ((data >> 8)<< LCD_DATA_LOW_BIT_POS) & LCD_DATA_LOW_MASK;
    GPIOC->ODR = tmp;
    tmp = GPIOC->ODR;
    
    Clr_nWr();
    Set_nWr();
}
#endif

void LCD_WR_DATA(uint8_t dataH,uint8_t dataL)
{
    Set_Rs();
    Clr_nCs();

    LCD_Writ_Bus((dataH << 8)|dataL);

    Set_nCs();
}   

void LCD_WR_DATA2(uint16_t data)
{
    Set_Rs();
    Clr_nCs();

    LCD_Writ_Bus(data);

    Set_nCs();
}     


void LCD_WR_CMD(uint8_t cmd, uint8_t data)   
{   
    Clr_Rs();
    Clr_nCs();
    LCD_Writ_Bus((cmd << 8)| data);
    Set_nCs();
}

void SetPointAddress(int x,int y)
{
#ifdef LCD_HORIZON_DISP_MODE
    LCD_WR_CMD(0x00,0x2a); // column address set
    LCD_WR_DATA(0x00,y>>8); // c start
    LCD_WR_DATA(0x00,y);
    LCD_WR_DATA(0x00,(y+1)>>8); // c end
    LCD_WR_DATA(0x00,(y+1));

    LCD_WR_CMD(0x00,0x2b);
    LCD_WR_DATA(0x00,(LCD_W-1-(x+1))>>8); // row start
    LCD_WR_DATA(0x00,LCD_W-1-(x+1));
    LCD_WR_DATA(0x00,(LCD_W-1-x)>>8); // rpw emd
    LCD_WR_DATA(0x00,LCD_W-1-x);

#else
    LCD_WR_CMD(0x00,0x2a);
    LCD_WR_DATA(0x00,x>>8);
    LCD_WR_DATA(0x00,x);
    LCD_WR_DATA(0x00,(x+1)>>8);
    LCD_WR_DATA(0x00,(x+1));

    LCD_WR_CMD(0x00,0x2b);
    LCD_WR_DATA(0x00,y>>8);
    LCD_WR_DATA(0x00,y);
    LCD_WR_DATA(0x00,(y+1)>>8);
    LCD_WR_DATA(0x00,(y+1));
#endif    
}

 
void LCD_AddressSet(unsigned int x1,unsigned int y1,unsigned int x2,unsigned int y2)
{ 
    if (x1 >= LCD_W)  x1 = LCD_W -1;
    if (y1 >= LCD_H)  y1 = LCD_H -1;

    if (x2 >= LCD_W)  x2 = LCD_W -1;
    if (y2 >= LCD_H)  y2 = LCD_H -1;

#ifdef LCD_HORIZON_DISP_MODE
    LCD_WR_CMD(0x00,0x2a); // column address set
    LCD_WR_DATA(0x00,y1>>8); // c start
    LCD_WR_DATA(0x00,y1);
    LCD_WR_DATA(0x00,y2>>8); // c end
    LCD_WR_DATA(0x00,y2);

    LCD_WR_CMD(0x00,0x2b);
    LCD_WR_DATA(0x00,(LCD_W-1-x2)>>8); // row start
    LCD_WR_DATA(0x00,LCD_W-1-x2);
    LCD_WR_DATA(0x00,(LCD_W-1-x1)>>8); // rpw emd
    LCD_WR_DATA(0x00,LCD_W-1-x1);

#else
    LCD_WR_CMD(0x00,0x2a); // column address set
    LCD_WR_DATA(0x00,x1>>8);
    LCD_WR_DATA(0x00,x1);
    LCD_WR_DATA(0x00,x2>>8);
    LCD_WR_DATA(0x00,x2);

    LCD_WR_CMD(0x00,0x2b); // row address set
    LCD_WR_DATA(0x00,y1>>8);
    LCD_WR_DATA(0x00,y1);
    LCD_WR_DATA(0x00,y2>>8);
    LCD_WR_DATA(0x00,y2);
#endif  
    LCD_WR_CMD(0x00,0x2c);          // ylf ->Write Data to GRAM                                             

}


void LCD_SetCursor(u16 x,u16 y)
{
    LCD_AddressSet(x,y,LCD_W-1,LCD_H-1);
}

void LCD_SetPoint(int x,int y,unsigned int color)
{
    SetPointAddress(x,y);
    LCD_WR_CMD(0x00,0x2c);
    LCD_WR_DATA2(color);     

}

void LCD_ChipInit(void)
{
    Set_nRd();
    Set_nWr();
    Set_nRst();
    OSTimeDlyHMSM(0,0,0,10);
    Clr_nRst();
    OSTimeDlyHMSM(0,0,0,20);
    Set_nRst();
    OSTimeDlyHMSM(0,0,0,20);
    
    LCD_WR_CMD(0x00,0x11); 
    OSTimeDlyHMSM(0,0,0,20);

    // ADD SHZN (MADCTL)
    LCD_WR_CMD(0x00,0x36);  
#ifdef LCD_HORIZON_DISP_MODE
    /* MV =1, MX =0, MY= 1*/
    LCD_WR_DATA(0x00,0X00);  /* SEEMS 00 IS a valid combination */
#else
    LCD_WR_DATA(0x00,0xc0); 
#endif    
    
    LCD_WR_CMD(0x00,0x3A); 
    LCD_WR_DATA(0x00,0X55); 
     
    LCD_WR_CMD(0x00,0xB2);  
    LCD_WR_DATA(0x00,0x0C); 
    LCD_WR_DATA(0x00,0x0C); 
    LCD_WR_DATA(0x00,0x00); 
    LCD_WR_DATA(0x00,0x33);
    LCD_WR_DATA(0x00,0x33);
    
    LCD_WR_CMD(0x00,0xB7);  
    LCD_WR_DATA(0x00,0x35); 
    
    LCD_WR_CMD(0x00,0xBB);                  //vcom
    LCD_WR_DATA(0x00,0x30);    //30
    
    LCD_WR_CMD(0x00,0xC0);                    //
    LCD_WR_DATA(0x00,0x2C); 
    
    LCD_WR_CMD(0x00,0xC2);  
    LCD_WR_DATA(0x00,0x01); 
     
    LCD_WR_CMD(0x00,0xC3);         //vrh
    LCD_WR_DATA(0x00,0x17);        //17        0D
    
    LCD_WR_CMD(0x00,0xC4);                 //vdv
    LCD_WR_DATA(0x00,0x20);   //20
    
    LCD_WR_CMD(0x00,0xC6);  
    LCD_WR_DATA(0x00,0x0F);
    
    LCD_WR_CMD(0x00,0xCA);  
    LCD_WR_DATA(0x00,0x0F);
    
    LCD_WR_CMD(0x00,0xC8);  
    LCD_WR_DATA(0x00,0x08);
    
    
    LCD_WR_CMD(0x00,0x55);  
    LCD_WR_DATA(0x00,0x90);     //90
    
    
    LCD_WR_CMD(0x00,0xD0);  
    LCD_WR_DATA(0x00,0xA4); 
    LCD_WR_DATA(0x00,0xA1); 
     
    LCD_WR_CMD(0x00,0xE0);    //Set Gamma 
    LCD_WR_DATA(0x00,0xf0); 
    LCD_WR_DATA(0x00,0x00); 
    LCD_WR_DATA(0x00,0x0a); 
    LCD_WR_DATA(0x00,0x10); 
    LCD_WR_DATA(0x00,0x12); 
    LCD_WR_DATA(0x00,0x1b); 
    LCD_WR_DATA(0x00,0x39); 
    LCD_WR_DATA(0x00,0X44); 
    LCD_WR_DATA(0x00,0x47); 
    LCD_WR_DATA(0x00,0x28); 
    LCD_WR_DATA(0x00,0x12); 
    LCD_WR_DATA(0x00,0x10); 
    LCD_WR_DATA(0x00,0x16);
    LCD_WR_DATA(0x00,0x1B); 
     
     
    LCD_WR_CMD(0x00,0XE1);    //Set Gamma 
    LCD_WR_DATA(0x00,0xf0); 
    LCD_WR_DATA(0x00,0x00); 
    LCD_WR_DATA(0x00,0x0a); 
    LCD_WR_DATA(0x00,0x10); 
    LCD_WR_DATA(0x00,0x11); 
    LCD_WR_DATA(0x00,0x1a); 
    LCD_WR_DATA(0x00,0x3b); 
    LCD_WR_DATA(0x00,0x34); 
    LCD_WR_DATA(0x00,0x4e); 
    LCD_WR_DATA(0x00,0x3a); 
    LCD_WR_DATA(0x00,0x17); 
    LCD_WR_DATA(0x00,0x16); 
    LCD_WR_DATA(0x00,0x21); 
    LCD_WR_DATA(0x00,0x22); 
    
    
    LCD_WR_CMD(0x00,0x2A); 
    LCD_WR_DATA(0x00,0x00);
    LCD_WR_DATA(0x00,0x00);
    LCD_WR_DATA(0x00,0x00);
    LCD_WR_DATA(0x00,0xef);  
    
    LCD_WR_CMD(0x00,0x2B); 
    LCD_WR_DATA(0x00,0x00);
    LCD_WR_DATA(0x00,0x00);
    LCD_WR_DATA(0x00,0x01);
    LCD_WR_DATA(0x00,0x3f);
     
    LCD_WR_CMD(0x00,0x29); //display on
    LCD_WR_CMD(0x00,0x2c);

    Clr_nBL();

    OSTimeDlyHMSM(0,0,0,20);   // ylf: Delay Some TIME ??

}

void LCD_Init(void)
{
    // gpio init
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_APB2PeriphClockCmd(LCD_CLK_RS | RCC_APB2Periph_GPIOB |RCC_APB2Periph_GPIOC|
                           LCD_CLK_WR | LCD_CLK_RD |LCD_CLK_RESET|
                           LCD_CLK_CS |LCD_CLK_BL, ENABLE);
    
    // DB7--0
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // DB15--8
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    //LCD_Pin_WR
    GPIO_InitStructure.GPIO_Pin = LCD_Pin_WR;
    GPIO_Init(LCD_PORT_WR, &GPIO_InitStructure);
    
    //LCD_Pin_CS
    GPIO_InitStructure.GPIO_Pin = LCD_Pin_CS;
    GPIO_Init(LCD_PORT_CS, &GPIO_InitStructure);
    
    //LCD_Pin_RS
    GPIO_InitStructure.GPIO_Pin = LCD_Pin_RS;
    GPIO_Init(LCD_PORT_RS, &GPIO_InitStructure);
    
    //LCD_Pin_RD
    GPIO_InitStructure.GPIO_Pin = LCD_Pin_RD;
    GPIO_Init(LCD_PORT_RD, &GPIO_InitStructure);

    //LCD_Pin_RESET
    GPIO_InitStructure.GPIO_Pin = LCD_Pin_RESET;
    GPIO_Init(LCD_PORT_RESET, &GPIO_InitStructure);

    //LCD_Pin_BL
    GPIO_InitStructure.GPIO_Pin = LCD_Pin_BL;
    GPIO_Init(LCD_PORT_BL, &GPIO_InitStructure);
    
    LCD_ChipInit();
}

void LCD_Clear(u16 Color)
{
    u16 i,j;
    
    LCD_AddressSet(0,0,LCD_W-1,LCD_H-1);
    
    for(i = 0;i< LCD_W; i++)
    {
        for (j = 0; j < LCD_H; j++)
        {
            LCD_WR_DATA2(Color );
        }
   
    }
}

void LCD_ClearRect(u16 x1,u16 y1,u16 x2,u16 y2,u16 Color)
{
    u16 i,j;

    LCD_AddressSet(x1,y1,x2,y2);
    
    for(i = x1;i<= x2; i++)
    {
        for (j = y1; j <= y2; j++)
        {
            LCD_WR_DATA2(Color);
        }
   
    }
}


void LCD_DrawPoint(u16 x,u16 y,u16 color)
{
    LCD_AddressSet(x,y,x+1,y+1);//���ù��λ�� 
    LCD_WR_DATA2(color);         
}    

void LCD_DrawPoint_big(u16 x,u16 y,u16 color)
{
    LCD_Fill(x-1,y-1,x+1,y+1,color);
} 

void LCD_Fill(u16 xsta,u16 ysta,u16 xend,u16 yend,u16 color)
{    
    u16 i,j; 

    LCD_AddressSet(xsta,ysta,xend,yend);   
    
    for(i=ysta;i<=yend;i++)
    {                                                           
        for(j=xsta;j<=xend;j++)
        {      
            LCD_WR_DATA2(color);
        }
    }   

}  

void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2,u16 color)
{
    u16 t; 
    int xerr=0,yerr=0,delta_x,delta_y,distance; 
    int incx,incy,uRow,uCol; 

    delta_x=x2-x1; //������������ 
    delta_y=y2-y1; 
    uRow=x1; 
    uCol=y1; 
    if(delta_x>0)incx=1; //���õ������� 
    else if(delta_x==0)incx=0;//��ֱ�� 
    else {incx=-1;delta_x=-delta_x;} 
    if(delta_y>0)incy=1; 
    else if(delta_y==0)incy=0;//ˮƽ�� 
    else{incy=-1;delta_y=-delta_y;} 
    if( delta_x>delta_y)distance=delta_x; //ѡȡ�������������� 
    else distance=delta_y; 
    for(t=0;t<=distance+1;t++ )//������� 
    {  
        LCD_DrawPoint(uRow,uCol,color);//���� 
        xerr+=delta_x ; 
        yerr+=delta_y ; 
        if(xerr>distance) 
        { 
            xerr-=distance; 
            uRow+=incx; 
        } 
        if(yerr>distance) 
        { 
            yerr-=distance; 
            uCol+=incy; 
        } 
    }  
    
    
}  

//������
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2,u16 color)
{
    LCD_DrawLine(x1,y1,x2,y1,color);
    LCD_DrawLine(x1,y1,x1,y2,color);
    LCD_DrawLine(x1,y2,x2,y2,color);
    LCD_DrawLine(x2,y1,x2,y2,color);
}

void Draw_Circle(u16 x0,u16 y0,u8 r,u16 color)
{
    int a,b;
    int di;
    a=0;b=r;      
    di=3-(r<<1);             //�ж��¸���λ�õı�־
    while(a<=b)
    {
        LCD_DrawPoint(x0-b,y0-a,color);             //3           
        LCD_DrawPoint(x0+b,y0-a,color);             //0           
        LCD_DrawPoint(x0-a,y0+b,color);             //1       
        LCD_DrawPoint(x0-b,y0-a,color);             //7           
        LCD_DrawPoint(x0-a,y0-b,color);             //2             
        LCD_DrawPoint(x0+b,y0+a,color);             //4               
        LCD_DrawPoint(x0+a,y0-b,color);             //5
        LCD_DrawPoint(x0+a,y0+b,color);             //6 
        LCD_DrawPoint(x0-b,y0+a,color);             
        a++;
        //ʹ��Bresenham�㷨��Բ     
        if(di<0)di +=4*a+6;   
        else
        {
            di+=10+4*(a-b);   
            b--;
        } 
        LCD_DrawPoint(x0+a,y0+b,color);
    }
} 

void LCD_PutChar8x16(u16 x,u16 y,u8 c,u16 charColor,u16 bkColor)
{
    u16 i=0;
    u16 j=0;
    
    u8 tmp_char=0;

    Clr_nCs();                  
  
    for (i = 0; i < 16; i++) // YLF: 16 ROWS
    {
        //tmp_char = asc2_1608[((c-0x20)*16)+i];
        tmp_char = ascii_8x16[((c-0x20)*16)+i];        
        for (j = 0;j < 8;j++)
        {
            if ( (tmp_char >> 7-j) & 0x01 == 0x01)
            {
              LCD_SetPoint(x+j,y+i,charColor); // �ַ���ɫ
            }
            else
            {
              LCD_SetPoint(x+j,y+i,bkColor); // ������ɫ
            }
        }
    }
    
    Set_nCs();                  
}


void LCD_PutString8x16(u16 x,u16 y,u8 *string,u16 charColor,u16 bkColor)
{
    while(*string != '\0')
    {       
        if(x > LCD_W-8) // move to next line
        {
            //x = 0; 
            //y += 16;
            
            break;
        }
        
        if(y > LCD_H-16)
        {
            //y = x = 0;
            //LCD_Clear(RED);
            
            break;
        }
        
        LCD_PutChar8x16(x,y,*string,charColor,bkColor);
        x += 8;
        string++;
    }  
                  
}

/*****************************************************************************************
Function: calculate text size
input : texgt
output : sx  size for x cord
            sy size for y cord
******************************************************************************************/
void LCD_GetStringGB8x16TextSize(u8 *s, int* sx, int* sy) 
{
    int offset = strlen((char *)s);

    *sx = offset*8; 
    *sy = offset*18; 
}


void LCD_PutChar12x24(u16 x,u16 y,u8 c,u16 charColor,u16 bkColor)
{
    u16 i=0;
    u16 j=0;
    
    u16 tmp_char=0;

    Clr_nCs();                  
  
    for (i = 0; i < 24; i++) // YLF: 16 ROWS
    {
        tmp_char = (ASCII_12x24[(c-0x20)*48 + 2*i] << 8)|ASCII_12x24[(c-0x20)*48 + 2*i+1];        
        for (j = 0;j < 12;j++)
        {
            if ( tmp_char & 0x8000)
            {
              LCD_SetPoint(x+j,y+i,charColor); // �ַ���ɫ
            }
            else
            {
              LCD_SetPoint(x+j,y+i,bkColor); // ������ɫ
            }
            tmp_char <<= 1;
        }
    }
    
    Set_nCs();                  
}

void LCD_PutString12x24(u16 x,u16 y,char *string,u16 charColor,u16 bkColor)
{
    while(*string != '\0')
    {       
        if(x >= LCD_W - 12) // move to next line
        {
            //x  = 0; 
            //y += 32;
            break;
        }
        
        if(y >= LCD_H - 24)
        {
            //y = x = 0;
            //LCD_Clear(RED);
            break;
        }
        
        LCD_PutChar12x24(x,y,*string,charColor,bkColor);
        x += 12;
        string++;
    }  
                  
}

#define pgm_read_byte(p)(*p)

void LCD_PutChar24x48(u16 x,u16 y,u8 c,u16 charColor,u16 bkColor)
{
    u16 i=0;
    u16 j=0;
    
    u8 m = 0;
  
    const u8 *pdata = &ascii_24x48[(c-0x20)*144];
    
    Clr_nCs();                  
    
    LCD_AddressSet(x,y,x+24-1,y+48-1);  
    
    for(i  =0; i < 144; i++)
    {
        m = pgm_read_byte(&pdata[i]);//��ȡc�ַ��ĵ�i���ֽ���,c��ȥ0x20������Ascii����е�0~1f��ȥ��
        for(j=0;j<8;j++)
        {                       
            if((m&0x80)==0x80) 
            {                                     //�ж����λ�Ƿ�Ϊ1
                LCD_WR_DATA2(charColor);           //���λΪ1��д���ַ���ɫ
            }
            else 
            {
                LCD_WR_DATA2(bkColor);           //���λΪ0��д�뱳����ɫ
            }
            m<<=1;                                //����1λ��׼��д��һλ
        }
    }
    Set_nCs();                  
}

void LCD_PutString24x48(u16 x,u16 y,char *string,u16 charColor,u16 bkColor)
{
    while(*string != '\0')
    {       
        if(x >= LCD_W-24) // move to next line
        {
            //x  = 0; 
            //y += 32;
            break;
        }
        
        if(y >= LCD_H-48)
        {
            //y = x = 0;
            //LCD_Clear(RED);
            break;
        }
        
        LCD_PutChar24x48(x,y,*string,charColor,bkColor);
        x += 24;
        string++;
    }  
                  
}



#ifdef FONT16x33
void LCD_PutChar16x33(unsigned short x, unsigned short y, unsigned char c, unsigned int fColor, unsigned int bColor)
{
   unsigned int i,j,m;
   Clr_nCs();                  
   
   LCD_AddressSet(x,y,x+16-1,y+33-1);                    //�����ַ���ʾλ��
   for(i=0;i<66;i++)
   {                                               //ѭ��д��16�ֽڣ�һ���ַ�Ϊ16�ֽ�
        m = pgm_read_byte(&Font16x33[c*66+i]);      //��ȡc�ַ��ĵ�i���ֽ���,c��ȥ0x20������Ascii����е�0~1f��ȥ��
        for(j=0;j<8;j++) 
        {                        //ѭ��д��8λ��һ���ֽ�Ϊ8λ
            if((m&0x80)==0x80) 
            {                  //�ж����λ�Ƿ�Ϊ1
                LCD_WR_DATA(fColor);           //���λΪ1��д���ַ���ɫ
            }
            else 
            {
                LCD_WR_DATA(bColor);           //���λΪ0��д�뱳����ɫ
            }
            m<<=1;                                //����1λ��׼��д��һλ
        }
   }
   Set_nCs();                  
}

void LCD_PutChar24x48(unsigned short x, unsigned short y, unsigned char c, unsigned int fColor, unsigned int bColor)
{
    unsigned int i,j,m;
    Clr_nCs();                  
    LCD_AddressSet(x,y,x+24-1,y+48-1);                    //�����ַ���ʾλ��
    for(i=0;i<144;i++) 
    {                             //ѭ��д��16�ֽڣ�һ���ַ�Ϊ16�ֽ�
        m=pgm_read_byte(&Font24x48[c*144+i]);//��ȡc�ַ��ĵ�i���ֽ���,c��ȥ0x20������Ascii����е�0~1f��ȥ��
        for(j=0;j<8;j++)
        {                        //ѭ��д��8λ��һ���ֽ�Ϊ8λ
            if((m&0x80)==0x80) 
            {                  //�ж����λ�Ƿ�Ϊ1
                LCD_WR_DATA(fColor);           //���λΪ1��д���ַ���ɫ
            }
            else 
            {
                LCD_WR_DATA(bColor);           //���λΪ0��д�뱳����ɫ
            }
            m<<=1;                                //����1λ��׼��д��һλ
        }
    }
    
    Set_nCs();                  
}
#endif


#if 0
void PutGB24x48(unsigned short x, unsigned short  y, unsigned char c[2], unsigned int fColor,unsigned int bColor)
{
    unsigned int i,j,k;
    Clr_nCs();                  

    LCD_AddressSet(x, y,x+24-1,y+48-1);

    for (k=0;k<GetcodeGB_49_LENGTH();k++) //11��ʾ�Խ����ֿ��еĸ�����ѭ����ѯ����
    { 
        if ((pgm_read_byte(&codeGB_49[k].Index[0])==c[0])
             &&(pgm_read_byte(&codeGB_49[k].Index[1])==c[1]))
        {
            for(i=0;i<144;i++) 
            {
                 unsigned char m = pgm_read_byte(&codeGB_49[k].Msk[i]); 
                 for(j=0;j<8;j++) 
                 {
                     if((m&0x80)==0x80) 
                     {
                         LCD_WR_DATA(fColor);
                     }
                     else 
                     {
                         LCD_WR_DATA(bColor);
                     }
                     m<<=1;
                 }
            }
            break;
         }
    }
    Set_nCs();                  

}
#endif
//-----------------��ʾ32X32�ĺ��֣�fColorΪ����ɫ��bColorΪ����ɫ----------------------------//

#ifdef CODEGB_32

void PutGB3232(u16 x, u16  y, unsigned char c[2], u16 fColor,u16 bColor)
{
    unsigned int i,j,k;

    Clr_nCs();                  
    LCD_AddressSet(x,y,x+32-1,y+32-1);

    for (k = 0;k< GetcodeGB_32_LENGTH();k++) // //15��ʾ�Խ����ֿ��еĸ�����ѭ����ѯ����
    { 
      if ((pgm_read_byte(&codeGB_32[k].Index[0]) == c[0])
          &&(pgm_read_byte(&codeGB_32[k].Index[1]) ==c[1]))
      {  
          for(i = 0; i< 128; i++) 
          {                     
              unsigned char m = pgm_read_byte(&codeGB_32[k].Msk[i]);
              for(j=0;j<8;j++)
              {
                  if((m&0x80)==0x80) 
                  {
                      LCD_WR_DATA2(fColor);
                  }
                  else
                  {
                      LCD_WR_DATA2(bColor);
                  }
                  m<<=1;
              } 
           }
           break;
        }  
    }
    
    Set_nCs();                  
}

void PutStringGB3232(u16 x, u16 y, u8 *s, u16 fColor, u16 bColor)
{
    unsigned char l=0;
    while(*s) 
    {
        if( *s < 0x80) 
        {
          //LCD_PutChar(y+l*8,x,*s,fColor,bColor);
          s++;l++;
        }
        else
        {
            PutGB3232(x+l*16,y,(unsigned char*)s,fColor,bColor);
            s+=2;
            l+=2;
        }
    }
}

void LCD_GetStringGB32x32TextSize(u8 *s, int* sx, int* sy) 
{
    int offset = strlen((char *)s);

    *sx = offset*16; 
    *sy = offset*32; 
}


#endif

//-----------------��ʾ32X32�ĺ��֣�fColorΪ����ɫ��bColorΪ����ɫ----------------------------//
#ifdef CODEGB_48

void PutGB4848(u16 x, u16  y, u8 c[2], u16 fColor,u16 bColor)
{
    unsigned int i,j,k;

    Clr_nCs();                  
    LCD_AddressSet(x,y,x+48-1,y+48-1);

    for (k = 0; k < GetcodeGB_48_LENGTH(); k++) //15��ʾ�Խ����ֿ��еĸ�����ѭ����ѯ����
    { 
        if ((pgm_read_byte(&codeGB_48[k].Index2[0]) == c[0])
           &&(pgm_read_byte(&codeGB_48[k].Index2[1]) == c[1]))
        {    
            for(i=0;i<288;i++) 
            {                     
                unsigned char m =pgm_read_byte(&codeGB_48[k].Msk2[i]);
                for(j=0;j<8;j++) 
                {
                    if((m&0x80)==0x80) 
                    {
                        LCD_WR_DATA2(fColor);
                    }
                    else 
                    {
                        LCD_WR_DATA2(bColor);
                    }
                    m<<=1;
                } 
            }
            break;
        }  
    }
  
    Set_nCs();                  
}

void PutStringGB4848(u16 x, u16 y, u8 *s, u16 fColor, u16 bColor)
{
    unsigned char l=0;
    while(*s) 
    {
        if( *s < 0x80) 
        {
          LCD_PutChar24x48(x + l*24,y,*s,fColor,bColor);
          s++;l++;
        }
        else
        {
            PutGB4848(x + l*24,y,(unsigned char*)s,fColor,bColor);
            s+=2;l+=2;
         }
    }
}
void LCD_GetStringGB48x48TextSize(u8 *s, int* sx, int* sy) 
{
    int offset = strlen((char *)s);

    *sx = offset*24; 
    *sy = offset*48; 
}


#endif
/***********************************************************************************************************
��������: ��ʾ����24x24
���������x ������
          y ������
          c ��Ҫ��ʾ�ĺ�����
          fColor �ַ���ɫ
          bColor �ַ�������ɫ
***********************************************************************************************************/
void PutGB2424(u16 x, u16  y, u8 c[2], u16 fColor,u16 bColor)
{
    unsigned int i,j,k;

    Clr_nCs();   

    for (k = 0;k<GetcodeGB_24_LENGTH();k++) 
    { 
        if ((pgm_read_byte(&codeGB_24[k].Index[0]) == c[0])
          &&(pgm_read_byte(&codeGB_24[k].Index[1]) == c[1]))
        {
        
            for(i = 0; i < 24; i++) 
            {    
                 int col;
                 for (col = 0; col < 3; col ++)
                 {
                     unsigned char m = pgm_read_byte(&codeGB_24[k].Msk[i*3 + col]); 
                     
                     for(j=0;j<8;j++) 
                     {
                         if((m&0x80)==0x80) 
                         {
                             LCD_SetPoint(x+j+col*8,y+i,fColor); // �ַ���ɫ
                         }
                         else
                         {
                             LCD_SetPoint(x+j+col*8,y+i,bColor); // ������ɫ
                         }
                         m<<=1;
                     }
                 }
            }
            break;
        }
    }

    Set_nCs();                  
}

/*****************************************************************************************
��������:��ʾ�ַ���,������Ӣ��ͬʱ��ʾ
���������x ������
          y ������
         *s ��Ҫ��ʾ���ַ���
      fColor �ַ���ɫ
          bColor �ַ�������ɫ
******************************************************************************************/

void LCD_PutStringGB24x24(u16 x, u16 y, u8 *s, u16 fColor, u16 bColor) 
{
    int offset = 0;
    while(*s) 
    {
        if( *s < 0x80) 
        {
            LCD_PutChar12x24(x + offset,y,*s,fColor,bColor);
            offset += 12;
            s++;
        }
        else
        {
            PutGB2424(x + offset,y,(unsigned char*)s,fColor,bColor);
            s += 2;
            offset += 24;
            
        }
    }
}

/*****************************************************************************************
Function: calculate text size
input : texgt
output : sx  size for x cord
            sy size for y cord
******************************************************************************************/
void LCD_GetStringGB24x24TextSize(u8 *s, int* sx, int* sy) 
{
    int offset = strlen((char *)s);

    *sx = offset*12; 
    *sy = offset*24; 
}


//m^n����
u32 mypow(u8 m,u8 n)
{
    u32 result=1;    
    while(n--)result*=m;    
    return result;
}            


void LCD_EnableBackLight(int bEnable)
{
    if (bEnable)
    {
        Clr_nBL();
    }
    else
    {
        Set_nBL();
    }
}

int LCD_GetBackLightState(void)
{
    return Get_nBL();
}


