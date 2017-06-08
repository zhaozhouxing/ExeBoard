#include <string.h>
#include <stdio.h>
#include "stm32_eval.h"


#include "bmp.h"

#include "sys_time.h"

#include "touch.h"

#include "button.h"

#include "lcd.h"

#ifdef BUTTON_DBG_INFO
#define btn_dbg(fmt,arg...) printf(fmt,## arg)
#else
#define btn_dbg(fmt,arg...)
#endif

enum
{
   BUTTON_STATE_PRESSED      = 0x1,
   BUTTON_STATE_LONG_PRESSED = 0x2,
   BUTTON_STATE_FOCUSED      = 0X4,
}BUTTON_STATE;

typedef struct
{
    BUTTON Btn;
    u8  ucIdx;
    u8  ucState;
    u8  ucVisible;
    sys_timeo to;
}BUTTON_INNER_STRU;

static BUTTON_INNER_STRU sButtons[BUTTON_MAX_NUMBER];

static u8                sBtnMask;

BUTTON *BtnCreateButton(BUTTON *pBtn)
{
    u8 ucLoop;
    
    for (ucLoop = 0; ucLoop < BUTTON_MAX_NUMBER; ucLoop++)
    {
        if (!(sBtnMask & (1 << ucLoop)))
        {
            break;
        }
    }

    if (ucLoop >= BUTTON_MAX_NUMBER)
    {
        return NULL;
    }

    sBtnMask |= (1 << ucLoop);

    sButtons[ucLoop].Btn = *pBtn;

    sys_untimeout(&sButtons[ucLoop].to);

    return (BUTTON *)&sButtons[ucLoop];
}

void BtnDestoryButton(void * btn)
{
    BUTTON_INNER_STRU *pInner = (BUTTON_INNER_STRU *)btn;

    if (!(sBtnMask & (1 << pInner->ucIdx)))
    {
        return;
    }
    sBtnMask &= ~(1 << pInner->ucIdx);
    
    sys_untimeout(&sButtons[pInner->ucIdx].to);
}

u8   BtnHitTest(u16 usX,u16 usY, BUTTON_INNER_STRU *btn)
{
     if ((usX > btn->Btn.usLeft && usX < btn->Btn.usLeft + btn->Btn.usWidth)
        && (usY > btn->Btn.usTop && usY < btn->Btn.usTop + btn->Btn.usHeight))
    {
        return TRUE;
    }
     return FALSE;
}

static void BtnRestoreNormalState(BUTTON_INNER_STRU *btn)
{
    btn_dbg("%s\r\n",__FUNCTION__);
    
    btn->ucState = 0;

    switch(btn->Btn.ucBtnType)
    {
    case BUTTON_TYPE_BITMAP:
        DrawBitmap(btn->Btn.bmpNormal,btn->Btn.usLeft,btn->Btn.usTop,LCD_DrawPoint);
        break;
    default:
        break;
    }
}

static void BtnCheckTimer(void *para)
{
    BUTTON_INNER_STRU *btn = (BUTTON_INNER_STRU *)para;

    btn_dbg("%s\r\n",__FUNCTION__);
    
    btn->ucState |= (BUTTON_STATE_LONG_PRESSED);

}

static void BtnActiveState(BUTTON_INNER_STRU *btn)
{
    sys_timeout(2000,SYS_TIMER_ONE_SHOT,2000,BtnCheckTimer,btn,&btn->to);

    btn_dbg("%s\r\n",__FUNCTION__);

    switch(btn->Btn.ucBtnType)
    {
    case BUTTON_TYPE_BITMAP:
        DrawBitmap(btn->Btn.bmpActive,btn->Btn.usLeft,btn->Btn.usTop,LCD_DrawPoint);
        break;
    default:
        break;
    }
}


static void BtnTouchEvent(TOUCH_EVENT *event,BUTTON_INNER_STRU *btn)
{
    btn_dbg("%s : %d & %d\r\n",__FUNCTION__,btn->ucState,event->usEvent);

    if (!btn->ucVisible)
    {
        return ;
    }
    
    if (!btn->ucState
        && !event->usEvent)
    {
        return ;
    }

    if (!event->usEvent)
    {
    
        if (BUTTON_STATE_PRESSED  == (btn->ucState & (BUTTON_STATE_FOCUSED | BUTTON_STATE_PRESSED)))
        {
            BtnRestoreNormalState(btn);
            return ;
        }

        if (BUTTON_STATE_LONG_PRESSED  == (btn->ucState & (BUTTON_STATE_LONG_PRESSED)))
        {
            BtnRestoreNormalState(btn);
            return ;
        }

        if ((BUTTON_STATE_FOCUSED | BUTTON_STATE_PRESSED) == (btn->ucState & (BUTTON_STATE_FOCUSED | BUTTON_STATE_PRESSED)))
        {
            if (btn->Btn.cb4c) btn->Btn.cb4c(btn->Btn.tag);
            BtnRestoreNormalState(btn);
            return ;
        }

        return ;
    }

    if (!btn->ucState)
    {
        if (BtnHitTest(event->usX,event->usY,btn))
        {
            btn->ucState = (BUTTON_STATE_PRESSED | BUTTON_STATE_FOCUSED);
            
            BtnActiveState(btn);
        }
        return ;
    }

    if (btn->ucState & BUTTON_STATE_PRESSED)
    {
        if (!BtnHitTest(event->usX,event->usY,btn))
        {
            btn->ucState &= ~(BUTTON_STATE_FOCUSED);
            
            return ;
        }

        btn->ucState |= (BUTTON_STATE_FOCUSED);

        if (btn->ucState & BUTTON_STATE_LONG_PRESSED)
        {
            if (btn->Btn.cb4lc) btn->Btn.cb4lc(btn->Btn.tag);
        }
    }
    
}

void BtnShow(BUTTON *btn,BOOL bShow)
{
    BUTTON_INNER_STRU *pInner = (BUTTON_INNER_STRU *)btn;

    pInner->ucVisible = bShow;

    if (bShow)
    {
        switch(pInner->Btn.ucBtnType)
        {
        case BUTTON_TYPE_BITMAP:
            DrawBitmap(pInner->Btn.bmpNormal,pInner->Btn.usLeft,pInner->Btn.usTop,LCD_DrawPoint);
            break;
        default:
            break;
        }
    }
}

void BtnTouchEventEntry(TOUCH_EVENT *event)
{
    u8 ucLoop;

    BUTTON_INNER_STRU *pInner ;
    
    for (ucLoop = 0; ucLoop < BUTTON_MAX_NUMBER; ucLoop++)
    {
        pInner = &sButtons[ucLoop];    
        if (sBtnMask & (1 << pInner->ucIdx))
        {
            BtnTouchEvent(event,pInner);
        }
    }
     
}

void BtnInit(void)
{
    u8 ucLoop;
    
    for (ucLoop = 0; ucLoop < BUTTON_MAX_NUMBER; ucLoop++)
    {
        memset(&sButtons[ucLoop] ,0, sizeof (BUTTON_INNER_STRU));
        sButtons[ucLoop].ucIdx = ucLoop;
    }

    sBtnMask = 0;
}
