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

#include "Dica.h"

#include "task.h"

#include "config.h"


#include "ctype.h"

#include "beep.h"

#include "UartCmd.h"

#include "tsc2007.h"

#include "touch.h"

#include "sys_time.h "

extern struct i2c_adapter *gpI2cAdpater;

#define swab16(x) ((u16)(               \
    (((u16)(x) & (u16)0x00ffU) << 8) |          \
    (((u16)(x) & (u16)0xff00U) >> 8)))


#define TSC2007_MEASURE_TEMP0       (0x0 << 4)
#define TSC2007_MEASURE_AUX     (0x2 << 4)
#define TSC2007_MEASURE_TEMP1       (0x4 << 4)
#define TSC2007_ACTIVATE_XN     (0x8 << 4)
#define TSC2007_ACTIVATE_YN     (0x9 << 4)
#define TSC2007_ACTIVATE_YP_XN      (0xa << 4)
#define TSC2007_SETUP           (0xb << 4)
#define TSC2007_MEASURE_X       (0xc << 4)
#define TSC2007_MEASURE_Y       (0xd << 4)
#define TSC2007_MEASURE_Z1      (0xe << 4)
#define TSC2007_MEASURE_Z2      (0xf << 4)

#define TSC2007_POWER_OFF_IRQ_EN    (0x0 << 2)
#define TSC2007_ADC_ON_IRQ_DIS0     (0x1 << 2)
#define TSC2007_ADC_OFF_IRQ_EN      (0x2 << 2)
#define TSC2007_ADC_ON_IRQ_DIS1     (0x3 << 2)

#define TSC2007_12BIT           (0x0 << 1)
#define TSC2007_8BIT            (0x1 << 1)

#define MAX_12BIT           ((1 << 12) - 1)

#define ADC_ON_12BIT    (TSC2007_12BIT | TSC2007_ADC_ON_IRQ_DIS0)

#define READ_Y      (ADC_ON_12BIT | TSC2007_MEASURE_Y)
#define READ_Z1     (ADC_ON_12BIT | TSC2007_MEASURE_Z1)
#define READ_Z2     (ADC_ON_12BIT | TSC2007_MEASURE_Z2)
#define READ_X      (ADC_ON_12BIT | TSC2007_MEASURE_X)
#define PWRDOWN     (TSC2007_12BIT | TSC2007_POWER_OFF_IRQ_EN)


#define TSC2007_NIRQ_PIN (STM32F103_GPA(0)) 

#define TSC2007_TIMER_CHECK_PERIOD (50)

#ifdef TSC2007_DBG_INFO
#define dev_dbg(fmt,arg...) printf(fmt,## arg)
#else
#define dev_dbg(fmt,arg...)
#endif

// add for shzn
//#define SHZN_XY_SWAP

//#define SHZN_Y_HACK

//#define SHZN_X_HACK


struct ts_event {
    u16 x;
    u16 y;
    u16 z1, z2;
};

struct tsc2007 {
    char                phys[32];
    u16                 model;
    u16                 x_plate_ohms;
    u16                 max_rt;
    struct i2c_client   *client;

    sys_timeo           to4check; // timer for repeat check
    
    int (*get_pendown_state)(void);
    void    (*clear_penirq)(void);      

};

typedef struct
{
    MsgHead msgHead;
    void *proc;
    void *para;
}TSC2007_MSG;


#define TSC2007_MSG_LENGHT (sizeof(TSC2007_MSG)-sizeof(MsgHead))

typedef void (*tsc2007_msg_cb)(void *);


static struct tsc2007 sTsc2007;

static void tsc2007_soft_irq(void);
static void tsc2007_data_proc(struct tsc2007 *ts);

int TSC2007_get_pendown_state(void)
{
    return !stm32_gpio_get_value(TSC2007_NIRQ_PIN);
}

static TSC2007_msg_Handler(Message *pMsg)
{
    TSC2007_MSG *dmsg = (TSC2007_MSG *)pMsg;

    if (dmsg->proc)
    {
        ((tsc2007_msg_cb)dmsg->proc)(dmsg->para);
    }
}

UINT8 TSC2007_ItfProcess(Message *pMsg)
{
    switch(pMsg->msgHead.nMsgType)
    {
    case TSC2007_MESSAGE_IRQ:
        {
           // do something here
           tsc2007_soft_irq();
        }
        break;
    case TSC2007_MESSAGE_DELAY_CHECK:
        {
           // do something here
           TSC2007_msg_Handler(pMsg);
        }
        break;
    default:
        break;
    }

    return 0;
}

int TSC2007_sh(int event,int chl,void *para)
{
    Message *Msg;

    if (DICA_SENSOR_EVENT_FALLING == event)
    {
        Msg = MessageAlloc(PID_TSC2007,0);
        
        if (Msg)
        {
            Msg->msgHead.nMsgType = TSC2007_MESSAGE_IRQ;
            Msg->msgHead.AddData = event;
            MessageSend(Msg);
        }
    }
    return 0;
}


static int tsc2007_xfer(struct tsc2007 *tsc, u8 cmd)
{
    s32 data;
    u16 val;

    data = i2c_smbus_read_word_data(tsc->client, cmd);
    if (data < 0) {
        dev_dbg( "i2c io error: %d\r\n", data);
        return data;
    }

    /* The protocol and raw data format from i2c interface:
     * S Addr Wr [A] Comm [A] S Addr Rd [A] [DataLow] A [DataHigh] NA P
     * Where DataLow has [D11-D4], DataHigh has [D3-D0 << 4 | Dummy 4bit].
     */
    val = swab16(data) >> 4;

    //printf( "data: 0x%x, val: 0x%x\r\n", data, val);

    return val;
}

static void tsc2007_read_values(struct tsc2007 *tsc, struct ts_event *tc)
{
    /* y- still on; turn on only y+ (and ADC) */
    tc->y = tsc2007_xfer(tsc, READ_Y);

    /* turn y- off, x+ on, then leave in lowpower */
    tc->x = tsc2007_xfer(tsc, READ_X);

    /* turn y+ off, x- on; we'll use formula #1 */
    tc->z1 = tsc2007_xfer(tsc, READ_Z1);
    tc->z2 = tsc2007_xfer(tsc, READ_Z2);

    /* Prepare for next touch reading - power down ADC, enable PENIRQ */
    tsc2007_xfer(tsc, PWRDOWN);
}

static u32 tsc2007_calculate_pressure(struct tsc2007 *tsc, struct ts_event *tc)
{
    u32 rt = 0;

    /* range filtering */
    if (tc->x == MAX_12BIT)
        tc->x = 0;

    if ((tc->x && tc->z1)) {
        /* compute touch pressure resistance using equation #1 */
        rt = tc->z2 - tc->z1;
        rt *= tc->x;
        rt *= tsc->x_plate_ohms;
        rt /= tc->z1;
        rt = (rt + 2047) >> 12;
    }

    return rt;
}

static int tsc2007_is_pen_down(struct tsc2007 *ts)
{
    /*
     * NOTE: We can't rely on the pressure to determine the pen down
     * state, even though this controller has a pressure sensor.
     * The pressure value can fluctuate for quite a while after
     * lifting the pen and in some cases may not even settle at the
     * expected value.
     *
     * The only safe way to check for the pen up condition is in the
     * work function by reading the pen signal state (it's a GPIO
     * and IRQ). Unfortunately such callback is not always available,
     * in that case we assume that the pen is down and expect caller
     * to fall back on the pressure reading.
     */

    if (!ts->get_pendown_state)
        return 1;

    return ts->get_pendown_state();
}


static void tsc2007_stop(struct tsc2007 *ts)
{
}

static int tsc2007_open(struct tsc2007 *ts)
{
    int err;

    /* Prepare for touch readings - power down ADC and enable PENIRQ */
    err = tsc2007_xfer(ts, PWRDOWN);
    if (err < 0) {
        tsc2007_stop(ts);
        return err;
    }

    return 0;
}


void tsc2007_report(void *proc ,void *para)
{
   Message *Msg;
   Msg = MessageAlloc(PID_TSC2007,TSC2007_MSG_LENGHT);

   if (Msg)
   {
       TSC2007_MSG *dmsg = (TSC2007_MSG *)Msg;
       dmsg->msgHead.nMsgType = TSC2007_MESSAGE_DELAY_CHECK;
       dmsg->proc = proc;
       dmsg->para = para;
       MessageSend(Msg);
   }
}

static void tsc2007_delay_check_proc(void *para)
{
    struct tsc2007 *ts = (struct tsc2007 *)para;

    tsc2007_data_proc(ts);
}

void tsc2007_delay_check_cb(void *para)
{
     tsc2007_report(tsc2007_delay_check_proc,para);
}

static void tsc2007_data_proc(struct tsc2007 *ts)
{
    struct ts_event tc;
    TOUCH_EVENT te;
    u32 rt;

    //printf("tsc0 \r\n");
    if (tsc2007_is_pen_down(ts)) {

        //printf("tsc1 \r\n");

        /* pen is down, continue with the measurement */
        tsc2007_read_values(ts, &tc);

        rt = tsc2007_calculate_pressure(ts, &tc);

        if (rt == 0 && !ts->get_pendown_state) {
            /*
             * If pressure reported is 0 and we don't have
             * callback to check pendown state, we have to
             * assume that pen was lifted up.
             */
            return;
        }

        if (rt <= ts->max_rt) {

   #ifdef SHZN_Y_HACK   
          tc.y = MAX_12BIT - tc.y;
   #endif

   #ifdef SHZN_X_HACK
          tc.x = MAX_12BIT - tc.x;
   #endif



#ifdef SHZN_XY_SWAP     
        te.usX = tc.y;
        te.usY = tc.x;
        te.usEvent = 1;
        touch_report(&te);
#else
        te.usX = tc.x;
        te.usY = tc.y;
        te.usEvent = 1;
        touch_report(&te);
#endif
        dev_dbg("DOWN point(%4d,%4d), pressure (%4u)\r\n",
            te.usX, te.usY, rt);

        }
        else
        {
            /*
             * Sample found inconsistent by debouncing or pressure is
             * beyond the maximum. Don't report it to user space,
             * repeat at least once more the measurement.
             */
            // dev_dbg(&ts->client->dev, "ignored pressure %d\n", rt);
        }
        // add timer
        sys_timeout(TSC2007_TIMER_CHECK_PERIOD,SYS_TIMER_PERIOD,TSC2007_TIMER_CHECK_PERIOD,tsc2007_delay_check_cb,ts,&ts->to4check);
        
    }
    else
    {
        te.usX = 0;
        te.usY = 0;
        te.usEvent = 0;
        touch_report(&te);

        sys_untimeout(&ts->to4check);
        
    }
}

static void tsc2007_soft_irq(void)
{
    struct tsc2007 *ts = &sTsc2007;

    tsc2007_data_proc(ts);

    if (ts->clear_penirq)
        ts->clear_penirq();
}


static struct i2c_client sTsc2007_client;

/*!
 *  Radio Initialization.
 *
 *  @author Sz. Papp
 *
 *  @note
 *
 */
void TSC2007_Init(void)
{
    struct tsc2007 *ts = &sTsc2007; 

    sTsc2007_client.adapter = gpI2cAdpater;
    sTsc2007_client.addr    = 0x48;
    sTsc2007_client.flags   = 0;

    ts->client            = &sTsc2007_client;
    ts->model             = 2003;
    ts->x_plate_ohms      = 300;
    ts->max_rt            =  MAX_12BIT;
    ts->get_pendown_state = TSC2007_get_pendown_state;
    ts->clear_penirq      = NULL;

    stm32_gpio_cfgpin(TSC2007_NIRQ_PIN,MAKE_PIN_CFG(0,GPIO_Mode_IPU));

    stm32_gpio_cfg_irq(TSC2007_NIRQ_PIN,EXTI_Trigger_Rising_Falling);

    // install IRQ Handler
    InstallSensorHandler(DICA_SENSOR_EVENT_RISING,stm32_gpio_get_ext_line(TSC2007_NIRQ_PIN),FALSE,DICA_TYPE_PERIOD,TSC2007_sh,NULL);
    
    InstallSensorHandler(DICA_SENSOR_EVENT_FALLING,stm32_gpio_get_ext_line(TSC2007_NIRQ_PIN),FALSE,DICA_TYPE_PERIOD,TSC2007_sh,NULL);


    tsc2007_open(ts);
}

