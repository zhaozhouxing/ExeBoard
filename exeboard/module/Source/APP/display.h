#ifndef _DISPLAY_H_
#define _DISPLAY_H_

//#define TOC
//#define UF_FUNCTION


#include "stm32_eval.h"
#include "DtypeStm32.h"
#include "config.h"
#include "sys_time.h "

#include "cminterface.h"

#define INPUT_REG_LOOP_OFFSET (0)
#define INPUT_REG_LOOP_NUM    (3)

#define INPUT_REG_RECTIFIER_OFFSET (INPUT_REG_LOOP_OFFSET + INPUT_REG_LOOP_NUM)
#define INPUT_REG_RECTIFIER_NUM    (3)

#define INPUT_REG_EDI_OFFSET       (INPUT_REG_RECTIFIER_OFFSET + INPUT_REG_RECTIFIER_NUM)
#define INPUT_REG_EDI_NUM    (1)

#define INPUT_REG_PUMP_OFFSET      (INPUT_REG_EDI_OFFSET + INPUT_REG_EDI_NUM)
#define INPUT_REG_PUMP_NUM         (2)

#define INPUT_REG_REGPUMPI_OFFSET (INPUT_REG_PUMP_OFFSET + INPUT_REG_PUMP_NUM)
#define INPUT_REG_REGPUMPI_NUM    (2)

#define INPUT_REG_REGPUMPV_OFFSET (INPUT_REG_REGPUMPI_OFFSET + INPUT_REG_REGPUMPI_NUM)
#define INPUT_REG_REGPUMPV_NUM    (2)

#define INPUT_REG_QMI_OFFSET (INPUT_REG_REGPUMPV_OFFSET + INPUT_REG_REGPUMPV_NUM)
#define INPUT_REG_QMI_NUM    (5*3)


#define MAX_INPUT_REGISTERS (INPUT_REG_LOOP_NUM + INPUT_REG_RECTIFIER_NUM + INPUT_REG_RECTIFIER_NUM +INPUT_REG_PUMP_NUM + INPUT_REG_REGPUMPI_NUM + INPUT_REG_REGPUMPV_NUM + INPUT_REG_QMI_NUM)
#define MAX_HOLD_REGISTERS  ( 3)

#define MAX_ECO_NUMBER   (5)

typedef enum
{
  HOLD_REGS_NAME_RPT = 0,
  HOLD_REGS_NAME_PUMP1,  
  HOLD_REGS_NAME_PUMP2,  
}HOLD_REGS_NAME_ENUM;


typedef struct
{
    sys_timeo to4Adc; // timer for sensor

    uint16_t ausInputRegs[MAX_INPUT_REGISTERS];

    uint16_t ausHoldRegs[MAX_HOLD_REGISTERS];
}DISPLAY_STRU;

extern DISPLAY_STRU Display;

void Disp_Init(void);
void Disp_SecondTask(void);

void Disp_ModbusReadInputRegister(MODBUS_PACKET_COMM_STRU *pModReq,MODBUS_PACKET_COMM_STRU *pModRsp);
void Disp_ModbusReadInputStatus(MODBUS_PACKET_COMM_STRU *pModReq,MODBUS_PACKET_COMM_STRU *pModRsp);
void Disp_ModbusReadCoilStatus(MODBUS_PACKET_COMM_STRU *pModReq,MODBUS_PACKET_COMM_STRU *pModRsp);
void Disp_ModbusMaskWrite0XRegister(MODBUS_PACKET_COMM_STRU *pModReq,MODBUS_PACKET_COMM_STRU *pModRsp);
void Disp_ModbusMaskWrite4XRegister(MODBUS_PACKET_COMM_STRU *pModReq,MODBUS_PACKET_COMM_STRU *pModRsp);
void Disp_ModbusForceMultipleCoils(MODBUS_PACKET_COMM_STRU *pModReq,MODBUS_PACKET_COMM_STRU *pModRsp);
void Disp_ModbusForceSingleCoil(MODBUS_PACKET_COMM_STRU *pModReq,MODBUS_PACKET_COMM_STRU *pModRsp);
void Disp_ModbusPresetSingleRegister(MODBUS_PACKET_COMM_STRU *pModReq,MODBUS_PACKET_COMM_STRU *pModRsp);
void Disp_ModbusPresetMultipleRegister(MODBUS_PACKET_COMM_STRU *pModReq,MODBUS_PACKET_COMM_STRU *pModRsp);

#endif
