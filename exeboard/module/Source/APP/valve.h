#ifndef _APP_VALVE_H
#define _APP_VALVE_H

#define VALVE_MAGNETIC_NUM                 (8)
#define VALVE_GENERAL_PUMP_NUM             (2)
#define VALVE_PRESSURE_REGULATING_PUMP_NUM (2)
#define VALVE_RECTIFIER_NUM                (3)
#define VALVE_EDI_NUM                      (1)

#define VALVE_TOTAL_NUMBER                 (VALVE_MAGNETIC_NUM + VALVE_GENERAL_PUMP_NUM + VALVE_PRESSURE_REGULATING_PUMP_NUM + VALVE_RECTIFIER_NUM + VALVE_EDI_NUM)

typedef enum
{
    VALVE_TYPE_MAGNETIC              = 0,
    VALVE_TYPE_GENERAL_PUMP             ,
    VALVE_TYPE_PRESSURE_REGULATING_PUMP ,
    VALVE_TYPE_RECTIFIER                ,  /* actually not a gValve */
    VALVE_TYPE_EDI                      ,  /* actually not a gValve */
    VALVE_NUM,
}VALVE_TYPE_ENUM;

typedef struct
{
    uint8_t ucType;
    uint8_t ucPcaChl;
    uint8_t ucPcaSubChl;

    uint8_t ucPcaChl4IV;
    uint8_t ucPcaSubChl4IV;
    uint8_t ucPacMask4IV;
    
    uint8_t ucAdcChl4I;
    uint8_t ucAdcSubChl4I;
    uint8_t ucAdcData4IPtr; // Temporary data
    
    uint8_t ucAdcChl4V;
    uint8_t ucAdcSubChl4V;
    uint8_t ucAdcData4VPtr;  // Temporary data
    

}VALVE_NODE_STRU;

typedef struct
{
    VALVE_NODE_STRU aValves[VALVE_TOTAL_NUMBER];
     
}VALVE_STRU;

void ValveInit(void);

uint8_t ValveGetStatus(uint8_t ucChl);

uint16_t ValveGetAllStatus(void);

void ValveCtrl(uint8_t ucChl,uint8_t ucEnable);

extern VALVE_STRU gValve;

#endif
