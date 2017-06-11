#ifndef _CM_INTERFACE_H_
#define _CM_INTERFACE_H_

#pragma   pack(1)

#define APP_PROTOL_CANID_RSVD           (0x0)
#define APP_PROTOL_CANID_ALLOC_BEGIN    (0X1)
#define APP_PROTOL_CANID_ALLOC_END      (511)
#define APP_PROTOL_CANID_DYNAMIC_BEGIN  (512)
#define APP_PROTOL_CANID_DYNAMIC_END    (1022)
#define APP_PROTOL_CANID_BROADCAST      (0x3ff)
#define APP_PROTOL_CANID_DYNAMIC_RANGE  (APP_PROTOL_CANID_DYNAMIC_END-APP_PROTOL_CANID_DYNAMIC_BEGIN)


#define APP_PROTOL_PACKET_LEN_POS      (0)
#define APP_PROTOL_PACKET_DEV_TYPE_POS (1)
#define APP_PROTOL_PACKET_MSG_TYPE_POS (2)
#define APP_PROTOL_PACKET_CONT_POS     (3)

#define APP_PROTOL_PACKET_HEADER       (3)

#define APP_PROTOL_PACKET_RSP_MASK     (0x80)

#define APP_HAND_SET_BEGIN_ADDRESS     (10)

#define APP_FM_FLOW_METER_NUM  (4)
#define APP_EXE_VALVE_NUM      (8)
#define APP_EXE_PRESSURE_METER (3)
#define APP_EXE_G_PUMP_NUM     (2)
#define APP_EXE_R_PUMP_NUM     (2)
#define APP_EXE_RECT_NUM       (3)
#define APP_EXE_EDI_NUM        (1)
#define APP_EXE_ECO_NUM        (5)

#define APP_EXE_E1_NO          (0)
#define APP_EXE_E2_NO          (1)
#define APP_EXE_E3_NO          (2)
#define APP_EXE_E4_NO          (3)
#define APP_EXE_E5_NO          (4)
#define APP_EXE_E6_NO          (5)
#define APP_EXE_E9_NO          (6)
#define APP_EXE_E10_NO         (7)

#define APP_EXE_C3_NO         (8)
#define APP_EXE_C4_NO         (9)
#define APP_EXE_C1_NO         (10)
#define APP_EXE_C2_NO         (11)


#define APP_FM_FM1_NO         (0)
#define APP_FM_FM2_NO         (1)
#define APP_FM_FM3_NO         (2)
#define APP_FM_FM4_NO         (3)
#define APP_FM_FM5_NO         (4)


typedef enum
{
    APP_OBJ_N_PUMP,/* Normal Pump */
    APP_OBJ_R_PUMP,/* Regulator Pump*/
    APP_OBJ_VALVE, /* Magnetic Valve */ 
    APP_OBJ_RECT,  /* Rectifer */
    APP_OBJ_B,     /* Pressure Meter */
    APP_OBJ_I,     /* Sensor for Water Qulality Messure */
    APP_OBJ_S,     /* Sensor for Water Volumn Messure */
    APP_OBJ_EDI,   /* EDI */
}APP_OBJ_TYPE_ENUM;

typedef enum
{
   APP_OBJ_VALUE_U32 = 0,
   APP_OBJ_VALUE_FLOAT ,
   APP_OBJ_VALUE_CUST ,
}APP_OBJ_VALUE_ENUM;

typedef enum
{
   APP_DEV_TYPE_HOST = 0,
   APP_DEV_TYPE_MAIN_CTRL ,
   APP_DEV_TYPE_EXE_BOARD,
   APP_DEV_TYPE_FLOW_METER,
   APP_DEV_TYPE_HAND_SET,
   APP_DEV_TYPE_BROADCAST = 0xff, 
}APP_DEV_TYPE_ENUM;

typedef enum
{
    APP_PAKCET_ADDRESS_HOST   = 0X00,
    APP_PAKCET_ADDRESS_MC     = 0X01 , 
    APP_PAKCET_ADDRESS_EXE    = 0X02 , 
    APP_PAKCET_ADDRESS_FM     = 0X03,
    APP_PAKCET_ADDRESS_HS     = 0X10,
    
}APP_PACKET_ADDRESS_ENUM;


typedef enum
{
    APP_PAKCET_COMM_ONLINE_NOTI   = 0X00,
    APP_PACKET_COMM_HEART_BEAT    = 0X01 , 
    APP_PACKET_COMM_HOST_RESET    = 0X02 , 
    APP_PACKET_CLIENT_REPORT      = 0X10,
    APP_PACKET_HAND_OPERATION     = 0X11,
    APP_PACKET_STATE_INDICATION   = 0X12,
    
}APP_PACKET_ENUM;



typedef enum
{
    APP_PACKET_RPT_FM = 0,    // FLOW METER
        
}APP_PACKET_RPT_FLOW_METER_TYPE;

typedef enum
{
    APP_PACKET_RPT_ECO =0 ,    //  Electrical pod coeffience
    APP_PACKET_MT_STATUS,      //  device state
        
}APP_PACKET_RPT_EXE_BOARD_TYPE;

typedef enum
{
    APP_PACKET_HO_CIR = 0,    //  CIRCULATION
    APP_PACKET_HO_QTW   ,    //  Taking quantified water
        
}APP_PACKET_HAND_OPERATION_TYPE;


typedef struct
{
    uint8_t ucLen;
    uint8_t ucTransId;
    uint8_t ucDevType;
    uint8_t ucMsgType;
}APP_PACKET_COMM_STRU;

typedef struct
{
    APP_PACKET_COMM_STRU hdr;
    uint16_t usDummy;
}APP_PACKET_ONLINE_NOTI_IND_STRU;


typedef struct
{
    APP_PACKET_COMM_STRU hdr;
    uint16_t usHeartBeatPeriod;
}APP_PACKET_ONLINE_NOTI_CONF_STRU;

typedef struct
{
    APP_PACKET_COMM_STRU hdr;
}APP_PACKET_HEART_BEAT_REQ_STRU;


typedef struct
{
    APP_PACKET_COMM_STRU hdr;
    uint8_t ucRptType;
    uint8_t aucData[1];
}APP_PACKET_CLIENT_RPT_IND_STRU;

typedef struct
{
     uint8_t ucId;
     uint32_t ulValue;
}APP_PACKET_RPT_FM_STRU; // flow meter


typedef struct
{
    APP_PACKET_COMM_STRU hdr;
    uint8_t ucOpsType;
    uint8_t aucData[1];
}APP_PACKET_HO_STRU;

typedef enum
{
    APP_PACKET_HO_ACTION_STOP = 0,    //  CIRCULATION
    APP_PACKET_HO_ACTION_START  ,    //  Taking quantified water
        
}APP_PACKET_HO_ACTION_TYPE;


typedef struct
{    
     uint8_t ucAction;
}APP_PACKET_HO_CIR_REQ_STRU; // flow meter

typedef struct
{    
     uint8_t ucAction;
     uint8_t ucResult;
}APP_PACKET_HO_CIR_RSP_STRU; // flow meter

typedef struct
{    
     uint8_t  ucAction;
     uint32_t ulVolumn;
}APP_PACKET_HO_QTW_REQ_STRU; // flow meter

typedef APP_PACKET_HO_CIR_RSP_STRU APP_PACKET_HO_QTW_RSP_STRU;

typedef struct
{
    unsigned short usTemp;
    float          fWaterQ; /* Water Quality */
}APP_ECO_VALUE_STRU;

typedef struct
{
     uint8_t  ucId;
     APP_ECO_VALUE_STRU ev;
}APP_PACKET_RPT_ECO_STRU; // flow meter

typedef struct
{
     uint8_t  ucId;
     uint8_t  ucType;  /* refer: APP_OBJ_TYPE_ENUM */
     uint8_t  ucState;
}APP_PACKET_STATE_STRU; // flow meter


typedef APP_PACKET_HEART_BEAT_REQ_STRU APP_PACKET_HEART_BEAT_RSP_STRU ;
typedef APP_PACKET_HEART_BEAT_REQ_STRU APP_PACKET_HOST_RESET_STRU ;
typedef APP_PACKET_HO_STRU             APP_PACKET_STATE_IND_STRU;


#define APP_PROTOL_HEADER_LEN (sizeof(APP_PACKET_COMM_STRU))

#define APP_POROTOL_PACKET_ONLINE_NOTI_IND_TOTAL_LENGTH (sizeof(APP_PACKET_ONLINE_NOTI_IND_STRU))

#define APP_POROTOL_PACKET_ONLINE_NOTI_IND_PAYLOAD_LENGTH (APP_POROTOL_PACKET_ONLINE_NOTI_IND_TOTAL_LENGTH - APP_PROTOL_HEADER_LEN)

#define APP_POROTOL_PACKET_ONLINE_NOTI_CNF_TOTAL_LENGTH (sizeof(APP_PACKET_ONLINE_NOTI_CONF_STRU))

#define APP_POROTOL_PACKET_ONLINE_NOTI_CNF_PAYLOAD_LENGTH (APP_POROTOL_PACKET_ONLINE_NOTI_CNF_TOTAL_LENGTH - APP_PROTOL_HEADER_LEN)

#define APP_POROTOL_PACKET_HEART_BEAT_RSP_TOTAL_LENGTH (sizeof(APP_PACKET_HEART_BEAT_RSP_STRU))

#define APP_POROTOL_PACKET_HEART_BEAT_RSP_PAYLOAD_LENGTH (APP_POROTOL_PACKET_HEART_BEAT_RSP_TOTAL_LENGTH - APP_PROTOL_HEADER_LEN)

#define APP_POROTOL_PACKET_HEART_BEAT_REQ_TOTAL_LENGTH (sizeof(APP_PACKET_HEART_BEAT_REQ_STRU))

#define APP_POROTOL_PACKET_HEART_BEAT_REQ_PAYLOAD_LENGTH (APP_POROTOL_PACKET_HEART_BEAT_REQ_TOTAL_LENGTH - APP_PROTOL_HEADER_LEN)

#define APP_POROTOL_PACKET_HOST_RESET_TOTAL_LENGTH (sizeof(APP_PACKET_HOST_RESET_STRU))

#define APP_POROTOL_PACKET_HOST_RESET_PAYLOAD_LENGTH (APP_POROTOL_PACKET_HOST_RESET_TOTAL_LENGTH - APP_PROTOL_HEADER_LEN)

#define APP_POROTOL_PACKET_CLIENT_RPT_IND_TOTAL_LENGTH (sizeof(APP_PACKET_CLIENT_RPT_IND_STRU) - 1)

#define APP_POROTOL_PACKET_CLIENT_RPT_IND_PAYLOAD_LENGTH (APP_POROTOL_PACKET_CLIENT_RPT_IND_TOTAL_LENGTH - APP_PROTOL_HEADER_LEN)

#define APP_POROTOL_PACKET_HO_COMMON_LENGTH (sizeof(APP_PACKET_HO_STRU) - 1)

typedef enum
{
    MODBUS_FUNC_CODE_Read_Coil_Status = 0X1,
    MODBUS_FUNC_CODE_Read_Input_Status = 0X2,
    MODBUS_FUNC_CODE_Read_Holding_Registers = 0X3,
    MODBUS_FUNC_CODE_Read_Input_Registers   = 0X4,
    MODBUS_FUNC_CODE_Force_Single_Coil = 0x5,
    MODBUS_FUNC_CODE_Preset_Single_Register = 0x6,
    MODBUS_FUNC_CODE_Force_Multiple_Coils = 0xF,
    MODBUS_FUNC_CODE_Preset_Multiple_Registers = 0x10,
    MODBUS_FUNC_CODE_Mask_Write_0X_Register    = 0x80,
    MODBUS_FUNC_CODE_Mask_Write_4X_Register    = 0x81,

}MODBUS_FUNC_CODE_ENUM;


typedef struct
{
    uint8_t ucFuncode;
    uint8_t aucData[1];
}MODBUS_PACKET_COMM_STRU;


#pragma pack()

#endif

