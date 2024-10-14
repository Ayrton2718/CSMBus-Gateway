/*
 * cs_type.h
 *
 *  Created on: Oct 27, 2023
 *      Author: sen
 */

#ifndef SRC_CAN_CSMBUS_CC_TYPE_H
#define SRC_CAN_CSMBUS_CC_TYPE_H

#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef uint8_t 	CSType_bool_t;

#define CCTYPE_FALSE 	(0)
#define CCTYPE_TRUE 	(1)
#define CCTYPE_BOOL_NOT(x) ((~x) & 0x01)

#define CCTYPE_SAFETY_TIMEOUT 	(500) // ms

#define CCTYPE_IS_M2S_PACKET(can_id) (((uint16_t)can_id & 0b10000000000) == 0)
#define CCTYPE_IS_S2M_PACKET(can_id) (((uint16_t)can_id & 0b10000000000) != 0)
#define CCTYPE_IS_BRC_PACKET(can_id) (((uint16_t)can_id & 0b11111000000) == 0x000)

#define CCTYPE_MAKE_M2S_CAN_ID(id, reg) ((((uint16_t)id & 0b1111) << 6) | ((uint16_t)reg & 0b111111))
#define CCTYPE_MAKE_S2M_CAN_ID(id, reg) (0b10000000000 | CCTYPE_MAKE_M2S_CAN_ID(id, reg))

#define CCTYPE_GET_PACKET_ID(can_id)    ((CSId_t)(((uint16_t)can_id & 0b01111000000) >> 6))
#define CCTYPE_GET_PACKET_REG(can_id)   ((uint16_t)(((uint16_t)can_id & 0b00000111111)))

#define CCTYPE_GET_BRC_REG(reg)         ((CSType_brcReg_t)((uint16_t)reg & 0b011111))
#define CCTYPE_GET_USER_REG(reg)        ((CSReg_t)((uint16_t)reg & 0b011111))
#define CCTYPE_GET_SYS_REG(reg)         ((CSReg_t)((uint16_t)reg & 0b000111))

#define CCTYPE_IS_USER_REG(reg)         (((uint16_t)reg & 0b011000) != 0b011000)
#define CCTYPE_IS_SYS_REG(reg)          (((uint16_t)reg & 0b011000) == 0b011000)
#define CCTYPE_IS_WRITE_REG(reg)        (((uint16_t)reg & 0b100000) == 0)
#define CCTYPE_IS_ACK_REG(reg)          (((uint16_t)reg & 0b100000) != 0)

#define CCTYPE_MAKE_USER_REG(reg)       ((uint16_t)(reg & 0b011111))
#define CCTYPE_MAKE_SYS_REG(reg)        ((uint16_t)(reg | 0b011000))
#define CCTYPE_MAKE_WRITE_REG(reg)      ((uint16_t)(reg & 0b011111))
#define CCTYPE_MAKE_ACK_REG(reg)        ((uint16_t)(reg | 0b100000))

typedef struct{
    uint8_t checksum;
} CSType_ack_t;

typedef enum
{
    CSId_BRC = 0,
    CSId_1 = 1, CSId_2, CSId_3, CSId_4, CSId_5,  CSId_6, 
    CSId_7 = 9, CSId_8, CSId_9, CSId_10, CSId_11, CSId_12,
    CSId_UNKNOWN = 15
} CSId_t;

typedef enum{
    CSReg_0 = 0, CSReg_1, CSReg_2, CSReg_3, CSReg_4, CSReg_5, CSReg_6, CSReg_7, CSReg_8, CSReg_9, CSReg_10,
    CSReg_11, CSReg_12, CSReg_13, CSReg_14, CSReg_15, CSReg_16, CSReg_17, CSReg_18, CSReg_19, CSReg_20,
    CSReg_21, CSReg_22, CSReg_23, 
} CSReg_t;


typedef enum
{
	CSType_brcReg_Safety    = 0b000001,
	CSType_brcReg_Unsafe    = 0b000010,
	CSType_brcReg_Reset     = 0b000011
} CSType_brcReg_t;

typedef enum{
    CSReg_s2mSystem_APPID = 0,
    CSReg_s2mSystem_ERR = 1,
} CSReg_s2mSystem_t;

typedef enum{
    CSReg_m2sSystem_REQ_APPID = 0,
} CSReg_m2sSystem_t;


typedef enum
{
    CSType_appid_AMT212B=0,
    CSType_appid_SICK,
    CSType_appid_SWITCH,
    CSType_appid_COLOR,
    CSType_appid_AIR,
    CSType_appid_SERVO,
    CSType_appid_AMT102,
    CSType_appid_UNKNOWN=255
} CSType_appid_t;


uint16_t CSId_convertId2Num(CSId_t id);
CSId_t CSId_convertNum2Id(uint8_t id);

#ifdef __cplusplus
}
#endif

#endif /* SRC_CAN_CSMBUS_CC_TYPE_H */
