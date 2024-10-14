/*
 * ec_backdoor_type.h
 *
 *  Created on: Oct 12, 2023
 *      Author: sen
 */

#ifndef SRC_ETH_CSMBUS_EC_BACKDOOR_TYPE_H
#define SRC_ETH_CSMBUS_EC_BACKDOOR_TYPE_H

#include "ec_type.h"

#define ECTYPE_MSG_MAX_LEN  (126)
#define ECTYPE_CAN_HZ_LEN   (12)

#ifdef __cplusplus
extern "C" {
#endif

typedef struct{
    uint8_t     port_apps[2];
    uint8_t     eth_status;
    uint8_t     can_status[2];
    uint8_t     panel_active;                
    uint8_t     panel_packet[2]; // data1, checksum
}__attribute__((__packed__)) ECBackdoor_s2mPingPacket_t;

typedef struct{
    uint8_t     lvl  :   4;
    uint8_t     type :   4;
    uint8_t     msg[ECTYPE_MSG_MAX_LEN + 1];
}__attribute__((__packed__)) ECBackdoor_s2mMsgPacket_t;

// enableのときは100msごとに送る
typedef struct{
    uint16_t        can1_ids[ECTYPE_CAN_HZ_LEN];
    uint8_t         can1_count[ECTYPE_CAN_HZ_LEN];
    uint16_t        can2_ids[ECTYPE_CAN_HZ_LEN];
    uint8_t         can2_count[ECTYPE_CAN_HZ_LEN];
}__attribute__((__packed__)) ECBackdoor_s2mDiagnosticsPacket_t;

typedef struct{
    uint8_t             can_packet[2]; // data1, data2, checksum
}__attribute__((__packed__)) ECBackdoor_m2sPanelPacket_t;

typedef struct{
    uint8_t             is_enable;
}__attribute__((__packed__)) ECBackdoor_m2sDiagnosticsPacket_t;


typedef enum{
    ECBackdoor_s2mRegType_PING = 0,
    ECBackdoor_s2mRegType_MSG = 2,
    ECBackdoor_s2mRegType_DIAGNOSTICS = 3,
} ECBackdoor_s2mRegType_t;

typedef enum{
    ECBackdoor_m2sRegType_PANEL = 0,
    ECBackdoor_m2sRegType_DIAGNOSTICS = 1,
} ECBackdoor_m2sRegType_t;

typedef enum{
    ECBackdoor_msgLvl_ERR   = 0,
    ECBackdoor_msgLvl_INFO  = 2
} ECBackdoor_msgLvl_t;

typedef enum{
    ECBackdoor_msgType_PORT1    = 0,
    ECBackdoor_msgType_PORT2    = 1,
    ECBackdoor_msgType_SYSTEM   = 2
} ECBackdoor_msgType_t;


enum ECBackdoor_eth_status_t {
    ECBackdoor_eth_status_SAFETY    = 0x01 << 0,
    ECBackdoor_eth_status_RESETING  = 0x01 << 1
};

typedef enum {
    ECBackdoor_can_status_OVERFLOWED    = 0x01 << 0,
    ECBackdoor_can_status_SEND_FAIL     = 0x01 << 1
} ECBackdoor_can_status_t;

#ifdef __cplusplus
}
#endif

#endif /* SRC_ETH_CSMBUS_EC_BACKDOOR_TYPE_H*/
