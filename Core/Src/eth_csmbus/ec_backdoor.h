/*
 * es_backdoor.h
 *
 *  Created on: Oct 12, 2023
 *      Author: sen
 */

#ifndef SRC_ETH_CSMBUS_EC_BACKDOOR_H
#define SRC_ETH_CSMBUS_EC_BACKDOOR_H

#include "can.h"
#include "ec_backdoor_type.h"
#include "../app/can_csmbus/cc_type.h"

#ifdef __cplusplus
extern "C" {
#endif

void ECBackdoor_init(void);

void ECBackdoor_setApp(ECPort_t port, ECEther_appid_t appid);

void ECBackdoor_enablePanel(ECPort_t port, CCId_t id);

void ECBackdoor_process(ECType_bool_t is_safety_on);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include <functional>
#include <string>

void ECBackdoor_log(ECBackdoor_msgLvl_t lvl, ECBackdoor_msgType_t type, std::string msg);

void ECBackdoor_ethStatusCode(ECBackdoor_eth_status_t err_code);
void ECBackdoor_canStatusCode(ECPort_t port, ECBackdoor_can_status_t err_code);

#define EC_ETH_RESET()          ECBackdoor_ethStatusCode(ECBackdoor_eth_status_RESETING)

#define EC_CAN_OVERFLOW(port)   ECBackdoor_canStatusCode(port, ECBackdoor_can_status_OVERFLOWED)
#define EC_CAN_SEND_FAIL(port)   ECBackdoor_canStatusCode(port, ECBackdoor_can_status_SEND_FAIL)

#define EC_INFO(port, msg)   ECBackdoor_log(ECBackdoor_msgLvl_INFO, (ECBackdoor_msgType_t)port, msg)
#define EC_ERR(port, msg)   ECBackdoor_log(ECBackdoor_msgLvl_ERR, (ECBackdoor_msgType_t)port, msg)

#define EC_SYS_INFO(msg)   ECBackdoor_log(ECBackdoor_msgLvl_INFO, ECBackdoor_msgType_SYSTEM, msg)
#define EC_SYS_ERR(msg)   ECBackdoor_log(ECBackdoor_msgLvl_ERR, ECBackdoor_msgType_SYSTEM, msg)

#endif /*__cplusplus*/

#endif /* SRC_ETH_CSMBUS_EC_BACKDOOR_H */
