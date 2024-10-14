/*
 * es_can.h
 *
 *  Created on: Oct 12, 2023
 *      Author: sen
 */

#ifndef SRC_ETH_SMBUS_ES_BACKDOOR_H_
#define SRC_ETH_SMBUS_ES_BACKDOOR_H_

#include "can.h"
#include "es_backdoor_type.h"
#include "../app/can_smbus/cs_type.h"

#ifdef __cplusplus
extern "C" {
#endif

void ESBackdoor_init(void);

void ESBackdoor_setApp(ESPort_t port, ESEther_appid_t appid);

void ESBackdoor_enablePanel(ESPort_t port, CSId_t id);

void ESBackdoor_process(ESType_bool_t is_safety_on);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include <functional>
#include <string>

void ESBackdoor_log(ESBackdoor_msgLvl_t lvl, ESBackdoor_msgType_t type, std::string msg);

void ESBackdoor_ethStatusCode(ESBackdoor_eth_status_t err_code);
void ESBackdoor_canStatusCode(ESPort_t port, ESBackdoor_can_status_t err_code);

#define ES_ETH_RESET()          ESBackdoor_ethStatusCode(ESBackdoor_eth_status_RESETING)

#define ES_CAN_OVERFLOW(port)   ESBackdoor_canStatusCode(port, ESBackdoor_can_status_OVERFLOWED)
#define ES_CAN_SEND_FAIL(port)   ESBackdoor_canStatusCode(port, ESBackdoor_can_status_SEND_FAIL)

#define ES_INFO(port, msg)   ESBackdoor_log(ESBackdoor_msgLvl_INFO, (ESBackdoor_msgType_t)port, msg)
#define ES_ERR(port, msg)   ESBackdoor_log(ESBackdoor_msgLvl_ERR, (ESBackdoor_msgType_t)port, msg)

#define ES_SYS_INFO(msg)   ESBackdoor_log(ESBackdoor_msgLvl_INFO, ESBackdoor_msgType_SYSTEM, msg)
#define ES_SYS_ERR(msg)   ESBackdoor_log(ESBackdoor_msgLvl_ERR, ESBackdoor_msgType_SYSTEM, msg)

#endif /*__cplusplus*/

#endif /* SRC_ETH_SMBUS_ES_BACKDOOR_H_ */
