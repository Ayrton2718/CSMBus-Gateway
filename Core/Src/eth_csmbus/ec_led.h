/*
 * ec_led.h
 *
 *  Created on: Oct 12, 2023
 *      Author: sen
 */

#ifndef SRC_ETH_CSMBUS_EC_LED_H
#define SRC_ETH_CSMBUS_EC_LED_H

#include "stddef.h"
#include "ec_type.h"

#ifdef __cplusplus
extern "C" {
#endif

void ECLed_init(void);

void ECLed_ethTx(void);
void ECLed_ethRx(void);

void ECLed_canRx1(void);
void ECLed_canRx2(void);

void ECLed_err(void);
void ECLed_bus_err(void);

void ECLed_hungUp(void);

void ECLed_process(ECType_bool_t is_safety_on);

#ifdef __cplusplus
}
#endif

#endif /* SRC_ETH_CSMBUS_EC_LED_H */
