/*
 * es_led.h
 *
 *  Created on: Oct 12, 2023
 *      Author: sen
 */

#ifndef SRC_ETH_CSMBUS_EC_LED_H_
#define SRC_ETH_CSMBUS_EC_LED_H_

#include "stddef.h"
#include "es_type.h"

#ifdef __cplusplus
extern "C" {
#endif

void ESLed_init(void);

void ESLed_ethTx(void);
void ESLed_ethRx(void);

void ESLed_canRx1(void);
void ESLed_canRx2(void);

void ESLed_err(void);
void ESLed_bus_err(void);

void ESLed_hungUp(void);

void ESLed_process(ESType_bool_t is_safety_on);

#ifdef __cplusplus
}
#endif

#endif /* SRC_ETH_CSMBUS_EC_LED_H_ */
