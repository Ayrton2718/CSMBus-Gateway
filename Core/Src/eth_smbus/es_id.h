/*
 * es_id.h
 *
 *  Created on: Oct 27, 2023
 *      Author: sen
 */

#ifndef SRC_CAN_SMBUS_ES_ID_H_
#define SRC_CAN_SMBUS_ES_ID_H_

#include "es_type.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ES_ID_FIXED_ADDR (ESId_1)

void ESId_init(void);

ESId_t ESId_getId(void);

void ESId_process(ESType_bool_t is_safety_on);

#ifdef __cplusplus
}
#endif


#endif /* SRC_CAN_SMBUS_CS_ID_H_ */
