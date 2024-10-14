/*
 * ec_id.h
 *
 *  Created on: Oct 27, 2023
 *      Author: sen
 */

#ifndef SRC_CAN_CSMBUS_EC_ID_H
#define SRC_CAN_CSMBUS_EC_ID_H

#include "ec_type.h"

#ifdef __cplusplus
extern "C" {
#endif

#define EC_ID_FIXED_ADDR (ECId_1)

void ECId_init(void);

ECId_t ECId_getId(void);

void ECId_process(ECType_bool_t is_safety_on);

#ifdef __cplusplus
}
#endif


#endif /* SRC_CAN_CSMBUS_EC_ID_H */
