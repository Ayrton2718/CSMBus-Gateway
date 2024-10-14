/*
 * es_can.h
 *
 *  Created on: Oct 12, 2023
 *      Author: sen
 */

#ifndef SRC_ETH_SMBUS_ES_CTRL_H_
#define SRC_ETH_SMBUS_ES_CTRL_H_

#include "es_ctrl_type.h"

#ifdef __cplusplus
extern "C" {
#endif

void ESCtrl_init(void);

ESType_bool_t ESCtrl_isSafetyOn(void);

void ESCtrl_process(void);

#ifdef __cplusplus
}
#endif


#ifdef __cplusplus

namespace smbus::ctrl
{

void ctrl_bind(ESPort_t port, void* ctrl_iface);

class CtrlInterface
{
public:
    CtrlInterface(void){
    }

    void bind(ESPort_t port)
    {
        smbus::ctrl::ctrl_bind(port, this);
    }

    virtual void reset_callback(void) = 0;
};

}

#endif /*__cplusplus*/

#endif /* SRC_ETH_SMBUS_ES_CTRL_H_ */
