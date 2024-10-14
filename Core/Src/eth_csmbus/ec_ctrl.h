/*
 * ec_ctrl.h
 *
 *  Created on: Oct 12, 2023
 *      Author: sen
 */

#ifndef SRC_ETH_CSMBUS_EC_CTRL_H
#define SRC_ETH_CSMBUS_EC_CTRL_H

#include "ec_ctrl_type.h"

#ifdef __cplusplus
extern "C" {
#endif

void ECCtrl_init(void);

ECType_bool_t ECCtrl_isSafetyOn(void);

void ECCtrl_process(void);

#ifdef __cplusplus
}
#endif


#ifdef __cplusplus

namespace csmbus::ctrl
{

void ctrl_bind(ECPort_t port, void* ctrl_iface);

class CtrlInterface
{
public:
    CtrlInterface(void){
    }

    void bind(ECPort_t port)
    {
        csmbus::ctrl::ctrl_bind(port, this);
    }

    virtual void reset_callback(void) = 0;
};

}

#endif /*__cplusplus*/

#endif /* SRC_ETH_CSMBUS_EC_CTRL_H */
