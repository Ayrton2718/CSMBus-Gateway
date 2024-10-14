/*
 * es_can.h
 *
 *  Created on: Oct 12, 2023
 *      Author: sen
 */

#ifndef SRC_ETH_CSMBUS_EC_CAN_H
#define SRC_ETH_CSMBUS_EC_CAN_H

#include "can.h"
#include "ec_type.h"

#define EC_CAN_ENABLE_RX1_BIND
#define EC_CAN_ENABLE_RX2_BIND

#define EC_CAN_BUFF_MAX_COUNT (8)


#ifdef __cplusplus
extern "C" {
#endif

void ECCan_init(void);

void ECCan_process(void);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include <array>

namespace csmbus::can
{

size_t can_bind(ECPort_t port, void* socket);
void can_send(ECPort_t port, uint16_t can_id, const uint8_t* data, size_t len);
void set_enable(ECPort_t port, size_t array_index, bool is_enable);

class CanSocket
{
public:
    CanSocket(void){
    }

    void bind(ECPort_t port)
    {
        _port = port;
        _array_index = csmbus::can::can_bind(_port, this);
    }

    void can_send(uint16_t can_id, const uint8_t* data, size_t len)
    {
        csmbus::can::can_send(_port, can_id, data, len);
    }

    void disable(void){
        csmbus::can::set_enable(_port, _array_index, false);
    }

    void enable(void){
        csmbus::can::set_enable(_port, _array_index, true);
    }

    virtual bool can_callback(uint16_t can_id, const uint8_t* data, size_t len) = 0;

protected:
    ECPort_t _port = ECPort_1;

private:
    CAN_HandleTypeDef* _hcan = NULL;
    size_t _array_index = ECTYPE_APP_MAX_COUNT - 1;
};

}

#endif

#endif /* SRC_ETH_CSMBUS_EC_TIMER_H */
