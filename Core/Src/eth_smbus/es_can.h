/*
 * es_can.h
 *
 *  Created on: Oct 12, 2023
 *      Author: sen
 */

#ifndef SRC_ETH_SMBUS_ES_CAN_H_
#define SRC_ETH_SMBUS_ES_CAN_H_

#include "can.h"
#include "es_type.h"

#define ES_CAN_ENABLE_RX1_BIND
#define ES_CAN_ENABLE_RX2_BIND

#define ES_CAN_BUFF_MAX_COUNT (8)


#ifdef __cplusplus
extern "C" {
#endif

void ESCan_init(void);

void ESCan_process(void);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include <array>

namespace smbus::can
{

size_t can_bind(ESPort_t port, void* socket);
void can_send(ESPort_t port, uint16_t can_id, const uint8_t* data, size_t len);
void set_enable(ESPort_t port, size_t array_index, bool is_enable);

class CanSocket
{
public:
    CanSocket(void){
    }

    void bind(ESPort_t port)
    {
        _port = port;
        _array_index = smbus::can::can_bind(_port, this);
    }

    void can_send(uint16_t can_id, const uint8_t* data, size_t len)
    {
        smbus::can::can_send(_port, can_id, data, len);
    }

    void disable(void){
        smbus::can::set_enable(_port, _array_index, false);
    }

    void enable(void){
        smbus::can::set_enable(_port, _array_index, true);
    }

    virtual bool can_callback(uint16_t can_id, const uint8_t* data, size_t len) = 0;

protected:
    ESPort_t _port = ESPort_1;

private:
    CAN_HandleTypeDef* _hcan = NULL;
    size_t _array_index = ES_APP_MAX_COUNT - 1;
};

}

#endif

#endif /* SRC_ETH_SMBUS_ES_TIMER_H_ */
