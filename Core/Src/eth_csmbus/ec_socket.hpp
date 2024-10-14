/*
 * ec_socket.hpp
 *
 *  Created on: Oct 12, 2023
 *      Author: sen
 */

#pragma once

#include "ec_socket_base.hpp"
#include "ec_led.h"
#include <string>
#include <functional>

namespace csmbus::socket
{

class AppSocket : private SocketBase
{
protected:
    ECPort_t _port = ECPort_1;

public:
    AppSocket(void) : SocketBase(){
    }

    void init(ECPort_t port, ECEther_appid_t appid)
    {
        _port = port;
        SocketBase::bind(ECTYPE_APP_S2M_PORT(appid), ECTYPE_APP_M2S_PORT(_port, appid));
    }

    void ether_send(ECReg_t reg, const void* data, size_t len)
    {
        uint8_t reg_type = ECTYPE_REG_2_REGTYPE(_port, reg);
        SocketBase::send(reg_type, data, len);
    }

    virtual void eth_callback(ECReg_t reg, const void* data, size_t len) = 0;

private:
    void callback(uint8_t reg_type, const void* data, size_t len) override
    {
        this->eth_callback((ECReg_t)ECTYPE_REGTYPE_2_REG(reg_type), data, len);
    }
};

}
