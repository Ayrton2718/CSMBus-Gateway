#ifndef SRC_ETH_CSMBUS_EC_SOCKET_H_
#define SRC_ETH_CSMBUS_EC_SOCKET_H_

#include "es_socket_base.hpp"
#include "es_led.h"


/*********C++ only*******/
#ifdef __cplusplus

#include <string>
#include <functional>

namespace csmbus::socket
{

class AppSocket : private SocketBase
{
protected:
    ESPort_t _port = ESPort_1;

public:
    AppSocket(void) : SocketBase(){
    }

    void init(ESPort_t port, ESEther_appid_t appid)
    {
        _port = port;
        SocketBase::bind(ECTYPE_APP_S2M_PORT(appid), ECTYPE_APP_M2S_PORT(_port, appid));
    }

    void ether_send(ESReg_t reg, const void* data, size_t len)
    {
        uint8_t reg_type = ECTYPE_REG_2_REGTYPE(_port, reg);
        SocketBase::send(reg_type, data, len);
    }

    virtual void eth_callback(ESReg_t reg, const void* data, size_t len) = 0;

private:
    void callback(uint8_t reg_type, const void* data, size_t len) override
    {
        this->eth_callback((ESReg_t)ECTYPE_REGTYPE_2_REG(reg_type), data, len);
    }
};

}
#endif /*__cplusplus*/


#endif /*SRC_ETH_CSMBUS_EC_SOCKET_H_*/
