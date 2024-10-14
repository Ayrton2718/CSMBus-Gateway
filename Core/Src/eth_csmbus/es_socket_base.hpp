#ifndef SRC_ETH_CSMBUS_EC_SOCKET_BASE_H_
#define SRC_ETH_CSMBUS_EC_SOCKET_BASE_H_

#ifdef __cplusplus

#include <string.h>
#include <lwip.h>
#include <udp.h>
#include <ip_addr.h>
#include "es_led.h"


namespace csmbus::socket
{

class SocketBase
{
public:
    SocketBase(void){}

    void bind(u16_t send_port, u16_t recv_port)
    {
        _send_pcb = udp_new();
        IP4_ADDR(&_send_addr, ECTYPE_MASTER_IP1, ECTYPE_MASTER_IP2, ECTYPE_MASTER_IP3, ECTYPE_MASTER_IP4);
        _send_is_lock = 0;
        _send_seq = 0;
        _send_port = send_port;
        
        _recv_pcb = udp_new();
        if(udp_bind(_recv_pcb, IP_ADDR_ANY, recv_port) != ERR_OK)
        {
            ESLed_err();
        }
        udp_recv(_recv_pcb, raw_callback, (void*)this);
    }

    void send(uint8_t reg_type, const void* data, size_t len)
    {
        struct pbuf* p = pbuf_alloc(PBUF_TRANSPORT, sizeof(ESEther_header_t) + len, PBUF_POOL);
        if(p != NULL)
        {
            ESEther_header_t* header = (ESEther_header_t*)p->payload;
            header->seq = ++_send_seq;
            header->ack = 0;
            header->reg_type = reg_type;
            memcpy(&(((uint8_t*)p->payload)[sizeof(ESEther_header_t)]), data, len);

            this->send_locking(p);
            pbuf_free(p);
        }else{
            ESLed_err();
        }
    }

    virtual void callback(uint8_t reg_type, const void* data, size_t len) = 0;

private:
    struct udp_pcb*     _recv_pcb = NULL;
    
    struct udp_pcb*     _send_pcb = NULL;
    ip4_addr_t          _send_addr;
    volatile uint8_t    _send_is_lock;
    volatile uint8_t    _send_seq;
    u16_t               _send_port;

    static void raw_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);

    void send_locking(struct pbuf* p)
    {
        if(_send_is_lock == 0 && _send_pcb != NULL)
        {
            _send_is_lock = 1;
            err_t err = udp_sendto(_send_pcb, p, &_send_addr, _send_port);
            _send_is_lock = 0;

            if(err == ERR_OK)
            {
                ESLed_ethTx();
            }else{
                ESLed_err();
            }
        }else{
            _send_is_lock = 0;
            ESLed_err();
        }
    }
};

}

#endif /*__cplusplus*/

#endif /*SRC_ETH_CSMBUS_EC_SOCKET_BASE_H_*/