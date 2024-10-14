/*
 * ec_socket_base.hpp
 *
 *  Created on: Oct 12, 2023
 *      Author: sen
 */

#include "ec_socket_base.hpp"

namespace csmbus::socket
{

void SocketBase::raw_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
    SocketBase* obj = (SocketBase*)arg;
    if(sizeof(ESEther_header_t) <= p->len)
    {
        const uint8_t* recv_buffer = (const uint8_t*)p->payload;
        const ESEther_header_t* header = (const ESEther_header_t*)recv_buffer;

        uint8_t data_len =  p->len - sizeof(ESEther_header_t);
        const uint8_t* data = (uint8_t*)&recv_buffer[sizeof(ESEther_header_t)];

        if(header->ack)
        {
            struct pbuf* ack_p = pbuf_alloc(PBUF_TRANSPORT, sizeof(ESApp_ackPacket_t), PBUF_POOL);
            if(ack_p != NULL)
            {
                ESApp_ackPacket_t* ack_packet = (ESApp_ackPacket_t*)ack_p->payload;

                ack_packet->header = *header;
                ack_packet->checksum = ESType_ackChecksumCalculator(data, data_len);

                obj->send_locking(ack_p);
                pbuf_free(ack_p);
            }else{
                ESLed_err();
            }
        }

        obj->callback(header->reg_type, (const void*)data, data_len);
        ESLed_ethRx();
    }
    pbuf_free(p);
}

}