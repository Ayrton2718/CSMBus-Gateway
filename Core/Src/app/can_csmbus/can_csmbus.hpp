/*
 * can_csmbus.h
 *
 *  Created on: Oct 12, 2023
 *      Author: sen
 */

#ifndef SRC_APP_CAN_CSMBUS_H_
#define SRC_APP_CAN_CSMBUS_H_

#include "eth_csmbus.h"
#include "cc_type.h"

#define CAN_CSMBUS_BUFFER_COUNT (16)

namespace csmbus
{

class CanSMBus : public AppBase
{
private:
    typedef struct{
        uint16_t can_id;
        uint8_t len;
        uint8_t data[8];
    }__attribute__((__packed__)) CanSMBus_packet_t;

    typedef struct{
        uint8_t             count;
        CanSMBus_packet_t   packet[4];
    }__attribute__((__packed__)) CanSMBus_m2s_t;

    typedef struct{
        uint8_t             count;
        CanSMBus_packet_t   packet[4];
    }__attribute__((__packed__)) CanSMBus_s2m_t;

    typedef struct{
        uint8_t wp;
        volatile uint8_t rp;
        volatile uint8_t count;
        CanSMBus_packet_t packet[CAN_CSMBUS_BUFFER_COUNT];
    } ring_buff_t;

    ring_buff_t     _s2m_buff;

    timer::Timer           _tim;

    ESType_bool_t   _befor_safety;
    timer::Timer       _safety_tim;

public:
    CanSMBus(ESPort_t port) : AppBase(port)
    {
    }

    void init(void)
    {
        _s2m_buff.wp = 0;
        _s2m_buff.rp = 0;
        _s2m_buff.count = 0;
        memset(_s2m_buff.packet, 0x00, sizeof(_s2m_buff.packet));

        _befor_safety = ESCtrl_isSafetyOn();

        _tim.start();
        _safety_tim.start();

        this->setup_callbacks(ESEther_appid_CANSMBUS);
    }

    

    void process(void)
    {
        if(500 < _tim.get_us())
        {
            _tim.reset();

            CanSMBus_s2m_t s2m;
            s2m.count = 0;

            ring_buff_t* rb = &_s2m_buff;
            size_t send_count = rb->count;
            for(size_t i = 0; (i < send_count) && (i < 4); i++)
            {
                s2m.packet[s2m.count] = rb->packet[rb->rp % CAN_CSMBUS_BUFFER_COUNT];
                s2m.count++;
                
                rb->rp++;
                rb->count--;
            }

            if(s2m.count != 0)
            {
                this->ether_send(ESReg_0, &s2m, sizeof(CanSMBus_s2m_t));
            }
        }


        ESType_bool_t is_safety_on = ESCtrl_isSafetyOn();
        if(200 < _safety_tim.get_ms() || _befor_safety != is_safety_on)
        {
            _safety_tim.reset();

            if(is_safety_on)
            {
                uint8_t buff[4];
                buff[3] = 'U'; buff[2] = 'N'; buff[1] = 'S'; buff[0] = 'F'; 
                this->can_send(CCTYPE_MAKE_M2S_CAN_ID(CSId_BRC, CSType_brcReg_Unsafe), buff, 4);
            }else{
                uint8_t buff[4];
                this->can_send(CCTYPE_MAKE_M2S_CAN_ID(CSId_BRC, CSType_brcReg_Safety), buff, 0);
            }
        }
        _befor_safety = is_safety_on;
    }

    void eth_callback(ESReg_t reg, const void* data, size_t len)
    {
        if(reg == ESReg_0 && len == sizeof(CanSMBus_m2s_t))
        {
            const CanSMBus_m2s_t* m2s = (const CanSMBus_m2s_t*)data;
            for(size_t i = 0; i < m2s->count && i < 4; i++)
            {
                this->can_send(m2s->packet[i].can_id, m2s->packet[i].data, m2s->packet[i].len);
            }
        }
    }

    bool can_callback(uint16_t can_id, const uint8_t* data, size_t len)
    {
        if(CCTYPE_IS_S2M_PACKET(can_id))
        {
            if(CSId_convertId2Num(CCTYPE_GET_PACKET_ID(can_id)) < CSId_convertId2Num(CSId_12))
            {
                ring_buff_t* rb = &_s2m_buff;
                if(rb->count == CAN_CSMBUS_BUFFER_COUNT)
                {
                    rb->rp++;
                    rb->count--;
                    ESLed_err();
                }

                rb->packet[rb->wp % CAN_CSMBUS_BUFFER_COUNT].can_id = can_id;
                rb->packet[rb->wp % CAN_CSMBUS_BUFFER_COUNT].len = len;
                memcpy(rb->packet[rb->wp % CAN_CSMBUS_BUFFER_COUNT].data, data, 8);
                rb->wp++;
                rb->count++;
                return true;
            }
        }
        
        return false;
    }

    void timer_callback(void){}

    void reset_callback(void)
    {
        uint8_t buff[4];
        buff[3] = 'R'; buff[2] = 'E'; buff[1] = 'S'; buff[0] = 'T';
        this->can_send(CCTYPE_MAKE_M2S_CAN_ID(CSId_BRC, CSType_brcReg_Reset), buff, 4);
        HAL_Delay(1);

        _tim.reset();
        _safety_tim.reset();
    }
};

}

#endif /* SRC_APP_CAN_CSMBUS_H_ */
