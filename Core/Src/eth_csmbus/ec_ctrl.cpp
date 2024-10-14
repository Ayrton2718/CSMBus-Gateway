/*
 * ec_ctrl.cpp
 *
 *  Created on: Oct 12, 2023
 *      Author: sen
 */

#include "ec_ctrl.h"

#include "ec_timer.h"
#include "ec_socket_base.hpp"
#include "ec_backdoor.h"

#define EC_CTRL_CB_MAX_COUNT (ECTYPE_APP_MAX_COUNT*2)


static uint32_t     g_safety_time;
static ECTimer_t    g_ping_tim;

static uint32_t     g_reset_seed;

static std::array<std::pair<bool, csmbus::ctrl::CtrlInterface*>, EC_CTRL_CB_MAX_COUNT> g_callback;


class CtrlSocket : public csmbus::socket::SocketBase
{
public:
    CtrlSocket() : csmbus::socket::SocketBase(){}

    void callback(uint8_t reg_type, const void* data, size_t len)
    {
        switch((ECCtrl_m2sRegType_t)reg_type)
        {
        case ECCtrl_m2sRegType_PING:{
            if(len == sizeof(ECCtrl_s2mPingPacket_t))
            {
                const ECCtrl_m2sPingPacket_t* ping = (const ECCtrl_m2sPingPacket_t*)data;
                if(ping->is_safety_on != 1)
                {
                    g_safety_time = HAL_GetTick() + 500;
                }else{
                    g_safety_time = 0;
                }
            }
            }break;
        
        case ECCtrl_m2sRegType_RESET:{
            if(len == sizeof(ECCtrl_m2sResetPacket_t))
            {
                const ECCtrl_m2sResetPacket_t* reset = (const ECCtrl_m2sResetPacket_t*)data;
                if(g_reset_seed == 0)
                {
                    g_reset_seed = reset->host_seed;
                }else{
                    if(g_reset_seed != reset->host_seed)
                    {
                        for(size_t i = 0; i < ECTYPE_APP_MAX_COUNT; i++)
                        {
                            if(g_callback[i].first)
                            {
                                g_callback[i].second->reset_callback();
                            }
                        }
                        g_reset_seed = reset->host_seed;
                        EC_ETH_RESET();
                    }
                }
            }
            }break;
        default:
            break;
        };
    }
};

void csmbus::ctrl::ctrl_bind(ECPort_t port, void* ctrl_iface)
{
    bool is_hit = false;
    for(size_t i = 0; i < EC_CTRL_CB_MAX_COUNT; i++)
    {
        if(g_callback[i].second == nullptr)
        {
            g_callback[i].second = static_cast<csmbus::ctrl::CtrlInterface*>(ctrl_iface);
            g_callback[i].first = true;
            is_hit = true;
            break;
        }
    }

    if(is_hit == false)
    {
        EC_ERR(port, "Too many app");
    }
}


static CtrlSocket g_sock;

void ECCtrl_init(void)
{
    g_safety_time = 0;
    ECTimer_timStart(&g_ping_tim);
    g_reset_seed = 0;

    for(size_t i = 0; i < EC_CTRL_CB_MAX_COUNT; i++)
    {
        g_callback[i].first = false;
        g_callback[i].second = nullptr;
    }

    g_sock.bind(ECTYPE_CTRL_S2M_PORT, ECTYPE_CTRL_M2S_PORT);
}

ECType_bool_t ECCtrl_isSafetyOn(void)
{
    if(HAL_GetTick() < g_safety_time)
    {
        return ECTYPE_FALSE;
    }else{
        return ECTYPE_TRUE;
    }
}

void ECCtrl_process(void)
{
    if(200 <= ECTimer_getMs(g_ping_tim))
	{
		ECTimer_timStart(&g_ping_tim);

        ECCtrl_s2mPingPacket_t packet;
        packet.is_active = 1;
        g_sock.send(ECCtrl_s2mRegType_PING, &packet, sizeof(ECCtrl_s2mPingPacket_t));
	}
}