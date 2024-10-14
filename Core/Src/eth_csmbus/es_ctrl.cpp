#include "es_ctrl.h"

#include "es_timer.h"
#include "es_socket_base.hpp"
#include "es_backdoor.hpp"

#define ES_CTRL_CB_MAX_COUNT (ES_APP_MAX_COUNT*2)


static uint32_t     g_safety_time;
static ESTimer_t    g_ping_tim;

static uint32_t     g_reset_seed;

static std::array<std::pair<bool, smbus::ctrl::CtrlInterface*>, ES_CTRL_CB_MAX_COUNT> g_callback;


class CtrlSocket : public smbus::socket::SocketBase
{
public:
    CtrlSocket() : smbus::socket::SocketBase(){}

    void callback(uint8_t reg_type, const void* data, size_t len)
    {
        switch((ESCtrl_m2sRegType_t)reg_type)
        {
        case ESCtrl_m2sRegType_PING:{
            if(len == sizeof(ESCtrl_s2mPingPacket_t))
            {
                const ESCtrl_m2sPingPacket_t* ping = (const ESCtrl_m2sPingPacket_t*)data;
                if(ping->is_safety_on != 1)
                {
                    g_safety_time = HAL_GetTick() + 500;
                }else{
                    g_safety_time = 0;
                }
            }
            }break;
        
        case ESCtrl_m2sRegType_RESET:{
            if(len == sizeof(ESCtrl_m2sResetPacket_t))
            {
                const ESCtrl_m2sResetPacket_t* reset = (const ESCtrl_m2sResetPacket_t*)data;
                if(g_reset_seed == 0)
                {
                    g_reset_seed = reset->host_seed;
                }else{
                    if(g_reset_seed != reset->host_seed)
                    {
                        for(size_t i = 0; i < ES_APP_MAX_COUNT; i++)
                        {
                            if(g_callback[i].first)
                            {
                                g_callback[i].second->reset_callback();
                            }
                        }
                        g_reset_seed = reset->host_seed;
                        ES_ETH_RESET();
                    }
                }
            }
            }break;
        default:
            break;
        };
    }
};

void smbus::ctrl::ctrl_bind(ESPort_t port, void* ctrl_iface)
{
    bool is_hit = false;
    for(size_t i = 0; i < ES_CTRL_CB_MAX_COUNT; i++)
    {
        if(g_callback[i].second == nullptr)
        {
            g_callback[i].second = static_cast<smbus::ctrl::CtrlInterface*>(ctrl_iface);
            g_callback[i].first = true;
            is_hit = true;
            break;
        }
    }

    if(is_hit == false)
    {
        ES_ERR(port, "Too many app");
    }
}


static CtrlSocket g_sock;

void ESCtrl_init(void)
{
    g_safety_time = 0;
    ESTimer_timStart(&g_ping_tim);
    g_reset_seed = 0;

    for(size_t i = 0; i < ES_CTRL_CB_MAX_COUNT; i++)
    {
        g_callback[i].first = false;
        g_callback[i].second = nullptr;
    }

    g_sock.bind(ESTYPE_CTRL_S2M_PORT, ESTYPE_CTRL_M2S_PORT);
}

ESType_bool_t ESCtrl_isSafetyOn(void)
{
    if(HAL_GetTick() < g_safety_time)
    {
        return ESTYPE_FALSE;
    }else{
        return ESTYPE_TRUE;
    }
}

void ESCtrl_process(void)
{
    if(200 <= ESTimer_getMs(g_ping_tim))
	{
		ESTimer_timStart(&g_ping_tim);

        ESCtrl_s2mPingPacket_t packet;
        packet.is_active = 1;
        g_sock.send(ESCtrl_s2mRegType_PING, &packet, sizeof(ESCtrl_s2mPingPacket_t));
	}
}