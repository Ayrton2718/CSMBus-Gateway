/*
 * ec_backdoor.cpp
 *
 *  Created on: Oct 12, 2023
 *      Author: sen
 */

#include "ec_backdoor.h"

#include "ec_socket_base.hpp"
#include "ec_timer.h"
#include "ec_can.h"
#include "../app/can_csmbus/cc_type.h"

#define EC_MSG_BUFF_COUNT   (4)

typedef struct{    
    volatile uint8_t            ping_wp;
    ECBackdoor_s2mPingPacket_t  ping_packet[2];
    ECTimer_t                   ping_tim;

    ECTimer_t    diagnostic_tim;

    ECTimer_t           log_tim;
    uint8_t             log_wp;
    volatile uint8_t    log_rp;
    volatile uint8_t    log_count;
    ECBackdoor_s2mMsgPacket_t   log_buff[EC_MSG_BUFF_COUNT];
} ECBackdoor_obj_t;

static ECBackdoor_obj_t 	g_obj;


class BackdoorCan : public csmbus::can::CanSocket
{
private:
    bool _is_enable_panel = false;
    uint16_t _panel_recv_id = 0x000;
    uint16_t _panel_send_id = 0x000;
    
    bool _panel_active = false;
    uint8_t _panel_packet[sizeof(ECBackdoor_s2mPingPacket_t::panel_packet)];

public:
    BackdoorCan() : csmbus::can::CanSocket(){}

    void enable_panel(CCId_t id){
        _panel_recv_id = CCTYPE_MAKE_S2M_CAN_ID(id, CCTYPE_MAKE_USER_REG(CCTYPE_MAKE_WRITE_REG(CCReg_0)));
        _panel_send_id = CCTYPE_MAKE_M2S_CAN_ID(id, CCTYPE_MAKE_USER_REG(CCTYPE_MAKE_WRITE_REG(CCReg_0)));
        _is_enable_panel = true;
    }

    bool get_panel(uint8_t* panel_packet){
        bool is_active = _panel_active;
        memcpy(panel_packet, _panel_packet, sizeof(_panel_packet));
        _panel_active = false;
        return is_active;
    }

    void send_panel(const ECBackdoor_m2sPanelPacket_t* packet){
        this->can_send(_panel_send_id, (const uint8_t*)packet, sizeof(ECBackdoor_m2sPanelPacket_t));
    }

    bool is_enable_panel(void){
        return _is_enable_panel;
    }

    bool can_callback(uint16_t can_id, const uint8_t* data, size_t len)
    {
        if(_panel_recv_id == can_id && len == sizeof(_panel_packet))
        {
            memcpy(_panel_packet, data, sizeof(_panel_packet));
            _panel_active = true;
            return true;
        }

        return false;
    }
};

class BackdoorEther : public csmbus::socket::SocketBase
{
private:
    bool _is_enable_panel = false;
    BackdoorCan* _can_sock = nullptr;

public:
    BackdoorEther() : csmbus::socket::SocketBase(){}

    void set_panel(BackdoorCan* can_sock){
        _can_sock = can_sock;
        _is_enable_panel = true;
    }

    void callback(uint8_t reg_type, const void* data, size_t len)
    {
        switch((ECBackdoor_m2sRegType_t)reg_type)
        {
        case ECBackdoor_m2sRegType_PANEL:
            if(_is_enable_panel && len == sizeof(ECBackdoor_m2sPanelPacket_t))
                _can_sock->send_panel((const ECBackdoor_m2sPanelPacket_t*)data);
            break;

        case ECBackdoor_m2sRegType_DIAGNOSTICS:
            break;
        
        default:
            break;
        };
    }
};


static BackdoorEther    g_ether_sock;
static BackdoorCan      g_can_sock[2];

void ECBackdoor_init(void)
{
    g_obj.ping_wp = 0;
    memset(g_obj.ping_packet, 0x00, sizeof(g_obj.ping_packet));
    ECTimer_timStart(&g_obj.ping_tim);

    ECTimer_timStart(&g_obj.diagnostic_tim);

    ECTimer_timStart(&g_obj.log_tim);
    g_obj.log_wp = 0;
    g_obj.log_rp = 0;
    g_obj.log_count = 0;
    memset(g_obj.log_buff, 0x00, sizeof(g_obj.log_buff));

    g_ether_sock.bind(ECTYPE_BACKDOOR_S2M_PORT, ECTYPE_BACKDOOR_M2S_PORT);

    g_can_sock[0].bind(ECPort_1);
    g_can_sock[0].disable();
    g_can_sock[1].bind(ECPort_2);
    g_can_sock[1].disable();
}

void ECBackdoor_setApp(ECPort_t port, ECEther_appid_t appid)
{
    g_obj.ping_packet[0].port_apps[port] |= (0x01 << appid);
    g_obj.ping_packet[1].port_apps[port] |= (0x01 << appid);
}

void ECBackdoor_enablePanel(ECPort_t port, CCId_t id)
{
    g_can_sock[port].enable_panel(id);
    g_can_sock[port].enable();
    g_ether_sock.set_panel(&g_can_sock[port]);
}

void ECBackdoor_process(ECType_bool_t is_safety_on)
{
    if(20 <= ECTimer_getMs(g_obj.ping_tim))
	{
		ECTimer_timStart(&g_obj.ping_tim);

        ECBackdoor_s2mPingPacket_t* packet = &g_obj.ping_packet[g_obj.ping_wp];
        g_obj.ping_wp = (g_obj.ping_wp + 1) % 2;

        if(g_can_sock[0].is_enable_panel()){
            packet->panel_active = g_can_sock[0].get_panel(packet->panel_packet);
        }else if(g_can_sock[1].is_enable_panel()){
            packet->panel_active = g_can_sock[1].get_panel(packet->panel_packet);
        }else{
            packet->panel_active = 0;
        }
        g_ether_sock.send(ECBackdoor_s2mRegType_PING, packet, sizeof(ECBackdoor_s2mPingPacket_t));
        packet->eth_status = 0;
        packet->can_status[0] = 0;
        packet->can_status[1] = 0;
	}

    if(5 <= ECTimer_getMs(g_obj.log_tim))
    {
        ECTimer_timStart(&g_obj.log_tim);
        if(0 < g_obj.log_count)
        {
            ECBackdoor_s2mMsgPacket_t* packet = &g_obj.log_buff[g_obj.log_rp % EC_MSG_BUFF_COUNT];
            g_ether_sock.send(ECBackdoor_s2mRegType_MSG, packet, sizeof(ECBackdoor_s2mMsgPacket_t));
            g_obj.log_rp++;
            g_obj.log_count--;
        }
    }
}


void ECBackdoor_log(ECBackdoor_msgLvl_t lvl, ECBackdoor_msgType_t type, std::string msg)
{
    if(ECTYPE_MSG_MAX_LEN < msg.size())
    {
        msg.resize(127, '\0');
    }

    if(EC_MSG_BUFF_COUNT == g_obj.log_count)
    {
        g_obj.log_rp++;
        g_obj.log_count--;
    }

    ECBackdoor_s2mMsgPacket_t* packet = &g_obj.log_buff[g_obj.log_wp % EC_MSG_BUFF_COUNT];
    packet->lvl = lvl;
    packet->type = type;
    memcpy(packet->msg, msg.data(), msg.size());
    packet->msg[ECTYPE_MSG_MAX_LEN] = '\0';

    g_obj.log_wp++;
    g_obj.log_count++;
}

void ECBackdoor_ethStatusCode(ECBackdoor_eth_status_t err_code)
{
    g_obj.ping_packet[g_obj.ping_wp].eth_status |= err_code;
    ECLed_err();
}

void ECBackdoor_canStatusCode(ECPort_t port, ECBackdoor_can_status_t err_code)
{
    g_obj.ping_packet[g_obj.ping_wp].can_status[port] |= err_code;
    ECLed_bus_err();
}
