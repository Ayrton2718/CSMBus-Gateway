/*
 * robomas.h
 *
 *  Created on: Oct 12, 2023
 *      Author: sen
 */

#ifndef SRC_ROBOMAS_H_
#define SRC_ROBOMAS_H_

#include "eth_smbus.h"

#define ROBOMAS_CUR_LP_COEFF (0.1)

namespace smbus
{

class Robomas : public AppBase
{
private:
    typedef enum{
        Robomas_mode_DISABLE = 0,
        Robomas_mode_CURRENT = 1,
        Robomas_mode_RPM = 2,
        Robomas_mode_ANGLE = 3,
    } Robomas_mode_t;

    typedef struct{
        uint8_t mode;
        int16_t cur;
        int16_t rpm;
        int64_t ang;
    }__attribute__((__packed__)) Robomas_power_t;

    typedef struct{
        int16_t max_cur;
        int8_t direction;
        float rpm_p;
        float rpm_i;
        float rpm_d;
        float ang_p;
        float ang_i;
        float ang_d;
        int32_t pid_i_max;
    }__attribute__((__packed__)) Robomas_param_t;

    typedef struct
    {
        uint8_t is_received : 1;
        uint8_t is_connected : 1;
        uint8_t dummy : 6;
        int16_t rpm;
        int16_t cur;
        int16_t set_cur;
        int64_t ang; // [8191 / rota]
    }__attribute__((__packed__)) Robomas_sensor_t;

    typedef struct{
        Robomas_power_t power;
        Robomas_param_t param;

        int32_t rpm_u_i;
        int16_t rpm_u_befor;
        int32_t ang_u_i;
        int16_t ang_u_befor;

        Robomas_sensor_t sens;
        int16_t now_cur;
        float   filt_cur;
        uint16_t ang_befor;
        int32_t rota_count;
    } mot_t;

    timer::Timer    _tim;
    mot_t           _mot[6];

public:
    Robomas(ESPort_t port) : AppBase(port)
    {
    }

    void init(void)
    {
        for(size_t i = 0; i < 6; i++)
        {
            this->mot_reset(&_mot[i]);
        }
        _tim.reset();

        this->setup_callbacks(ESEther_appid_ROBOMAS);
    }

    void process(void)
    {
        if(900 < _tim.get_us())
        {
            _tim.reset();

            Robomas_sensor_t send_data[6];
            for(size_t i = 0; i < 6; i++)
            {
                _mot[i].filt_cur += ROBOMAS_CUR_LP_COEFF * (_mot[i].now_cur - _mot[i].filt_cur);
                _mot[i].sens.cur = (int16_t)_mot[i].filt_cur;

                send_data[i] = _mot[i].sens;
                _mot[i].sens.is_received = 0;

            }
            this->ether_send(ESReg_0, send_data, sizeof(Robomas_sensor_t) * 6);
        }
    }
    
    void eth_callback(ESReg_t reg, const void* data, size_t len)
    {
        if(ESReg_0 <= reg && len == sizeof(Robomas_power_t)*6)
        {
            for(size_t num = 0; num < 6; num++)
            {
                const Robomas_power_t* power_buff = static_cast<const Robomas_power_t*>(data);
                _mot[num].power = power_buff[num];
            }
        }

        if(ESReg_8 <= reg && reg <= ESReg_13 && len == sizeof(Robomas_param_t))
        {
            uint8_t num = reg - ESReg_8;
            _mot[num].param = *((const Robomas_param_t*)data);
        }
    }

    bool can_callback(uint16_t can_id, const uint8_t* data, size_t len)
    {
        uint8_t mot_id = (can_id & 0x00F) - 1; // 受信したIDから，ロボますモータのIDを求める．
        if((can_id & 0xFF0) == 0x200 && mot_id < 6)
        {
            mot_t* mot = &_mot[mot_id];
            int8_t direction = mot->param.direction;

            mot->sens.is_connected = 1;
            mot->sens.is_received = 1;

            uint16_t ang = ((int16_t)data[0] << 8) | data[1];
            mot->sens.rpm =(((int16_t)data[2] << 8) | data[3]) * direction;
            mot->now_cur = (((int16_t)data[4] << 8) | data[5]) * direction;

            if(mot->ang_befor < ang)
            {
                if(4096 < (ang - mot->ang_befor))
                {
                    mot->rota_count--;
                }
            }else{
                if(4096 < (mot->ang_befor - ang))
                {
                    mot->rota_count++;
                }
            }

            mot->sens.ang = ((mot->rota_count * 8191) + (int64_t)ang) * direction;
            mot->ang_befor = ang;
            return true;
        }
        return false;
    }

    void timer_callback(void)
    {
        ESType_bool_t is_safety_on = ESCtrl_isSafetyOn();

        uint8_t status_map[6] = {0, 0, 0, 0, 0, 0};
        int32_t set_cur[6];
        for(size_t i = 0; i < 6; i++)
        {
            mot_t* mot = &_mot[i];
            status_map[i] = mot->sens.is_connected;

            Robomas_mode_t mode = (Robomas_mode_t)mot->power.mode;
            if(is_safety_on)
            {
                mode = Robomas_mode_DISABLE;
            }

            switch(mode)
            {
            case Robomas_mode_CURRENT:
                set_cur[i] = mot->power.cur * mot->param.direction;
                mot->rpm_u_i = 0;
                mot->rpm_u_befor = 0;
                mot->ang_u_i = 0;
                mot->ang_u_befor = 0;
                break;
            
            case Robomas_mode_RPM:
                set_cur[i] = rpm_pid(mot);
                break;
            
            case Robomas_mode_ANGLE:
                set_cur[i] = ang_pid(mot);
                break;
            
            case Robomas_mode_DISABLE:
            default:
                set_cur[i] = 0;
                mot->rpm_u_i = 0;
                mot->rpm_u_befor = 0;
                mot->ang_u_i = 0;
                mot->ang_u_befor = 0;
                break;
            }

            if(mot->param.max_cur < set_cur[i])
            {
                set_cur[i] = mot->param.max_cur;
            }else if(set_cur[i] < -mot->param.max_cur){
                set_cur[i] = -mot->param.max_cur;
            }

            mot->sens.set_cur = set_cur[i];
        }

        uint8_t tx_data[8];

        if((status_map[0] + status_map[1] + status_map[2] + status_map[3]) != 0)
        {
            tx_data[0] = (uint8_t)(set_cur[0] >> 8);    tx_data[1] = (uint8_t)set_cur[0];
            tx_data[2] = (uint8_t)(set_cur[1] >> 8);    tx_data[3] = (uint8_t)set_cur[1];
            tx_data[4] = (uint8_t)(set_cur[2] >> 8);    tx_data[5] = (uint8_t)set_cur[2];
            tx_data[6] = (uint8_t)(set_cur[3] >> 8);    tx_data[7] = (uint8_t)set_cur[3];
            this->can_send(0x200, tx_data, 8);
        }

        if((status_map[4] + status_map[5]) != 0)
        {
            tx_data[0] = (uint8_t)(set_cur[4] >> 8);    tx_data[1] = (uint8_t)set_cur[4];
            tx_data[2] = (uint8_t)(set_cur[5] >> 8);    tx_data[3] = (uint8_t)set_cur[5];
            tx_data[4] = (uint8_t)(0 >> 8);             tx_data[5] = (uint8_t)0;
            tx_data[6] = (uint8_t)(0 >> 8);             tx_data[7] = (uint8_t)0;
            this->can_send(0x1FF, tx_data, 8);
        }
    }

    void reset_callback(void)
    {
        for(size_t i = 0; i < 6; i++)
        {
            mot_reset(&_mot[i]);
        }

        _tim.reset();
    }

private:
    int32_t rpm_pid(mot_t* mot)
    {
        int32_t rpm_u = mot->power.rpm - mot->sens.rpm;

        mot->rpm_u_i += rpm_u;
        if(!(-mot->param.pid_i_max < mot->rpm_u_i && mot->rpm_u_i < mot->param.pid_i_max))
        {
            mot->rpm_u_i -= rpm_u;
        }

        int32_t rpm_u_d = rpm_u - mot->rpm_u_befor;
        mot->rpm_u_befor = rpm_u;

        rpm_u = mot->param.rpm_p * rpm_u + mot->param.rpm_i * mot->rpm_u_i + mot->param.rpm_d * rpm_u_d;

        return (rpm_u + mot->power.cur) * mot->param.direction;
    }

    int32_t ang_pid(mot_t* mot)
    {
        int32_t ang_u = mot->power.ang - mot->sens.ang;

        mot->ang_u_i += ang_u;
        if(!(-mot->param.pid_i_max < mot->rpm_u_i && mot->rpm_u_i < mot->param.pid_i_max))
        {
            mot->ang_u_i -= ang_u;
        }

        int32_t ang_u_d = ang_u - mot->ang_u_befor;
        mot->ang_u_befor = ang_u;

        int32_t target_rpm = mot->param.ang_p * ang_u + mot->param.ang_i * mot->ang_u_i + mot->param.ang_d * ang_u_d;
        if(mot->power.rpm < target_rpm)
        {
            target_rpm = mot->power.rpm;
        }else if(target_rpm < -mot->power.rpm){
            target_rpm = -mot->power.rpm;
        }

        int32_t rpm_u = target_rpm - mot->sens.rpm;

        mot->rpm_u_i += rpm_u;
        if(!(-mot->param.pid_i_max < mot->rpm_u_i && mot->rpm_u_i < mot->param.pid_i_max))
        {
            mot->rpm_u_i -= rpm_u;
        }

        int32_t rpm_u_d = rpm_u - mot->rpm_u_befor;
        mot->rpm_u_befor = rpm_u;

        rpm_u = mot->param.rpm_p * rpm_u + mot->param.rpm_i * mot->rpm_u_i + mot->param.rpm_d * rpm_u_d;

        return (rpm_u + mot->power.cur) * mot->param.direction;
    }

    void mot_reset(mot_t* mot)
    {
        mot->power.mode = Robomas_mode_DISABLE;
        mot->power.cur = 0;
        mot->power.rpm = 0;
        mot->power.ang = 0;

        mot->param.max_cur = 10;
        mot->param.direction = 1;
        mot->param.rpm_p = 5;
        mot->param.rpm_i = 0.1;
        mot->param.rpm_d = 0.05;
        mot->param.ang_p = 2;
        mot->param.ang_i = 0;
        mot->param.ang_d = 0;
        mot->param.pid_i_max = 4000;

        mot->rpm_u_i = 0;
        mot->rpm_u_befor = 0;
        mot->ang_u_i = 0;
        mot->ang_u_befor = 0;

        mot->sens.is_connected = 0;
        mot->sens.is_received = 0;
        mot->sens.rpm = 0;
        mot->sens.cur = 0;
        mot->sens.set_cur = 0;
        mot->sens.ang = 0;

        mot->now_cur = 0;
        mot->filt_cur = 0;
        mot->ang_befor = 0;
        mot->rota_count = 0;
    }
};

}

#endif /* SRC_ROBOMAS_H_ */
