/*
 * odrive.h
 *
 *  Created on: Oct 12, 2023
 *      Author: sen
 */

#pragma once

#include "eth_csmbus.h"

namespace csmbus
{

class Odrive : public AppBase
{
private:
    typedef enum{
        Odrive_mode_DISABLE = 0,
        Odrive_mode_TORQUE = 1,
        Odrive_mode_VELOCITY = 2,
    } Odrive_mode_t;

    enum class control_mode_t : uint32_t {
        VOLTAGE_CONTROL,
        TORQUE_CONTROL,
        VELOCITY_CONTROL,
        POSITION_CONTROL
    };

    enum class input_mode_t : uint32_t {
        INACTIVE = 0x00,    //入力無効
        PASSTHROUGH, //直接制御 : 有効な入力：input_pos, input_vel 有効なコントロールモード：すべて
        VEL_RAMP, //傾斜速度制御  : 有効な入力：input_vel 有効なコントロールモード CONTROL_MODE_VELOCITY_CONTROL
        POS_FILTER,   // 2次位置フィルタ位置制御 : 有効な入力:input_pos
                    // 有効なコントロールモード:CONTROL_MODE_POSITION_CONTROL
        MIX_CHANNELS, //未実装
        TRAP_TRAJ,    //台形軌道位置制御
        TORQUE_RAMP,  //トルクの傾斜制御
        MIRROR       //他の軸のミラーをする
    };

    enum class cmd_t : uint32_t{
        s2m_get_version = 0x000,
        s2m_heart_beat = 0x001,
        m2s_estop = 0x002,
        s2m_get_error = 0x003,
        m2s_rx_sdo = 0x004,
        s2m_tx_sdo = 0x005,
        m2s_set_axis_node_id = 0x006,
        m2s_set_axis_state = 0x007,
        s2m_get_encoder_estimate = 0x009,
        m2s_controller_modes = 0x00B,
        m2s_set_input_pos = 0x00C,
        m2s_set_input_vel = 0x00D,
        m2s_set_input_torque = 0x00E,
        m2s_set_limits = 0x00F,
        m2s_set_traj_vel_limit = 0x011,
        m2s_set_traj_accel_limit = 0x012,
        m2s_set_traj_inertia = 0x013,
        s2m_get_iq = 0x014,
        s2m_get_temperature = 0x015,
        m2s_reboot = 0x016,
        s2m_get_bus_voltage_current = 0x017,
        m2s_clear_errors = 0x018,
        m2s_set_absolute_position = 0x019,
        m2s_set_pos_gain = 0x01A,
        m2s_set_vel_gains = 0x01B,
        s2m_get_torques = 0x01C,
        m2s_enter_dfu_module = 0x01f
    };

    enum class axis_err_t : uint32_t {
        INITIALIZING = 0x1,
        SYSTEM_LEVEL = 0x2,
        TIMING_ERROR = 0x4,
        MISSING_ESTIMATE = 0x8,
        BAD_CONFIG = 0x10,
        DRV_FAULT = 0x20,
        MISSING_INPUT = 0x40,
        DC_BUS_OVER_VOLTAGE = 0x100,
        DC_BUS_UNDER_VOLTAGE = 0x200,
        DC_BUS_OVER_CURRENT = 0x400,
        DC_BUS_OVER_REGEN_CURRENT = 0x800,
        CURRENT_LIMIT_VIOLATION = 0x1000,
        MOTOR_OVER_TEMP = 0x2000,
        INVERTER_OVER_TEMP = 0x4000,
        VELOCITY_LIMIT_VIOLATION = 0x8000,
        POSITION_LIMIT_VIOLATION = 0x10000,
        WATCHDOG_TIMER_EXPIRED = 0x1000000,
        ESTOP_REQUESTED = 0x2000000,
        SPINOUT_DETECTED = 0x4000000,
        BRAKE_RESISTOR_DISARMED = 0x8000000,
        THERMISTOR_DISCONNECTED = 0x10000000,
        CALIBRATION_ERROR = 0x40000000
    };

    enum class axis_state_t : uint8_t {
        UNDEFINED = 0x0,
        IDLE = 0x1,
        STARTUP_SEQUENCE = 0x2,
        FULL_CALIBRATION_SEQUENCE = 0x3,
        MOTOR_CALIBRATION = 0x4,
        SENSORLESS_CONTROL = 0x5,
        ENCODER_INDEX_SEARCH = 0x6,
        ENCODER_OFFSET_CALIBRATION = 0x7,
        CLOSED_LOOP_CONTROL = 0x8,
        LOCKIN_SPIN = 0x9,
        ENCODER_DIR_FIND = 0xA,
        HOMING = 0xB,
        ENCODER_HALL_POLARITY_CALIBRATION = 0xC,
        ENCODER_HALL_PHASE_CALIBRATION = 0xD
    };

private:
    typedef struct{
        uint8_t mode;
        float torque;
        float velocity;
    }__attribute__((__packed__)) Odrive_power_t;

    typedef struct{
        float vel_p;
        float vel_i;
    }__attribute__((__packed__)) Odrive_param_t;

    typedef struct{
        uint8_t is_connected :1;
        uint8_t heartbeat_received : 1;
        uint8_t error_received : 1;
        uint8_t encoder_estimate_received : 1;
        uint8_t torques_received : 1;
        uint8_t dummy : 3;
        uint32_t axis_error;
        uint8_t axis_state;
        uint8_t procedure_result;
        uint32_t active_errors;
        uint32_t disarm_reason;
        float pos_estimate;
        float vel_estimate;
        float torque_target;
        float torque_estimate;
    }__attribute__((__packed__)) Odrive_sensor_t;

    typedef struct{
        Odrive_power_t power;
        Odrive_param_t param;

        Odrive_sensor_t sens;
        timer::Timer    timeout;
    } mot_t;

    uint16_t        _base_id;

    timer::Timer    _sens_tim;
    timer::Timer    _checker_tim;
    timer::Timer    _output_tim;

    mot_t           _mot[4];

public:
    Odrive(ECPort_t port, uint16_t base_id=3<<3) : AppBase(port), _base_id(base_id)
    {
    }


    void init(void)
    {
        for(size_t i = 0; i < 4; i++)
        {
            mot_reset(&_mot[i]);
        }

        _sens_tim.reset();
        _checker_tim.reset();
        _output_tim.reset();
        this->setup_callbacks(ECEther_appid_ODRIVE);
    }

    void process(void)
    {
        if(900 < _sens_tim.get_us())
        {
            _sens_tim.reset();

            Odrive_sensor_t send_data[4];
            for(size_t num = 0; num < 4; num++)
            {
                send_data[num] = _mot[num].sens;
                _mot[num].sens.heartbeat_received = 0;
                _mot[num].sens.error_received = 0;
                _mot[num].sens.encoder_estimate_received = 0;
                _mot[num].sens.torques_received = 0;
            }
            this->ether_send(ECReg_0, send_data, sizeof(Odrive_sensor_t) * 4);
        }

        if(30 < _checker_tim.get_ms())
        {
            _checker_tim.reset();

            for(size_t num = 0; num < 4; num++)
            {
                if(_mot[num].sens.is_connected)
                {
                    if( (_mot[num].sens.axis_state != (uint8_t)axis_state_t::CLOSED_LOOP_CONTROL) &&
                        (_mot[num].sens.axis_state != (uint8_t)axis_state_t::ENCODER_INDEX_SEARCH) &&
                        ((_mot[num].sens.axis_error & (uint32_t)axis_err_t::INITIALIZING) == 0))
                    {
                        uint32_t tx_data = (uint32_t)axis_state_t::CLOSED_LOOP_CONTROL;
                        this->can_send(get_id(num, cmd_t::m2s_set_axis_state), (const uint8_t*)&tx_data, 4);
                    }
                }

                if(500 < _mot[num].timeout.get_ms()){
                    _mot[num].sens.is_connected = 0;
                }
            }
        }

        if(10 < _output_tim.get_ms())
        {
            _output_tim.reset();
            for(size_t num = 0; num < 4; num++)
            {
                if(_mot[num].sens.is_connected)
                {
                    set_output(num);
                }
            }
        }
    }

    void eth_callback(ECReg_t reg, const void* data, size_t len)
    {
        if(ECReg_0 <= reg && len == sizeof(Odrive_power_t)*4)
        {
            for(size_t num = 0; num < 4; num++)
            {
                const Odrive_power_t* power_buff = static_cast<const Odrive_power_t*>(data);
                _mot[num].power = power_buff[num];
                _output_tim.reset();
                set_output(num);
            }
        }

        if(ECReg_8 <= reg && reg <= ECReg_11 && len == sizeof(Odrive_param_t))
        {
            uint8_t num = reg - ECReg_8;
            _mot[num].param = *((const Odrive_param_t*)data);
        }
    }

    bool can_callback(uint16_t can_id, const uint8_t* data, size_t len)
    {
        uint8_t id = (uint8_t)((can_id >> 5) & (~_base_id));
        cmd_t cmd = (cmd_t)(can_id & 0b00011111);

        if(id < 4)
        {
            Odrive_sensor_t* sens = &_mot[id].sens;

            switch((uint32_t)cmd)
            {
            case (uint32_t)cmd_t::s2m_heart_beat:{
                if(len == 8)
                {
                    sens->is_connected = 1;
                    sens->heartbeat_received = 1;
                    memcpy(&sens->active_errors, &data[0], sizeof(uint32_t));
                    memcpy(&sens->axis_state, &data[4], sizeof(uint8_t));
                    memcpy(&sens->procedure_result, &data[5], sizeof(uint8_t));
                    _mot[id].timeout.reset();
                    return true;
                }
                }break;

            case (uint32_t)cmd_t::s2m_get_error:{
                if(len == 8)
                {
                    sens->error_received = 1;
                    memcpy(&sens->axis_error, &data[0], sizeof(uint32_t));
                    memcpy(&sens->disarm_reason, &data[4], sizeof(uint32_t));
                    return true;
                }
                }break;

            case (uint32_t)cmd_t::s2m_get_encoder_estimate:{
                if(len == 8)
                {
                    sens->encoder_estimate_received = 1;
                    memcpy(&sens->pos_estimate, &data[0], sizeof(float));
                    memcpy(&sens->vel_estimate, &data[4], sizeof(float));
                    return true;
                }
                }break;

            case (uint32_t)cmd_t::s2m_get_torques:{
                if(len == 8)
                {
                    sens->torques_received = 1;
                    memcpy(&sens->torque_target, &data[0], sizeof(float));
                    memcpy(&sens->torque_estimate, &data[4], sizeof(float));
                    return true;
                }
                }break;
            }
        }
        return false;
    }

    void timer_callback(void)
    {
    }

    void reset_callback(void)
    {
        for(size_t i = 0; i < 4; i++)
        {
            mot_reset(&_mot[i]);
        }
        
        _sens_tim.reset();
        _checker_tim.reset();
        _output_tim.reset();
    }

private:
    void mot_reset(mot_t* mot)
    {
        mot->power.mode = Odrive_mode_DISABLE;
        mot->power.torque = 0;
        mot->power.velocity = 0;
        mot->timeout.start();

        mot->param.vel_p = 0.163;
        mot->param.vel_i = 0.333;

        mot->sens.is_connected = 0;
        mot->sens.heartbeat_received = 0;
        mot->sens.error_received = 0;
        mot->sens.encoder_estimate_received = 0;
        mot->sens.torques_received = 0;
        mot->sens.axis_error = 0;
        mot->sens.axis_state = 0;
        mot->sens.procedure_result = 0;
        mot->sens.active_errors = 0;
        mot->sens.disarm_reason = 0;
        mot->sens.pos_estimate = 0;
        mot->sens.vel_estimate = 0;
        mot->sens.torque_target = 0;
        mot->sens.torque_estimate = 0;
    }

    uint32_t get_id(uint8_t num, cmd_t cmd){
        return ((_base_id | (uint32_t)num) << 5) | (uint32_t)cmd;
    }

    void set_output(size_t num){
        mot_t* mot = &_mot[num];

        Odrive_mode_t mode = (Odrive_mode_t)mot->power.mode;
        if(ECCtrl_isSafetyOn())
        {
            mode = Odrive_mode_DISABLE;
        }
        
        switch (mode)
        {
        case Odrive_mode_TORQUE:
            this->can_send(get_id(num, cmd_t::m2s_set_input_torque), (const uint8_t*)&mot->power.torque, sizeof(float));
            break;

        case Odrive_mode_VELOCITY:{
            uint8_t data[8];
            memcpy(&data[0], &mot->power.velocity, 4);
            memcpy(&data[4], &mot->power.torque, 4);
            this->can_send(get_id(num, cmd_t::m2s_set_input_vel), (const uint8_t*)&data, sizeof(data));
            }break;

        case Odrive_mode_DISABLE:
        default:{
            // Brake
            float velocity = 0;
            float torque = 0;
            if(num == 0 || num == 1)
            {
                torque = -mot->sens.vel_estimate * 5.0;
                torque = std::max(-16.0f, std::min(16.0f, torque));
            }

            uint8_t data[8];
            memcpy(&data[0], &velocity, 4);
            memcpy(&data[4], &torque, 4);
            this->can_send(get_id(num, cmd_t::m2s_set_input_vel), (const uint8_t*)&data, sizeof(data));
            }break;
        }
    }
};

}
