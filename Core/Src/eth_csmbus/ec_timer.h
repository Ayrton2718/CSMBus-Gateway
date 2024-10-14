/*
 * ec_timer.h
 *
 *  Created on: Oct 12, 2023
 *      Author: sen
 */

#ifndef SRC_ETH_CSMBUS_EC_TIMER_H
#define SRC_ETH_CSMBUS_EC_TIMER_H

#include "ec_type.h"
#include "tim.h"

#define EC_TIMER_USE_HTIM (&htim7)

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    volatile uint32_t ms;
    volatile uint16_t us;
} ECTimer_t;

void ECTimer_init(void);

void ECTimer_timStart(ECTimer_t* tim);

uint32_t ECTimer_getMs(const ECTimer_t tim);
uint32_t ECTimer_getUs(const ECTimer_t tim);

void ECTimer_delayUs(uint32_t us);

void __ECTimer_interrupt(TIM_HandleTypeDef* htim);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include <functional>

namespace csmbus::timer
{

void timer_bind(ECPort_t port, void* wall_tim);

class WallTimer
{
public:
    WallTimer(void){
    }

    void bind(ECPort_t port)
    {
        csmbus::timer::timer_bind(port, this);
    }

    virtual void timer_callback(void) = 0;
};

class Timer
{
private:
    ECTimer_t _tim;

public:
    Timer(void){}

    void start(void)
    {
        ECTimer_timStart(&_tim);
    }

    void reset(void)
    {
        this->start();
    }

    uint32_t get_ms(void)
    {
        return ECTimer_getMs(_tim);
    }

    uint32_t get_us(void)
    {
        return ECTimer_getUs(_tim);
    }
};

}

#endif

#endif /* SRC_ETH_CSMBUS_EC_TIMER_H */
