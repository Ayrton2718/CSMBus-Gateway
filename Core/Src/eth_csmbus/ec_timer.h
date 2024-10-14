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
} ESTimer_t;

void ESTimer_init(void);

void ESTimer_timStart(ESTimer_t* tim);

uint32_t ESTimer_getMs(const ESTimer_t tim);
uint32_t ESTimer_getUs(const ESTimer_t tim);

void ESTimer_delayUs(uint32_t us);

void __ESTimer_interrupt(TIM_HandleTypeDef* htim);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include <functional>

namespace csmbus::timer
{

void timer_bind(ESPort_t port, void* wall_tim);

class WallTimer
{
public:
    WallTimer(void){
    }

    void bind(ESPort_t port)
    {
        csmbus::timer::timer_bind(port, this);
    }

    virtual void timer_callback(void) = 0;
};

class Timer
{
private:
    ESTimer_t _tim;

public:
    Timer(void){}

    void start(void)
    {
        ESTimer_timStart(&_tim);
    }

    void reset(void)
    {
        this->start();
    }

    uint32_t get_ms(void)
    {
        return ESTimer_getMs(_tim);
    }

    uint32_t get_us(void)
    {
        return ESTimer_getUs(_tim);
    }
};

}

#endif

#endif /* SRC_ETH_CSMBUS_EC_TIMER_H */
