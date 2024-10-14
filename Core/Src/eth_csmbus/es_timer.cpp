#include "es_timer.h"

#include "es_backdoor.hpp"
#include "tim.h"

#define ES_TIMER_CB_MAX_COUNT (ES_APP_MAX_COUNT*2)

volatile static uint32_t g_ms_count;

static std::array<std::pair<bool, smbus::timer::WallTimer*>, ES_TIMER_CB_MAX_COUNT> g_callback;

void ESTimer_dummyCallback(void){}

void ESTimer_init(void)
{
    g_ms_count = 0;


    for(size_t i = 0; i < ES_TIMER_CB_MAX_COUNT; i++)
    {
        g_callback[i].first = false;
        g_callback[i].second = nullptr;
    }

    HAL_TIM_Base_Start_IT(ES_TIMER_USE_HTIM);
}


namespace smbus::timer
{

void timer_bind(ESPort_t port, void* wall_tim)
{
    bool is_hit = false;
    for(size_t i = 0; i < ES_TIMER_CB_MAX_COUNT; i++)
    {
        if(g_callback[i].second == nullptr)
        {
            g_callback[i].second = static_cast<WallTimer*>(wall_tim);
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

}


void ESTimer_timStart(ESTimer_t* tim)
{
    uint16_t now_us = __HAL_TIM_GET_COUNTER(ES_TIMER_USE_HTIM);
	uint32_t now_ms = g_ms_count;

    tim->ms = now_ms;
    tim->us = now_us;
}

uint32_t ESTimer_getMs(const ESTimer_t tim)
{
    uint16_t now_us = __HAL_TIM_GET_COUNTER(ES_TIMER_USE_HTIM);
	uint32_t now_ms = g_ms_count;

    uint32_t ms;
    if(tim.us < now_us)
    {
        ms = (now_ms - tim.ms);
    }else if(now_ms == tim.ms){
        ms = 0;
    }else{
        ms = (now_ms - tim.ms - 1);
    }
    return ms;
}

uint32_t ESTimer_getUs(const ESTimer_t tim)
{
    uint16_t now_us = __HAL_TIM_GET_COUNTER(ES_TIMER_USE_HTIM);
    uint32_t now_ms = g_ms_count;

    uint32_t us;
    if(tim.us < now_us)
    {
    	us = (now_ms - tim.ms) * 1000 + (now_us - tim.us);
    }else if(now_ms == tim.ms){
        us = ((1000 + now_us) - tim.us);
    }else{
    	us = (now_ms - tim.ms - 1) * 1000 + ((1000 + now_us) - tim.us);
    }
    return us;
}

void ESTimer_delayUs(uint32_t us)
{
    ESTimer_t start;
    ESTimer_timStart(&start);
    while(1)
    {
        if(us <= ESTimer_getUs(start))
        {
            break;
        }

        __asm__(
            "nop\n\r"
            "nop\n\r"
        );
    }
}


void __ESTimer_interrupt(TIM_HandleTypeDef* htim)
{
    if(htim->Instance == ES_TIMER_USE_HTIM->Instance)
    {
        g_ms_count++;

        for(size_t i = 0; i < ES_TIMER_CB_MAX_COUNT; i++)
        {
            if(g_callback[i].first)
            {
                g_callback[i].second->timer_callback();
            }
        }
    }
}
