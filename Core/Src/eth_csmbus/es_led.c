#include "es_led.h"

#include <main.h>

#define ES_LED_ID_BLINK_TIM (10)
#define ES_LED_ID_SLEEP_TIM (100)


static uint8_t      g_isEnableBlink;
static uint32_t     g_tenMsTimer;

static uint8_t      g_txStopCount;
static uint8_t      g_rxStopCount;

static uint8_t      g_errStopCount;
static uint8_t      g_errFlgCount;

static uint8_t      g_can1StopCount;
static uint8_t      g_can2StopCount;

void ESLed_init(void)
{
    g_isEnableBlink = 1;
    g_tenMsTimer = 0;

	g_txStopCount = 0;
	g_rxStopCount = 0;

    g_errStopCount = 0;
	g_errFlgCount = 0;

    g_can1StopCount = 0;
    g_can2StopCount = 0;
    
    HAL_GPIO_WritePin(LED_ETH_ERR_GPIO_Port, LED_ETH_ERR_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_ETH_TX_GPIO_Port, LED_ETH_TX_Pin, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(LED_ETH_RX_GPIO_Port, LED_ETH_RX_Pin, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(LED_CAN_RX1_GPIO_Port, LED_CAN_RX1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_CAN_RX2_GPIO_Port, LED_CAN_RX2_Pin, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(LED_ID_GPIO_Port, LED_ID_Pin, GPIO_PIN_RESET);
}

void ESLed_ethTx(void)
{
	g_txStopCount = 1;
}

void ESLed_ethRx(void)
{
	g_rxStopCount = 2;
}

void ESLed_err(void)
{
    HAL_GPIO_WritePin(LED_ETH_ERR_GPIO_Port, LED_ETH_ERR_Pin, GPIO_PIN_SET);
	g_errStopCount = 50;
}

void ESLed_bus_err(void)
{
    HAL_GPIO_WritePin(LED_ETH_ERR_GPIO_Port, LED_ETH_ERR_Pin, GPIO_PIN_SET);
	g_errStopCount = 255;
}

void ESLed_canRx1(void)
{
    g_can1StopCount = 1;
}

void ESLed_canRx2(void)
{
    g_can2StopCount = 1;
}


void ESLed_hungUp(void)
{
	g_isEnableBlink = 0;

    HAL_GPIO_WritePin(LED_ETH_ERR_GPIO_Port, LED_ETH_ERR_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_ETH_TX_GPIO_Port, LED_ETH_TX_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_ETH_RX_GPIO_Port, LED_ETH_RX_Pin, GPIO_PIN_SET);
}


void ESLed_process(ESType_bool_t is_safety_on)
{
    register uint32_t now_tick = HAL_GetTick();

    if(g_isEnableBlink && g_tenMsTimer < now_tick)
    {
        g_tenMsTimer = now_tick + 10;

        if(g_txStopCount == 0)
        {
            HAL_GPIO_WritePin(LED_ETH_TX_GPIO_Port, LED_ETH_TX_Pin, GPIO_PIN_RESET);
        }else{
            HAL_GPIO_WritePin(LED_ETH_TX_GPIO_Port, LED_ETH_TX_Pin, GPIO_PIN_SET);
            g_txStopCount--;
        }

        if(g_rxStopCount == 0)
        {
            HAL_GPIO_WritePin(LED_ETH_RX_GPIO_Port, LED_ETH_RX_Pin, GPIO_PIN_RESET);
        }else{
            HAL_GPIO_WritePin(LED_ETH_RX_GPIO_Port, LED_ETH_RX_Pin, GPIO_PIN_SET);
            g_rxStopCount--;
        }

        if(g_can1StopCount == 0)
        {
            HAL_GPIO_WritePin(LED_CAN_RX1_GPIO_Port, LED_CAN_RX1_Pin, GPIO_PIN_RESET);
        }else{
            HAL_GPIO_WritePin(LED_CAN_RX1_GPIO_Port, LED_CAN_RX1_Pin, GPIO_PIN_SET);
            g_can1StopCount--;
        }

        if(g_can2StopCount == 0)
        {
            HAL_GPIO_WritePin(LED_CAN_RX2_GPIO_Port, LED_CAN_RX2_Pin, GPIO_PIN_RESET);
        }else{
            HAL_GPIO_WritePin(LED_CAN_RX2_GPIO_Port, LED_CAN_RX2_Pin, GPIO_PIN_SET);
            g_can2StopCount--;
        }

        if(g_errStopCount == 0)
        {
            HAL_GPIO_WritePin(LED_ETH_ERR_GPIO_Port, LED_ETH_ERR_Pin, GPIO_PIN_RESET);
            g_errFlgCount = 10;
        }else{
            if(g_errFlgCount == 0)
            {
                g_errFlgCount = 10;
                HAL_GPIO_TogglePin(LED_ETH_ERR_GPIO_Port, LED_ETH_ERR_Pin);
            }else{
                g_errFlgCount--;
            }
            g_errStopCount--;
        }
    }
}
