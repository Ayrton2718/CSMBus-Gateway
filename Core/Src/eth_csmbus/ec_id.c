/*
 * ec_id.c
 *
 *  Created on: Oct 27, 2023
 *      Author: sen
 */

#include "ec_id.h"

#include <string.h>
#include "main.h"
#include "stm32f1xx_hal.h"

#define EC_ID_CHECK_PATTERN (0x94DACB7A)

#define EC_ID_FLASH_START_ADDR (0x803F000)
#define EC_ID_FLASH_END_ADDR   (0x803F7FF)

#define EC_ID_BLINK_TIM (20)
#define EC_ID_SLEEP_TIM (100)

typedef struct
{
	uint32_t  	check_pattern;
}__attribute__((__packed__)) ECId_flash1_t;

typedef struct
{
    uint16_t     id;
    uint16_t     checksum;
}__attribute__((__packed__)) ECId_flash2_t;

// The type must be 16 bit or larger.
typedef struct
{
	uint32_t     flash1;
    uint32_t     flash2;
}__attribute__((__packed__)) ECId_flashData_t;


static ECId_t g_my_addr;
static uint32_t g_tenMsTimer;
static uint16_t g_push_counter;

static uint16_t     g_idLoopCount;

static void ECId_flashWriteId(ECId_t id);
static ECId_t ECId_flashReadId(void);
static ECType_bool_t ECId_isPushingBtn(void);


void ECId_init(void)
{
    g_my_addr = ECId_flashReadId();
    if(g_my_addr == ECId_UNKNOWN)
    {
        for(uint8_t i = 0; i < 4; i++)
        {
            HAL_GPIO_WritePin(LED_ID_GPIO_Port, LED_ID_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_ETH_TX_GPIO_Port, LED_ETH_TX_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_ETH_RX_GPIO_Port, LED_ETH_RX_Pin, GPIO_PIN_SET);
            HAL_Delay(200);
            HAL_GPIO_WritePin(LED_ID_GPIO_Port, LED_ID_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_ETH_TX_GPIO_Port, LED_ETH_TX_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_ETH_RX_GPIO_Port, LED_ETH_RX_Pin, GPIO_PIN_RESET);
            HAL_Delay(200);
        }

        ECId_flashWriteId(EC_ID_FIXED_ADDR);
        NVIC_SystemReset();
    }
}

ECId_t ECId_getId(void)
{
    return g_my_addr;
}

void ECId_process(ECType_bool_t is_safety_on)
{
    register uint32_t now_tick = HAL_GetTick();

    if(g_tenMsTimer < now_tick)
    {
        g_tenMsTimer = now_tick + 10;

        ECType_bool_t btn = ECId_isPushingBtn();

        if(is_safety_on)
        {
            if(btn)
            {
                g_push_counter++;
            }
            else
            {
                if(300 <= g_push_counter)
                {
                    if(btn == ECTYPE_FALSE)
                    {
                        uint16_t id_num = 0;
                        uint8_t flg = 0;
                        uint32_t end_tick = HAL_GetTick() + 3000;

                        uint16_t blinkFlgCount = 10;
                        HAL_GPIO_WritePin(LED_ETH_TX_GPIO_Port, LED_ETH_TX_Pin, GPIO_PIN_SET);
                        HAL_GPIO_WritePin(LED_ETH_RX_GPIO_Port, LED_ETH_RX_Pin, GPIO_PIN_RESET);
                        while(HAL_GetTick() < end_tick)
                        {
                            ECType_bool_t btn = ECId_isPushingBtn();

                            if(btn && flg == 0)
                            {
                                id_num++;
                                HAL_GPIO_WritePin(LED_ID_GPIO_Port, LED_ID_Pin, GPIO_PIN_SET);
                                flg = 1;
                                end_tick = HAL_GetTick() + 3000;
                            }

                            if((btn == ECTYPE_FALSE) && flg == 1)
                            {
                                HAL_GPIO_WritePin(LED_ID_GPIO_Port, LED_ID_Pin, GPIO_PIN_RESET);
                                flg = 0;
                                end_tick = HAL_GetTick() + 3000;
                            }

                            if(blinkFlgCount == 0)
                            {
                                HAL_GPIO_TogglePin(LED_ETH_TX_GPIO_Port, LED_ETH_TX_Pin);
                                HAL_GPIO_TogglePin(LED_ETH_RX_GPIO_Port, LED_ETH_RX_Pin);
                                blinkFlgCount = 10;
                            }else{
                                blinkFlgCount--;
                            }

                            HAL_Delay(10);
                        }

                        HAL_GPIO_WritePin(LED_ETH_TX_GPIO_Port, LED_ETH_TX_Pin, GPIO_PIN_RESET);
                        HAL_GPIO_WritePin(LED_ETH_RX_GPIO_Port, LED_ETH_RX_Pin, GPIO_PIN_RESET);

                        if((id_num != 0) && (id_num <= ECId_UNKNOWN))
                        {
                            ECId_flashWriteId(id_num - 1);
                            NVIC_SystemReset();
                        }
                    }
                }

                g_push_counter = 0;
            }
        }else{
            g_push_counter = 0;
        }

        if(g_my_addr != ECId_UNKNOWN)
        {
            if(g_push_counter < 300)
            {
                if(g_idLoopCount <= (EC_ID_SLEEP_TIM + EC_ID_BLINK_TIM))
                {
                    HAL_GPIO_WritePin(LED_ID_GPIO_Port, LED_ID_Pin, GPIO_PIN_RESET);
                }else{
                    if((g_idLoopCount % EC_ID_BLINK_TIM) == 0)
                    {
                        HAL_GPIO_TogglePin(LED_ID_GPIO_Port, LED_ID_Pin);
                    }
                }

                if(g_idLoopCount == 0)
                {
                    g_idLoopCount = (((uint16_t)g_my_addr + 1) * EC_ID_BLINK_TIM * 2) + (EC_ID_SLEEP_TIM + EC_ID_BLINK_TIM);
                }else{
                    g_idLoopCount--;
                }
            }else{
                HAL_GPIO_WritePin(LED_ID_GPIO_Port, LED_ID_Pin, GPIO_PIN_RESET);
            }
        }else{
            HAL_GPIO_WritePin(LED_ID_GPIO_Port, LED_ID_Pin, GPIO_PIN_SET);
        }
    }
}


static ECId_t ECId_flashReadId(void)
{
    const ECId_flashData_t* flash_data_p = (const ECId_flashData_t*)EC_ID_FLASH_START_ADDR;
    ECId_flashData_t flash_data;

    memcpy(&flash_data, flash_data_p, sizeof(ECId_flashData_t));

    ECId_flash1_t* flash1 = (ECId_flash1_t*)&flash_data.flash1;
    ECId_flash2_t* flash2 = (ECId_flash2_t*)&flash_data.flash2;

    if(flash1->check_pattern == EC_ID_CHECK_PATTERN)
    {
        if(flash2->checksum == flash2->id)
        {
            if(flash2->id < ECId_UNKNOWN)
            {
                return flash2->id;
            }
            else
            {
                HAL_GPIO_WritePin(LED_ETH_ERR_GPIO_Port, LED_ETH_ERR_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(LED_ETH_TX_GPIO_Port, LED_ETH_TX_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(LED_ETH_RX_GPIO_Port, LED_ETH_RX_Pin, GPIO_PIN_SET);
                HAL_Delay(1000);
                return ECId_UNKNOWN;
            }
        }
        else
        {
            HAL_GPIO_WritePin(LED_ETH_ERR_GPIO_Port, LED_ETH_ERR_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_ETH_TX_GPIO_Port, LED_ETH_TX_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_ETH_RX_GPIO_Port, LED_ETH_RX_Pin, GPIO_PIN_SET);
            HAL_Delay(1000);
            return ECId_UNKNOWN;
        }
    }
    else
    {
        HAL_GPIO_WritePin(LED_ETH_ERR_GPIO_Port, LED_ETH_ERR_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_ETH_TX_GPIO_Port, LED_ETH_TX_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_ETH_RX_GPIO_Port, LED_ETH_RX_Pin, GPIO_PIN_SET);
        HAL_Delay(1000);
        return ECId_UNKNOWN;
    }
}

static void ECId_flashWriteId(ECId_t id)
{
    if(HAL_FLASH_Unlock() != HAL_OK)
    {
        HAL_GPIO_WritePin(LED_ETH_ERR_GPIO_Port, LED_ETH_ERR_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_ETH_TX_GPIO_Port, LED_ETH_TX_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_ETH_RX_GPIO_Port, LED_ETH_RX_Pin, GPIO_PIN_SET);
        HAL_Delay(1000);
        return;
    }

    FLASH_EraseInitTypeDef erase;
    uint32_t pageError = 0;
	erase.TypeErase = FLASH_TYPEERASE_PAGES;	// select sector
    erase.Banks = FLASH_BANK_1;
    erase.PageAddress = EC_ID_FLASH_START_ADDR;
    erase.NbPages = 1;
	if(HAL_FLASHEx_Erase(&erase, &pageError) != HAL_OK)
    {
        HAL_FLASH_Lock();

        HAL_GPIO_WritePin(LED_ETH_ERR_GPIO_Port, LED_ETH_ERR_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_ETH_TX_GPIO_Port, LED_ETH_TX_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_ETH_RX_GPIO_Port, LED_ETH_RX_Pin, GPIO_PIN_SET);
        HAL_Delay(1000);
        return;
    }

    ECId_flashData_t flash_data;
    ECId_flash1_t* flash1 = (ECId_flash1_t*)&flash_data.flash1;
    ECId_flash2_t* flash2 = (ECId_flash2_t*)&flash_data.flash2;
    flash1->check_pattern = EC_ID_CHECK_PATTERN;
    flash2->id = id;
    flash2->checksum = flash2->id;

    if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, EC_ID_FLASH_START_ADDR, (uint64_t)flash_data.flash1) != HAL_OK)
    {
        HAL_GPIO_WritePin(LED_ETH_ERR_GPIO_Port, LED_ETH_ERR_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_ETH_TX_GPIO_Port, LED_ETH_TX_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_ETH_RX_GPIO_Port, LED_ETH_RX_Pin, GPIO_PIN_SET);
        HAL_Delay(1000);
    }

    if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, EC_ID_FLASH_START_ADDR + sizeof(flash_data.flash1), (uint64_t)flash_data.flash2) != HAL_OK)
    {
        HAL_GPIO_WritePin(LED_ETH_ERR_GPIO_Port, LED_ETH_ERR_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_ETH_TX_GPIO_Port, LED_ETH_TX_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_ETH_RX_GPIO_Port, LED_ETH_RX_Pin, GPIO_PIN_SET);
        HAL_Delay(1000);
    }

    if(HAL_FLASH_Lock() != HAL_OK)
    {
        HAL_GPIO_WritePin(LED_ETH_ERR_GPIO_Port, LED_ETH_ERR_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_ETH_TX_GPIO_Port, LED_ETH_TX_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_ETH_RX_GPIO_Port, LED_ETH_RX_Pin, GPIO_PIN_SET);
        HAL_Delay(1000);
        return;
    }
}


static ECType_bool_t ECId_isPushingBtn(void)
{
    static uint8_t befor1_state = 0;
    static uint8_t befor2_state = 0;
    if(!HAL_GPIO_ReadPin(BTN_ID_GPIO_Port, BTN_ID_Pin))
    {
        // on
        if(befor1_state)
        {
            befor2_state = 1;
            return ECTYPE_TRUE;
        }else if(!befor1_state && befor2_state){
            befor1_state = 1;
            return ECTYPE_TRUE;
        }else if(!befor1_state && !befor2_state){
            befor1_state = 1;
            return ECTYPE_FALSE;
        }
    }else{
        // off
        if(!befor1_state)
        {
            befor2_state = 0;
            return ECTYPE_FALSE;
        }else if(befor1_state && !befor2_state){
            befor1_state = 0;
            return ECTYPE_FALSE;
        }else if(befor1_state && befor2_state){
            befor1_state = 0;
            return ECTYPE_TRUE;
        }
    }

    return ECTYPE_FALSE;
}
