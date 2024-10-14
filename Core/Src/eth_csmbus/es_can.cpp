#include "es_can.h"

#include <string.h>
#include "es_led.h"
#include "es_backdoor.hpp"

#define EC_CAN_SWAP

typedef struct 
{
    CAN_HandleTypeDef* hcan;
    uint8_t wp;
    volatile uint8_t rp;
    volatile uint8_t count;
    std::array<std::pair<CAN_TxHeaderTypeDef, uint8_t[8]>, EC_CAN_BUFF_MAX_COUNT> buff;

    std::array<std::pair<bool, csmbus::can::CanSocket*>, EC_APP_MAX_COUNT> callback;
} ESCan_port_t;

static ESCan_port_t g_obj[2];

static void ESCan_filterSet(ESPort_t port);


ESType_bool_t ESCan_dummyCallback(uint16_t can_id, const uint8_t* data, size_t len)
{
    return ECTYPE_FALSE;
}


void ESCan_init(void)
{
#ifndef EC_CAN_SWAP
    g_obj[0].hcan = &hcan1;
    g_obj[1].hcan = &hcan2;
#else /*EC_CAN_SWAP*/
    g_obj[0].hcan = &hcan2;
    g_obj[1].hcan = &hcan1;
#endif /*EC_CAN_SWAP*/

    for(size_t port_i = 0; port_i < 2; port_i++)
    {
        g_obj[port_i].wp = 0;
        g_obj[port_i].rp = 0;
        g_obj[port_i].count = 0;


        CAN_TxHeaderTypeDef def_header;
        def_header.StdId = 0x000; 		// CAN ID
        def_header.RTR = CAN_RTR_DATA;            // フレームタイプはデータフレーム
        def_header.IDE = CAN_ID_STD;              // 標準ID(11ﾋﾞｯﾄ)
        def_header.DLC = 0;
        def_header.TransmitGlobalTime = DISABLE;  // ???
        for(size_t i = 0; i < EC_CAN_BUFF_MAX_COUNT; i++)
        {
            g_obj[port_i].buff[i].first = def_header;
            g_obj[port_i].buff[i].second[0] = 0;
            memset(g_obj[port_i].buff[i].second, 0x00, sizeof(uint8_t[8]));
        }

        for(size_t i = 0; i < EC_APP_MAX_COUNT; i++)
        {
            g_obj[port_i].callback[i].first = false;
            g_obj[port_i].callback[i].second = nullptr;
        }
    }

#ifdef EC_CAN_ENABLE_RX1_BIND
    ESCan_filterSet(ESPort_1);
#endif /*EC_CAN_ENABLE_RX2_BIND*/
#ifdef EC_CAN_ENABLE_RX2_BIND
    ESCan_filterSet(ESPort_2);
#endif /*EC_CAN_ENABLE_RX1_BIND*/
}

void ESCan_process(void)
{
    for(size_t port_i = 0; port_i < 2; port_i++)
    {
        size_t send_count = g_obj[port_i].count;
        for(size_t i = 0; (i < send_count) && (HAL_CAN_GetTxMailboxesFreeLevel(g_obj[port_i].hcan) != 0); i++)
        {
            uint32_t TxMailbox;
            auto packet = g_obj[port_i].buff[g_obj[port_i].rp & EC_CAN_BUFF_MAX_COUNT];
            HAL_StatusTypeDef result = HAL_CAN_AddTxMessage(g_obj[port_i].hcan, &packet.first, packet.second, &TxMailbox);
            if(result != HAL_OK)
            {
                EC_ERR((ESPort_t)port_i, "Can resend failed by " + std::to_string(result));
                break;
            }
            g_obj[port_i].rp++;
            g_obj[port_i].count--;
        }
    }

    if(HAL_CAN_GetError(&hcan1) != HAL_CAN_ERROR_NONE)
    {
        EC_ERR((ESPort_t)0, "Can getErr 0d" + std::to_string(HAL_CAN_GetError(&hcan1)));
        HAL_CAN_ResetError(&hcan1);
        ESLed_bus_err();
    }

    if(HAL_CAN_GetError(&hcan2) != HAL_CAN_ERROR_NONE)
    {
        EC_ERR((ESPort_t)1, "Can getErr 0d" + std::to_string(HAL_CAN_GetError(&hcan2)));
        HAL_CAN_ResetError(&hcan2);
        ESLed_bus_err();
    }
}

size_t csmbus::can::can_bind(ESPort_t port, void* socket)
{
    size_t array_index = EC_APP_MAX_COUNT - 1;
    bool is_hit = false;

    for(size_t i = 0; i < EC_APP_MAX_COUNT; i++)
    {
        if(g_obj[port].callback[i].second == nullptr)
        {
            g_obj[port].callback[i].second = static_cast<CanSocket*>(socket);
            g_obj[port].callback[i].first = true;
            array_index = i;
            is_hit = true;
            break;
        }
    }

    if(is_hit == false)
    {
        EC_ERR(port, "Too many app");
    }
    return array_index;
}

void csmbus::can::can_send(ESPort_t port, uint16_t can_id, const uint8_t* data, size_t len)
{
    CAN_TxHeaderTypeDef header;
    uint32_t TxMailbox;

    header.StdId = can_id; 		// CAN ID
    header.RTR = CAN_RTR_DATA;            // フレームタイプはデータフレーム
    header.IDE = CAN_ID_STD;              // 標準ID(11ﾋﾞｯﾄ)
    header.DLC = len;
    header.TransmitGlobalTime = DISABLE;  // ???

    if(HAL_CAN_GetTxMailboxesFreeLevel(g_obj[port].hcan) != 0)
    {
        HAL_StatusTypeDef result = HAL_CAN_AddTxMessage(g_obj[port].hcan, &header, data, &TxMailbox);
        if(result != HAL_OK)
        {
            EC_ERR(port, "Can send failed by " + std::to_string(result));
            EC_CAN_SEND_FAIL(port);
        }
    }else{
        if(g_obj[port].count == EC_CAN_BUFF_MAX_COUNT)
        {
            g_obj[port].rp++;
            g_obj[port].count--;
            EC_CAN_OVERFLOW(port);
        }

        g_obj[port].buff[g_obj[port].wp % EC_CAN_BUFF_MAX_COUNT].first = header;
        memcpy(g_obj[port].buff[g_obj[port].wp % EC_CAN_BUFF_MAX_COUNT].second, data, len);
        g_obj[port].wp++;
        g_obj[port].count++;
    }
}

void csmbus::can::set_enable(ESPort_t port, size_t array_index, bool is_enable)
{
    if(array_index < EC_APP_MAX_COUNT)
    {
        g_obj[port].callback[array_index].first = is_enable;
    }

}

#ifdef EC_CAN_ENABLE_RX1_BIND
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
#ifndef EC_CAN_SWAP
    const ESPort_t port = ESPort_1;
#else /*EC_CAN_SWAP*/
    const ESPort_t port = ESPort_2;
#endif /*EC_CAN_SWAP*/

    CAN_RxHeaderTypeDef RxHeader;
	uint8_t RxData[8];

    if((HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK))
    {
        for(size_t i = 0; i < EC_APP_MAX_COUNT; i++)
        {
            if(g_obj[port].callback[i].first && g_obj[port].callback[i].second->can_callback(RxHeader.StdId, RxData, RxHeader.DLC))
            {
                ESLed_canRx1();
            }
        }
    }
}
#endif /*EC_CAN_ENABLE_RX1_BIND*/

#ifdef EC_CAN_ENABLE_RX2_BIND
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
#ifndef EC_CAN_SWAP
    const ESPort_t port = ESPort_2;
#else /*EC_CAN_SWAP*/
    const ESPort_t port = ESPort_1;
#endif /*EC_CAN_SWAP*/

    CAN_RxHeaderTypeDef RxHeader;
	uint8_t RxData[8];

    if((HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, RxData) == HAL_OK))
    {
        for(size_t i = 0; i < EC_APP_MAX_COUNT; i++)
        {
            if(g_obj[port].callback[i].first && g_obj[port].callback[i].second->can_callback(RxHeader.StdId, RxData, RxHeader.DLC))
            {
                ESLed_canRx2();
            }
        }
    }
}
#endif /*EC_CAN_ENABLE_RX2_BIND*/

static void ESCan_filterSet(ESPort_t port)
{
    CAN_HandleTypeDef* hcan;
    uint32_t FilterFIFOAssignment;
    uint32_t FilterBank;
    uint32_t ActiveITs;

    if(g_obj[port].hcan->Instance == hcan1.Instance)
    {
        hcan = &hcan1;
		FilterFIFOAssignment = CAN_FILTER_FIFO0;
		FilterBank = 0;
		ActiveITs = CAN_IT_RX_FIFO0_MSG_PENDING;
    }else{
        hcan = &hcan2;
        FilterFIFOAssignment = CAN_FILTER_FIFO1;
        FilterBank = 14;
        ActiveITs = CAN_IT_RX_FIFO1_MSG_PENDING;
    }

    CAN_FilterTypeDef filter;
	filter.FilterIdHigh         = 0;                           // フィルターID(上位16ビット)
	filter.FilterIdLow          = 0;                           // フィルターID(下位16ビット)
	filter.FilterMaskIdHigh     = 0;                           // フィルターマスク(上位16ビット)
	filter.FilterMaskIdLow      = 0;                           // フィルターマスク(下位16ビット)
	filter.FilterScale          = CAN_FILTERSCALE_32BIT;       // フィルタースケール
	filter.FilterFIFOAssignment = FilterFIFOAssignment;        // フィルターに割り当てるFIFO
	filter.FilterBank           = FilterBank;                  // フィルターバンクNo
	filter.FilterMode           = CAN_FILTERMODE_IDMASK;       // フィルターモード
	filter.SlaveStartFilterBank = 14;                          // スレーブCANの開始フィルターバンクNo
	filter.FilterActivation     = ENABLE;                      // フィルター無効／有効
	if (HAL_CAN_ConfigFilter(hcan, &filter) != HAL_OK)
	{
        ESLed_hungUp();
	}

    if(HAL_CAN_ActivateNotification(hcan, ActiveITs) != HAL_OK)
	{
        ESLed_hungUp();
	}

    if(HAL_CAN_Start(hcan) != HAL_OK)
    {
        ESLed_hungUp();
    }
}
