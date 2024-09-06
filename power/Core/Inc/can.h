/*
 * can.h
 *
 *  Created on: Aug 31, 2024
 *      Author: KIKS
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_

#include <string.h>
#include <stdbool.h>

#include "../../../MDC/Core/Inc/can_data.h"
#include "stm32f3xx_hal_can.h"

static uint8_t g_power_tx_data[POWER_RESULT_BUFFER_SIZE];
static uint8_t g_power_rx_data[POWER_COMMAND_BUFFER_SIZE];

static CAN_HandleTypeDef* g_hcan;

static CAN_FilterTypeDef g_filter;

static bool g_power_updated = false;

void canInit(CAN_HandleTypeDef *hcan){
	g_hcan = hcan;
	uint32_t fId1 = MDC_CAN_ID << 21;
	g_filter.FilterIdHigh         = fId1 >> 16;               // フィルターID(上位16ビット)
	g_filter.FilterIdLow          = fId1;                     // フィルターID(下位16ビット)
	g_filter.FilterMaskIdHigh     = 0;                        // フィルターマスク(上位16ビット)
	g_filter.FilterMaskIdLow      = 0;                        // フィルターマスク(下位16ビット)
	g_filter.FilterScale          = CAN_FILTERSCALE_32BIT;    // フィルタースケール
	g_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;         // フィルターに割り当てるFIFO
	g_filter.FilterBank           = 0;                        // フィルターバンクNo
	g_filter.FilterMode           = CAN_FILTERMODE_IDMASK;    // フィルターモード
	g_filter.SlaveStartFilterBank = 14;                       // スレーブCANの開始フィルターバンクNo
	g_filter.FilterActivation     = ENABLE;                   // フィルター無効／有効
	HAL_CAN_Start(hcan);
	HAL_CAN_ConfigFilter(hcan, &g_filter);
	HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void getPowerCanData(PowerResult* res){
	powerResultDeserialize(res, g_power_rx_data, sizeof(g_power_rx_data));
}

void setPowerCanData(PowerCommand* cmd){
	powerCommandSerialize(cmd, g_power_tx_data);
}

bool isPowerUpdated(){
	if(g_power_updated) {
		g_power_updated = false;
		return true;
	}
	else {
		return false;
	}
}

void sendPowerCanData(){
	CAN_TxHeaderTypeDef TxHeader;
	uint32_t TxMailbox;
	uint8_t data[POWER_COMMAND_BUFFER_SIZE];
	if(0 < HAL_CAN_GetTxMailboxesFreeLevel(g_hcan)){
	    TxHeader.StdId = POWER_CAN_ID;                 // CAN ID
	    TxHeader.RTR = CAN_RTR_DATA;            // フレームタイプはデータフレーム
	    TxHeader.IDE = CAN_ID_STD;              // 標準ID(11ﾋﾞｯﾄ)
	    TxHeader.DLC = 8;                       // データ長は8バイトに
	    TxHeader.TransmitGlobalTime = DISABLE;  // ???
	    memcpy(data,g_power_tx_data,sizeof(g_power_tx_data));
	    HAL_CAN_AddTxMessage(g_hcan, &TxHeader, data, &TxMailbox);
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t data[POWER_RESULT_BUFFER_SIZE];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, data) == HAL_OK)
    {
        uint32_t id = RxHeader.StdId;
        if(id == POWER_CAN_ID){
        	g_power_updated = true;
        	memcpy(g_power_rx_data,data,sizeof(g_power_rx_data));
        }
    }
}

#endif /* INC_CAN_H_ */