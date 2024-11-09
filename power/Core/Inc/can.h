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

#include <stdio.h>

#include "can_data.h"
#include "stm32f3xx_hal_can.h"

static uint8_t g_tx_data[POWER_RESULT_BUFFER_SIZE];
static uint8_t g_rx_data[CAN_COMMAND_BUFFER_SIZE];

static CAN_HandleTypeDef* g_hcan;

static CAN_FilterTypeDef g_filter;

static bool g_updated = false;

HAL_StatusTypeDef canInit(CAN_HandleTypeDef *hcan){
	g_hcan = hcan;
	g_filter.FilterIdHigh         = MDC_CAN_ID << 5;               // フィルターID(上位16ビット)
	g_filter.FilterIdLow          = MDC_CAN_ID << 5;     // フィルターID(下位16ビット)                         // フィルターマスク(下位16ビット)
	g_filter.FilterScale          = CAN_FILTERSCALE_16BIT;    // フィルタースケール
	g_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;         // フィルターに割り当てるFIFO
	g_filter.FilterBank           = 0;                        // フィルターバンクNo
	g_filter.FilterMode           = CAN_FILTERMODE_IDLIST;    // フィルターモード
	g_filter.SlaveStartFilterBank = 14;                       // スレーブCANの開始フィルターバンクNo
	g_filter.FilterActivation     = ENABLE;                   // フィルター無効／有効
	HAL_CAN_Stop(hcan);
	if (HAL_CAN_ConfigFilter(hcan, &g_filter) != HAL_OK) return HAL_ERROR;
	if (HAL_CAN_Start(hcan)!=HAL_OK) return HAL_ERROR;
	if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING)!=HAL_OK) return HAL_ERROR;
	return HAL_OK;
}

void getCanData(CanCommand* cmd){
	g_updated = false;
	canCommandDeserialize(cmd, g_rx_data);
}

void setCanData(PowerResult* res){
	powerResultSerialize(res,g_tx_data);
}

bool isCanUpdated(){
	return g_updated;
}

void sendCanData(){
	CAN_TxHeaderTypeDef TxHeader;
	uint32_t TxMailbox;
	uint8_t data[8];
	if(0 < HAL_CAN_GetTxMailboxesFreeLevel(g_hcan)){
	    TxHeader.StdId = POWER_CAN_ID;          // CAN ID
	    TxHeader.RTR = CAN_RTR_DATA;            // フレームタイプはデータフレーム
	    TxHeader.IDE = CAN_ID_STD;              // 標準ID(11ﾋﾞｯﾄ)
	    TxHeader.DLC = POWER_RESULT_BUFFER_SIZE;// データ長は8バイトに
	    TxHeader.TransmitGlobalTime = DISABLE;  // ???
	    memcpy(data,g_tx_data,sizeof(g_tx_data));
	    if(HAL_CAN_AddTxMessage(g_hcan, &TxHeader, data, &TxMailbox)!=HAL_OK) Error_Handler();
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t data[8];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, data) == HAL_OK)
    {
    	//printf("callback %d",RxHeader.StdId);
				g_updated = true;
				memcpy(g_rx_data,data,sizeof(g_rx_data));
    }
}

#endif /* INC_CAN_H_ */
