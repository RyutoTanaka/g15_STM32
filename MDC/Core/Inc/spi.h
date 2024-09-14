/*
 * spi.h
 *
 *  Created on: Aug 29, 2024
 *      Author: KIKS
 */

#ifndef INC_SPI_H_
#define INC_SPI_H_

#include <string.h>
#include <stdbool.h>

#include "spi_data.h"

#include "stm32f3xx_hal_spi.h"

static uint8_t g_spi_tx_data[SPI_BUFFER_SIZE] = {0};
static uint8_t g_spi_rx_data[SPI_BUFFER_SIZE] = {0};

static SPI_HandleTypeDef* g_hspi;

static bool g_spi_updated = false;

void spiInit(SPI_HandleTypeDef *hspi){
	g_hspi = hspi;
	HAL_SPI_TransmitReceive_DMA(hspi, g_spi_tx_data, g_spi_rx_data, sizeof(g_spi_tx_data));
}

void getSpiData(Command* cmd){
	g_spi_updated = false;
	commandDeserialize(cmd, g_spi_rx_data);
}

void setSpiData(Result* res){
	resultSerialize(res, g_spi_tx_data);
}

bool isSpiUpdated(){
	return g_spi_updated;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
	g_spi_updated = true;
	HAL_SPI_Transmit_DMA(hspi, g_spi_tx_data, sizeof(g_spi_tx_data));
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi){
	g_spi_updated = true;
	uint8_t tx_buffer[SPI_BUFFER_SIZE] = {0};
	HAL_SPI_TransmitReceive_DMA(hspi, tx_buffer, g_spi_rx_data, sizeof(g_spi_rx_data));
}

#endif /* INC_SPI_H_ */
