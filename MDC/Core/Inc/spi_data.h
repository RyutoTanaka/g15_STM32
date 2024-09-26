/*
 * spi_data.h
 *
 *  Created on: Aug 29, 2024
 *      Author: KIKS
 */

#ifndef INC_SPI_DATA_H_
#define INC_SPI_DATA_H_

#define SPI_BUFFER_SIZE 16

#include <stdbool.h>
#include <stdint.h>

#include "can_data.h"

typedef struct{
	PowerCommand power_command;
	float vel_l;
	float vel_r;
	uint8_t check_sum;
} Command;

typedef struct {
	PowerResult power_result;
	float vel_l;
	float vel_r;
	uint16_t cnt_l;
	uint16_t cnt_r;
	uint8_t check_sum;
} Result;

// チェックサムを計算する関数
uint8_t calculateChecksum(uint8_t* data, uint8_t length);

void commandSerialize(Command*, uint8_t*);
bool commandDeserialize(Command*, uint8_t*);
void resultSerialize(Result*, uint8_t*);
bool resultDeserialize(Result*, uint8_t*);

#endif /* INC_SPI_DATA_H_ */
