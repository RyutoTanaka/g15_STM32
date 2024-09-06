/*
 * spi_data.h
 *
 *  Created on: Aug 29, 2024
 *      Author: KIKS
 */

#ifndef INC_SPI_DATA_H_
#define INC_SPI_DATA_H_

#define SPI_COMMAND_BUFFER_SIZE 7
#define SPI_RESULT_BUFFER_SIZE 13

#include <stdbool.h>
#include <stdint.h>

#include "can_data.h"
typedef struct{
	PowerCommand power_command;
	float vel_l;
	float vel_r;
} Command;

typedef struct {
	PowerResult power_result;
	float vel_l;
	float vel_r;
	uint16_t cnt_l;
	uint16_t cnt_r;
} Result;

void commandSerialize(Command*, uint8_t*);
bool commandDeserialize(Command*, uint8_t*, uint8_t);
void resultSerialize(Result*, uint8_t*);
bool resultDeserialize(Result*, uint8_t*, uint8_t);
uint16_t float32ToFloat16(float);
float float16ToFloat32(uint16_t);

#endif /* INC_SPI_DATA_H_ */
