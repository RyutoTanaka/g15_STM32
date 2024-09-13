/*
 * can_data.c
 *
 *  Created on: Sep 12, 2024
 *      Author: ryuto
 */

#include "can_data.h"

void powerCommandSerialize(PowerCommand* data, uint8_t* buffer){
	*buffer =  (uint8_t)(data->motor_output) + ((uint8_t)data->power_off << 1);
}

void powerCommandDeserialize(PowerCommand* data, uint8_t* buffer){
	data->motor_output = (buffer[0]&0x01)==0x01;
	data->power_off = (buffer[0]&0x02)==0x02;
}

void powerResultSerialize(PowerResult* data, uint8_t* buffer){
	memcpy(buffer, data, sizeof(PowerResult));
}

void powerResultDeserialize(PowerResult* data, uint8_t* buffer){
	memcpy(data, buffer, sizeof(PowerResult));
}

