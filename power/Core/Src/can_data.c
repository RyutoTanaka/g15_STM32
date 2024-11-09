/*
 * can_data.c
 *
 *  Created on: Sep 12, 2024
 *      Author: ryuto
 */

#include "can_data.h"

void canCommandSerialize(CanCommand* data, uint8_t* buffer){
	buffer[0] = data->motor_output
						+ data->power_off << 1
						+ data->mode      << 2
						+ data->pull      << 3
						+ data->release   << 4;
}

void canCommandDeserialize(CanCommand* data, uint8_t* buffer){
	data->motor_output = (buffer[0] & 0b00000001) != 0;
	data->power_off    = (buffer[0] & 0b00000010) != 0;
	data->mode         = (buffer[0] & 0b00000100) != 0;
	data->pull         = (buffer[0] & 0b00001000) != 0;
	data->release      = (buffer[0] & 0b00010000) != 0;

}

void powerResultSerialize(PowerResult* data, uint8_t* buffer){
	buffer[0] = (uint8_t)(data->emergency) + ((uint8_t)data->motor_output << 1);
	buffer[1] = data->v_bat;
	buffer[2] = data->i_bat;
}

void powerResultDeserialize(PowerResult* data, uint8_t* buffer){
	data->emergency = (buffer[0]&0x01)==0x01;
	data->motor_output = (buffer[0]&0x02)==0x02;
	data->v_bat = buffer[1];
	data->i_bat = buffer[2];
}

void lockerResultSerialize(LockerResult* data, uint8_t* buffer){
	memcpy(buffer,data,1);
}
void lockerResultDeserialize(LockerResult* data, uint8_t* buffer){
	memcpy(data,buffer,1);
}

