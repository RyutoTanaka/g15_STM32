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

void lockerCommandSerialize(LockerCommand* data, uint8_t* buffer){
	memcpy(buffer,data,1);
}
void lockerCommandDeserialize(LockerCommand* data, uint8_t* buffer){
	memcpy(data,buffer,1);
}
void lockerResultSerialize(LockerResult* data, uint8_t* buffer){
	memcpy(buffer,data,1);
}
void lockerResultDeserialize(LockerResult* data, uint8_t* buffer){
	memcpy(data,buffer,1);
}

