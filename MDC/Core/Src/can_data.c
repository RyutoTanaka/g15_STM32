/*
 * can_data.c
 *
 *  Created on: Aug 31, 2024
 *      Author: KIKS
 */

#include <string.h>

#include "can_data.h"

void powerCommandSerialize(PowerCommand* data, uint8_t* buffer){
	*buffer =  (uint8_t)(data->motor_output) + ((uint8_t)data->power_off << 1);
}

bool powerCommandDeserialize(PowerCommand* data, uint8_t* buffer, uint8_t length){
	bool is_ok = (calculateChecksum(buffer,length - 1) == *(buffer+length));
	data->motor_output = (buffer[0]&0x01)==0x01;
	data->power_off = (buffer[0]&0x02)==0x02;
	return is_ok;
}

void powerResultSerialize(PowerResult* data, uint8_t* buffer){
	uint8_t length = 0;
	uint8_t tmp = (uint8_t) data->is_motor_output + ((uint8_t) data->is_not_emergency << 1);
	memcpy(buffer,&tmp,sizeof(uint8_t));
	length += sizeof(uint8_t);
	memcpy(buffer+length,&data->v_bat,sizeof(uint16_t));
	length += sizeof(uint16_t);
	memcpy(buffer+length,&data->i_bat,sizeof(uint16_t));
	length += sizeof(uint16_t);
	tmp = calculateChecksum(buffer,length);
	memcpy(buffer+length,&tmp,sizeof(uint8_t));
}

bool powerResultDeserialize(PowerResult* data, uint8_t* buffer, uint8_t length){
	bool is_ok = (calculateChecksum(buffer,length - 1) == *(buffer+length));
	if(is_ok){
		data->is_motor_output = (buffer[0] & 0x01) == 0x01;
		data->is_not_emergency = (buffer[0] & 0x02) == 0x02;
		data->v_bat = buffer[1];
		data->i_bat = buffer[2];
	}

	return is_ok;
}

// チェックサムを計算する関数
uint8_t calculateChecksum(uint8_t* data, uint8_t length) {
	int sum = 0;
	for (uint8_t i = 0; i < length; i++) {
			sum += data[i];
	}
	// 合計の下位バイトを返す
	return (uint8_t)(sum & 0xFF);
}

