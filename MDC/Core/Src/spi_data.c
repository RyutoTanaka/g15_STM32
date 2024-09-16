/*
 * spi_data.c
 *
 *  Created on: Aug 31, 2024
 *      Author: KIKS
 */

#include <string.h>
#include <math.h>

#include "spi_data.h"

// チェックサムを計算する関数
uint8_t calculateChecksum(uint8_t* data, uint8_t length) {
	int sum = 0;
	for (uint8_t i = 0; i < length; i++) {
			sum += data[i];
	}
	// 合計の下位バイトを返す
	return (uint8_t)(sum & 0xFF);
}

void commandSerialize(Command* data, uint8_t* buffer){
	uint8_t length = 0;
	powerCommandSerialize(&data->power_command, buffer);
	length += POWER_COMMAND_BUFFER_SIZE;
	memcpy(buffer+length,&data->vel_l,sizeof(float));
	length += sizeof(float);
	memcpy(buffer+length,&data->vel_r,sizeof(float));
	buffer += sizeof(float);
	data->check_sum = calculateChecksum(buffer, length);
	memcpy(buffer+length,&data->check_sum,sizeof(uint8_t));
}

bool commandDeserialize(Command* data, uint8_t* buffer){
	powerCommandDeserialize(&data->power_command, buffer);
	uint8_t length = POWER_COMMAND_BUFFER_SIZE;
	memcpy(&data->vel_l,buffer+length,sizeof(float));
	length += sizeof(float);
	memcpy(&data->vel_r,buffer+length,sizeof(float));
	length += sizeof(float);
	memcpy(&data->check_sum,buffer+length,sizeof(uint8_t));
	return calculateChecksum(buffer, length) == data->check_sum;
}

void resultSerialize(Result* data, uint8_t* buffer){
	uint8_t length = 0;
	powerResultSerialize(&data->power_result, buffer);
	length += POWER_RESULT_BUFFER_SIZE;
	memcpy(buffer+length,&data->vel_l,sizeof(float));
	length += sizeof(float);
	memcpy(buffer+length,&data->vel_r,sizeof(float));
	length += sizeof(float);
	memcpy(buffer+length,&data->cnt_l,sizeof(uint16_t));
	length += sizeof(uint16_t);
	memcpy(buffer+length,&data->cnt_r,sizeof(uint16_t));
	length += sizeof(uint16_t);
	data->check_sum = calculateChecksum(buffer,length);
	memcpy(buffer+length,&data->check_sum,sizeof(uint8_t));
}

bool resultDeserialize(Result* data, uint8_t* buffer){
	powerResultDeserialize(&data->power_result, buffer);
	uint8_t length =  POWER_RESULT_BUFFER_SIZE;
	memcpy(&data->vel_l,buffer+length,sizeof(float));
	length += sizeof(float);
	memcpy(&data->vel_r,buffer+length,sizeof(float));
	length += sizeof(float);
	memcpy(&data->cnt_l,buffer+length,sizeof(uint16_t));
	length += sizeof(uint16_t);
	memcpy(&data->cnt_r,buffer+length,sizeof(uint16_t));
	length += sizeof(uint16_t);
	memcpy(&data->check_sum,buffer+length,sizeof(uint8_t));
	return calculateChecksum(buffer, length) == data->check_sum;
}




