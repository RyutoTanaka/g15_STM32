/*
 * spi_data.c
 *
 *  Created on: Aug 31, 2024
 *      Author: KIKS
 */

#include <string.h>
#include <math.h>

#include "spi_data.h"

void commandSerialize(Command* data, uint8_t* buffer){
	uint8_t length = 0;
	powerCommandSerialize(&data->power_command, buffer);
	length += POWER_COMMAND_BUFFER_SIZE;
	uint16_t tmp = 0;
	tmp = float16ToFloat32(data->vel_l);
	memcpy(buffer,&tmp,sizeof(uint16_t));
	length += sizeof(uint16_t);
	tmp = float16ToFloat32(data->vel_r);
	memcpy(buffer+length,&tmp,sizeof(uint16_t));
	length += sizeof(uint16_t);
	tmp = calculateChecksum(buffer,length);
	memcpy(buffer+length,&tmp,sizeof(uint8_t));
}

bool commandDeserialize(Command* data, uint8_t* buffer, uint8_t length){
	bool is_ok = (calculateChecksum(buffer, length - 1) == *(buffer + length))
			&& powerCommandDeserialize(&data->power_command, buffer, POWER_COMMAND_BUFFER_SIZE);
	uint8_t i = POWER_COMMAND_BUFFER_SIZE;
	data->vel_l = float16ToFloat32(*(uint16_t*) buffer + i);
	i += sizeof(uint16_t);
	data->vel_r = float16ToFloat32(*(uint16_t*) buffer + i);
	return is_ok;
}

void resultSerialize(Result* data, uint8_t* buffer){
	uint8_t length = 0;
	powerResultSerialize(&data->power_result, buffer);
	length += POWER_RESULT_BUFFER_SIZE;
	uint16_t tmp = 0;
	tmp = float16ToFloat32(data->vel_l);
	memcpy(buffer,&tmp,sizeof(uint16_t));
	length += sizeof(uint16_t);
	tmp = float16ToFloat32(data->vel_r);
	memcpy(buffer+length,&tmp,sizeof(uint16_t));
	length += sizeof(uint16_t);
	memcpy(buffer+length,&data->cnt_l,sizeof(uint16_t));
	length += sizeof(uint16_t);
	memcpy(buffer+length,&data->cnt_r,sizeof(uint16_t));
	length += sizeof(uint16_t);
	tmp = calculateChecksum(buffer,length);
	memcpy(buffer+length,&tmp,sizeof(uint8_t));
}

bool resultDeserialize(Result* data, uint8_t* buffer, uint8_t length){
	bool is_ok = ((calculateChecksum(buffer,length - 1) == *(buffer+length)))
			&& powerResultDeserialize(&data->power_result, buffer, POWER_RESULT_BUFFER_SIZE);
	if(is_ok){
		data->vel_l = float16ToFloat32(*(uint16_t*)buffer);
		data->vel_r = float16ToFloat32(*((uint16_t*)buffer+2));
		data->cnt_l = *((uint16_t*)buffer+4);
		data->cnt_l = *((uint16_t*)buffer+6);
	}
	return is_ok;
}

// 32ビット浮動小数点数を16ビットに変換する関数
uint16_t float32ToFloat16(float value) {
    uint32_t f32 = *((uint32_t*)&value);
    uint32_t sign = (f32 >> 16) & 0x8000; // 符号ビット
    uint32_t exponent = ((f32 >> 23) & 0xFF) - 127 + 15; // 指数ビット
    uint32_t mantissa = f32 & 0x7FFFFF; // 仮数部

    if (exponent <= 0) {
        // 正規化されていない数
        exponent = 0;
    } else if (exponent > 0x1F) {
        // オーバーフローの場合
        exponent = 0x1F;
        mantissa = 0;
    } else {
        // 最上位のビットを削除して仮数部をシフト
        mantissa >>= 13;
    }

    // 16ビットの浮動小数点数を作成
    return (uint16_t)(sign | (exponent << 10) | mantissa);
}

// 16ビット浮動小数点数を32ビット浮動小数点数に変換する関数
float float16ToFloat32(uint16_t value) {
    uint32_t sign = (value & 0x8000) << 16; // 符号ビット
    uint32_t exponent = (value & 0x7C00) >> 10; // 指数ビット
    uint32_t mantissa = value & 0x03FF; // 仮数部

    if (exponent == 0x1F) {
        // 無限大またはNaN
        if (mantissa != 0) {
            // NaN
            return NAN;
        }
        return sign ? -INFINITY : INFINITY;
    }

    if (exponent == 0) {
        // 正規化されていない数
        if (mantissa == 0) {
            // ゼロ
            return sign ? -0.0f : 0.0f;
        }
        // 正規化されていない数
        while ((mantissa & 0x0400) == 0) {
            mantissa <<= 1;
            exponent--;
        }
        exponent++;
        mantissa &= ~0x0400;
    } else {
        // 正規化数
        mantissa |= 0x0400;
    }

    exponent = exponent + (127 - 15); // IEEE 754 32ビット形式への変換

    // 32ビット浮動小数点数の組み立て
    uint32_t f32 = sign | (exponent << 23) | (mantissa << 13);
    return *((float*)&f32);
}

