/*
 * can_data.h
 *
 *  Created on: Aug 29, 2024
 *      Author: KIKS
 */

#ifndef INC_CAN_DATA_H_
#define INC_CAN_DATA_H_

#include <stdbool.h>
#include <stdint.h>

#define MDC_CAN_ID 0x100
#define POWER_CAN_ID 0x200

#define POWER_COMMAND_BUFFER_SIZE 2
#define POWER_RESULT_BUFFER_SIZE 4

typedef struct{
	bool motor_output;
	bool power_off;
} PowerCommand;

typedef struct {
	bool is_motor_output;  //true : motor power is suppled
	bool is_not_emergency; //true : not emergency
	uint8_t v_bat; // 0: 255:
	uint8_t i_bat; // 0: 255:
} PowerResult;

// チェックサムを計算する関数
uint8_t calculateChecksum(uint8_t* data, uint8_t length);

void powerCommandSerialize(PowerCommand* data, uint8_t* buffer);

bool powerCommandDeserialize(PowerCommand* data, uint8_t* buffer, uint8_t length);

void powerResultSerialize(PowerResult* data, uint8_t* buffer);

bool powerResultDeserialize(PowerResult* data, uint8_t* buffer, uint8_t length);

#endif /* INC_CAN_DATA_H_ */
