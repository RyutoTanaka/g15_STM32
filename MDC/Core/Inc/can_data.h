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

void powerCommandSerialize(PowerCommand*, uint8_t*);
bool powerCommandDeserialize(PowerCommand*, uint8_t*, uint8_t);
void powerResultSerialize(PowerResult*, uint8_t*);
bool powerResultDeserialize(PowerResult*, uint8_t*, uint8_t);
uint8_t calculateChecksum(uint8_t*, uint8_t);

#endif /* INC_CAN_DATA_H_ */
