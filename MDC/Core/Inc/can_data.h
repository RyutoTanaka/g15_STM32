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
#include <string.h>

#define MDC_CAN_ID 0x100
#define POWER_CAN_ID 0x200
#define LOCKER_CAN_ID 0x300

#define CAN_COMMAND_BUFFER_SIZE 1
#define POWER_RESULT_BUFFER_SIZE 3
#define LOCKER_RESULT_BUFFER_SIZE 1

typedef struct{
	//power
	bool motor_output;
	bool power_off;
	//locker
	bool mode; //0:chair, 1:desk
	bool pull;
	bool release;
} CanCommand;

typedef struct {
	bool emergency;
	bool motor_output;
	uint8_t v_bat; // 0: 255:
	uint8_t i_bat; // 0: 255:
} PowerResult;

typedef struct {
	uint8_t state;
} LockerResult;

void canCommandSerialize(CanCommand* data, uint8_t* buffer);
void canCommandDeserialize(CanCommand* data, uint8_t* buffer);
void powerResultSerialize(PowerResult* data, uint8_t* buffer);
void powerResultDeserialize(PowerResult* data, uint8_t* buffer);

void lockerResultSerialize(LockerResult* data, uint8_t* buffer);
void lockerResultDeserialize(LockerResult* data, uint8_t* buffer);

#endif /* INC_CAN_DATA_H_ */
