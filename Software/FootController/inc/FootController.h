/*
 * Copyright (c) 2021, Adrian Chemicz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *    3. Neither the name of contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _FOOT_CONTROLLER_H_
#define _FOOT_CONTROLLER_H_

/*
 * This is master module with main function. This module is reponsible for call
 * hardware initialization, handle USB communication via virtual serial port and
 * HID interface and call laser module. This module provide for other module
 * global FootControllerStructure structure which contain most important information
 * about device.
 */

#include <stdbool.h>
#include <stdint.h>

#define MAX_SUPPORTED_SENSOR 					1
#define MAX_SUPPORTED_LASER_MEASUREMENT 		99
#define NUMBER_OF_SEND_WAIT_CYCLE 				2000
#define LASER_JITTER_VALUE 						5
#define LASER_JITTER_SUSPEND_STATE_VALUE 		8
#define NUMBER_OF_COUNT_TO_CLEAR_LASER_OFFSET 	5
#define MAX_MEASUREMENTS_IN_SINGLE_SEND			5
#define MAX_SUPPORTED_RANGE_IN_MM				300
#define MINIMAL_LASER_SENSOR_RANGE_VALUE		16

//HVQFN33-16  PIO0_7/CTS  CN7-11
#define CDC_STATE_INDICATOR_PORT				0
#define CDC_STATE_INDICATOR_PIN					7

//HVQFN33-17  PIO0_8/MISO0/CT16B0_MAT0  CN7-10
#define FIRST_SENSOR_CONTROL_LINE_PORT			0
#define FIRST_SENSOR_CONTROL_LINE_PIN			8

//HVQFN33-18  PIO0_9/MOSI0/CT16B0_MAT1  CN7-9
#define SECOND_SENSOR_CONTROL_LINE_PORT			0
#define SECOND_SENSOR_CONTROL_LINE_PIN			9

//HVQFN33-1  PIO1_19/DTR/SSEL1  CN8-4
#define BLOCK_SEND_EVENT_PORT					1
#define BLOCK_SEND_EVENT_PIN					19

//HVQFN33-27  PIO0_23/AD7  CN7-2
#define DEBUG_PORT								0
#define DEBUG_PIN								23

#define LASER_SENSOR_CONFIG_BEGINNING			64
#define LASER_MEASUREMENT_BUFFER_DEPH			10

#define NUMBER_OF_STORED_ERRORS					8
#define NUMBER_OF_CYCLE_TO_CLEAR_ERROR			40000
#define INITIALIZATION_ERROR_CODE				1
#define NO_FAULT_ERROR_CODE						0
#define ERROR_CODE_POSITION						0
#define INDEX_CODE_POSITION						1

#define NUMBER_OF_CYCLE_TO_REACH_1S_BY_TIMER	11976

typedef enum SELECTED_COMMAND_TYPE
{
	HELP_COMAMND,
	STATUS_COMAMND,
	CONFIGURE_SENSOR_COMAMND,
	GET_RANGE_COMAMND,
	SUSPEND_EVENT_COMMAND,
	EVENT_PARAMETERS_COMMAND,
	UNKNOWN_COMMAND
}SELECTED_COMMAND;

typedef enum KEY_MODE_TYPE
{
	PRESS_KEY,
	HOLD_KEY
}KEY_MODE;

typedef struct
{
	uint8_t keyCode;
	uint8_t keyMode;
	uint16_t laserRangeThreshold;
}SensorActivationParametersType;

typedef struct
{
	SensorActivationParametersType sensorActivationParameters;
	uint16_t counter;
	uint16_t checksum;
}SensorActivationParametersEepromStoreType;

typedef struct
{
	//flag which contain decision that button event must be send via USB or not
	bool eventKeyState;

	//flag which contain state with information that key event was send via USB
	bool eventKeyWasSend;

	//flag with state set when appropriate range will be traveled by foot
	bool thresholdState;

	//this counter is used to clear laserRangeOffset if value is not big enaugh to raise event
	uint16_t clearLaserOffsetCounter;

	//value hold travel of foot above sensor. It is used during comparison which decide about event
	uint16_t laserRangeOffset;

	/* value which hold first measurement of foot possition when foot start moving.
	 Using this value laserRangeOffset is calculated */
	uint16_t distanceAssignedToRangeOffset;

	//variable used to store last laser distance used to check that foot move
	uint16_t previousLaserMeasurement;
}LaserSensorEventStructureType;

typedef struct
{
	SensorActivationParametersType sensorActivationParametersTable[MAX_SUPPORTED_SENSOR];
	uint8_t processedCommand;
	bool correctCommandParametersFlag;

	//parameters for GET_RANGE_COMAMND
	uint16_t numberOfRangeMeasurement;
	uint16_t laserMeasurementBuffer[LASER_MEASUREMENT_BUFFER_DEPH];
	uint16_t laserMeasurementCurrentStoreIndex;
	uint16_t laserMeasurementCurrentSendIndex;
	uint16_t laserMeasurementIndex;
	bool lockTableFlag;

	//parameters for laser sensor event
	LaserSensorEventStructureType laserSensorEventTable[MAX_SUPPORTED_SENSOR];

	//parameters for STATUS_COMAMND
	uint16_t deviceStatus;/* 0 mean no error. Other value mean step on which LaserProcess
	function stoped. */
	uint16_t errorCounter;
	uint16_t lastErrorCodeTable[NUMBER_OF_STORED_ERRORS][2];
	uint16_t *pointerToCurrentErrorCode;
	uint16_t clearErrorCounter;

	//parameters for SUSPEND_EVENT_COMMAND and virtual serial temporary suspend
	bool suspendSendEventByCdc;
	uint32_t suspendSerialCommandCounter;
	bool suspendSendEventPermanentlyWitOpenSerial;

	/* Pointer to function which will generate string. Function as parameter expect pointer to string
	 * buffer which will be used to generate string. Function return true when all data was send
	 * otherwise false. */
	bool (*stringFunctionGenerator)(uint8_t*);
}FootControllerStructureType;

typedef struct
{
	bool duringSendTransactionFlag;
	uint16_t stringOffset;
	uint8_t *stringPointer;
	uint16_t transactionSendWaitCounter;
}CDC_MessageStructureType;

extern FootControllerStructureType FootControllerStructure;

#endif /* _FOOT_CONTROLLER_H_ */
