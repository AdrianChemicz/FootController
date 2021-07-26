/*
 * Copyright (c) 2021 Adrian Chemicz
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

#include "StringGenerateParseModule.h"
#include <stddef.h>

/*****************************************************************************************
* onlyDigitsExistInString() - function to verfiifcation that string set as parameter
* contain only digits.
*
* Parameters:
* @verifiedString: pointer to string which will be verified by function.
*
* Return: Return true if string contain only numbers as characters otherwise false.
*****************************************************************************************/
bool onlyDigitsExistInString(uint8_t *verifiedString)
{
	for (int i = 0; i < strlen(verifiedString); i++)
	{
		if (isdigit(verifiedString[i]) == 0)
		{
			return false;
		}
	}

	return true;
}

/*****************************************************************************************
* atGetRangeCountStringGenerator() - function generate string for at command
* AT_GET_RANGE_COUNT_. To send all measurement set as parameters of at commead is possible
* that function will call multiply time. In one call function build string from multiply
* laser measurement and put it to buffer. In FootControllerStructure global structure
* function use necessary fields to construct string.
*
* Parameters:
* @transmitBuffer: pointer to string buffer which will be used to construct string for
*  send.
*
* Return: true if string was finished othrwise false.
*****************************************************************************************/
bool atGetRangeCountStringGenerator(uint8_t* transmitBuffer)
{
	uint8_t stringBufferTmp[8] = {0, 0, 0, 0, 0, 0, 0, 0};

	if (FootControllerStructure.laserMeasurementIndex == 0)
	{
		strcpy(transmitBuffer, " - correct parameters\r\n");
		FootControllerStructure.laserMeasurementIndex++;
		return false;
	}
	else if (FootControllerStructure.laserMeasurementIndex < (FootControllerStructure.numberOfRangeMeasurement + 1))
	{
		transmitBuffer[0] = 0;

		for (int i = 0; i < MAX_MEASUREMENTS_IN_SINGLE_SEND; i++)
		{
			//number of gathered measurement in  buffer is empty
			if (FootControllerStructure.laserMeasurementCurrentSendIndex == FootControllerStructure.laserMeasurementCurrentStoreIndex)
			{
				break;
			}

			if (FootControllerStructure.laserMeasurementIndex > FootControllerStructure.numberOfRangeMeasurement)
			{
				FootControllerStructure.laserMeasurementIndex++;
				break;
			}

			itoa(FootControllerStructure.laserMeasurementBuffer[FootControllerStructure.laserMeasurementCurrentSendIndex], stringBufferTmp, 10);

			if (i == 0)
			{
				strcpy(transmitBuffer, stringBufferTmp);
			}
			else
			{
				strcat(transmitBuffer, stringBufferTmp);
			}

			strcat(transmitBuffer, "mm\r\n");

			FootControllerStructure.laserMeasurementIndex++;
			FootControllerStructure.laserMeasurementCurrentSendIndex++;
		}/* for (int i = 0; i < MAX_MEASUREMENTS_IN_SINGLE_SEND; i++) */

		//move buffer if other thread not perform operation
		if ((FootControllerStructure.lockTableFlag == false)
			&& (FootControllerStructure.laserMeasurementCurrentSendIndex > 0))
		{
			uint16_t numberOfDataInBuffer = FootControllerStructure.laserMeasurementCurrentStoreIndex - FootControllerStructure.laserMeasurementCurrentSendIndex;

			if (numberOfDataInBuffer == 0)
			{
				FootControllerStructure.laserMeasurementCurrentSendIndex = 0;
				FootControllerStructure.laserMeasurementCurrentStoreIndex = 0;
			}
			else
			{
				memcpy(FootControllerStructure.laserMeasurementBuffer, &FootControllerStructure.laserMeasurementBuffer[FootControllerStructure.laserMeasurementCurrentSendIndex], numberOfDataInBuffer*2);

				FootControllerStructure.laserMeasurementCurrentSendIndex = 0;
				FootControllerStructure.laserMeasurementCurrentStoreIndex = numberOfDataInBuffer;
			}
		}/* if ((FootControllerStructure.lockTableFlag == false)
			&& (FootControllerStructure.laserMeasurementCurrentSendIndex == 0)) */

		return false;
	}
	else
	{
		transmitBuffer[0] = 0;
		FootControllerStructure.numberOfRangeMeasurement = 0;

		return true;
	}
}/* bool atGetRangeCountStringGenerator(uint8_t* transmitBuffer) */

/*****************************************************************************************
* generateStatusString() - function generate string for at command
* AT_STATUS. Function generate string with "Current system status", number of errors and
* previous errors as array. Other state/error different than 0 mean that LaserProcess
* function crash during execution and throw error. This function return errors in cyclic
* buffer which can be analize why it cause.
*
* Parameters:
* @transmitBuffer: pointer to string buffer which will be used to construct string for
* send.
*
*****************************************************************************************/
void generateStatusString(uint8_t* transmitBuffer)
{
	uint8_t stringBufferTmp[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	uint16_t *pointerToErrorCode = &FootControllerStructure.pointerToCurrentErrorCode[ERROR_CODE_POSITION];
	uint16_t temporaryErrorBuffer[NUMBER_OF_STORED_ERRORS];

	strcpy(transmitBuffer, " - Current system status: 0x");

	itoa(FootControllerStructure.deviceStatus, stringBufferTmp, 16);
	strcat(transmitBuffer, stringBufferTmp);

	strcat(transmitBuffer, " , number of errors: ");

	itoa(FootControllerStructure.errorCounter, stringBufferTmp, 10);
	strcat(transmitBuffer, stringBufferTmp);

	strcat(transmitBuffer, " , previous errors: 0x");

	//get errors to table
	for (int i = 0; i < NUMBER_OF_STORED_ERRORS; i++)
	{
		temporaryErrorBuffer[NUMBER_OF_STORED_ERRORS - 1 - i] = *pointerToErrorCode;

		pointerToErrorCode = FootControllerStructure.lastErrorCodeTable[pointerToErrorCode[INDEX_CODE_POSITION]];
	}

	//add string using data in table
	for (int i = 0; i < NUMBER_OF_STORED_ERRORS; i++)
	{
		itoa(temporaryErrorBuffer[i], stringBufferTmp, 16);
		strcat(transmitBuffer, stringBufferTmp);

		if (i == (NUMBER_OF_STORED_ERRORS - 1))
		{
			strcat(transmitBuffer, "\r\n");
		}
		else
		{
			strcat(transmitBuffer, ", 0x");
		}
	}/* for (int i = 0; i < NUMBER_OF_STORED_ERRORS; i++) */
}/* void generateStatusString(uint8_t* transmitBuffer) */

/*****************************************************************************************
* generateEventParametersString() - function generate string for at command
* AT_EVENT_PARAMETERS. Function generate string with parameters taken from
* FootControllerStructure.sensorActivationParametersTable and one constant value
* (MINIMAL_LASER_SENSOR_RANGE_VALUE). String inform user about parameters of activation
* sensor.
*
* Parameters:
* @transmitBuffer: pointer to string buffer which will be used to construct string for
*  send.
*
*****************************************************************************************/
void generateEventParametersString(uint8_t* transmitBuffer)
{
	uint8_t stringBufferTmp[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

	strcpy(transmitBuffer, " - Key code: ");
	itoa(FootControllerStructure.sensorActivationParametersTable[0].keyCode, stringBufferTmp, 10);
	strcat(transmitBuffer, stringBufferTmp);

	strcat(transmitBuffer, " , event type is: ");

	if (FootControllerStructure.sensorActivationParametersTable[0].keyMode == PRESS_KEY)
	{
		strcat(transmitBuffer, "press key");
	}
	else if (FootControllerStructure.sensorActivationParametersTable[0].keyMode == HOLD_KEY)
	{
		strcat(transmitBuffer, "hold key");
	}
	else
	{
		strcat(transmitBuffer, "unknown");
	}

	strcat(transmitBuffer, " , laser range threshold: ");

	itoa(FootControllerStructure.sensorActivationParametersTable[0].laserRangeThreshold, stringBufferTmp, 10);
	strcat(transmitBuffer, stringBufferTmp);

	strcat(transmitBuffer, " , offset value: ");
	itoa(MINIMAL_LASER_SENSOR_RANGE_VALUE, stringBufferTmp, 10);
	strcat(transmitBuffer, stringBufferTmp);

	strcat(transmitBuffer, "\r\n");
}

/*****************************************************************************************
* parseConfigureSensorCommandParameters() - function used to parse input parameters for
* AT_SENSOR_ at command. Return flag depend from correctness of parameters set as argument
* for at command. During validation variable from at command argument is assigned to
* pointer to temporary buffer.
*
* Parameters:
* @verifiedParameters: part of string from AT_SENSOR_ at command started after "AT_SENSOR_"
* @sensorActivationParametersPointer: pointer to temporary buffer which will be used as
*  temporary memory during command serialization. If all parameters will be ok then data
*  from this buffer will be assign otherwire it will be dropped.
* @sensorNumber: pointer to variable which hold sensor number. This value will be assigned
*  during string serialization.
*
* Return: Return true if string is correct otherwise return error. Return true not mean
* that value will be assigned to FootControllerStructure.sensorActivationParametersTable
* becouse after correct serialization some data can be incorrect like sensor number out
* of range.
*****************************************************************************************/
bool parseConfigureSensorCommandParameters(uint8_t *verifiedParameters, SensorActivationParametersType *sensorActivationParametersPointer, uint8_t *sensorNumber)
{
	//verify parameters
	uint8_t *stringParametersPointerTmp = verifiedParameters;

	if (stringParametersPointerTmp != NULL && strlen(stringParametersPointerTmp) == 1
		&& stringParametersPointerTmp[0] >= 48 && stringParametersPointerTmp[0] <= 57)
	{
		*sensorNumber = (stringParametersPointerTmp[0] - 48);
	}
	else
	{
		return false;
	}

	stringParametersPointerTmp = strtok(NULL, "_");

	if (stringParametersPointerTmp == NULL || strcmp(stringParametersPointerTmp, "MODE") != 0)
	{
		return false;
	}

	stringParametersPointerTmp = strtok(NULL, "_");

	if (stringParametersPointerTmp != NULL && strcmp(stringParametersPointerTmp, "HOLD") == 0)
	{
		sensorActivationParametersPointer->keyMode = HOLD_KEY;
	}
	else if (stringParametersPointerTmp != NULL && strcmp(stringParametersPointerTmp, "PRESS") == 0)
	{
		sensorActivationParametersPointer->keyMode = PRESS_KEY;
	}
	else
	{
		return false;
	}

	stringParametersPointerTmp = strtok(NULL, "_");

	if (stringParametersPointerTmp == NULL || strcmp(stringParametersPointerTmp, "KEYCODE") != 0)
	{
		return false;
	}

	stringParametersPointerTmp = strtok(NULL, "_");

	if (stringParametersPointerTmp != NULL && strlen(stringParametersPointerTmp) > 0 && strlen(stringParametersPointerTmp) <= 3
		&& onlyDigitsExistInString(stringParametersPointerTmp))
	{
		sensorActivationParametersPointer->keyCode = atoi(stringParametersPointerTmp);
	}
	else
	{
		return false;
	}

	stringParametersPointerTmp = strtok(NULL, "_");

	if (stringParametersPointerTmp == NULL || strcmp(stringParametersPointerTmp, "THRESHOLD") != 0)
	{
		return false;
	}

	stringParametersPointerTmp = strtok(NULL, "_");

	if (stringParametersPointerTmp != NULL && onlyDigitsExistInString(stringParametersPointerTmp))
	{
		sensorActivationParametersPointer->laserRangeThreshold = atoi(stringParametersPointerTmp);
	}
	else
	{
		return false;
	}

	return true;
}/* bool parseConfigureSensorCommandParameters(uint8_t *verifiedParameters, SensorActivationParametersType *sensorActivationParametersPointer, uint8_t *sensorNumber) */
