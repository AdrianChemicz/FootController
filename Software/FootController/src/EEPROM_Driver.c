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

#include "EEPROM_Driver.h"
#include <stddef.h>
#include "chip.h"
#include "iap.h"

/*****************************************************************************************
* calculateChecksum() - calculate checksum compatible with protocol used by IP layer
* in network communication. Checksum is calculated according to input parameters. Checksum
* is calculated as 16 bit format and it is returned by function.
*
* Parameters:
* @dataPointer: pointer to begining of data used to calculate checksum
* @numOfBytes: number of bytes used to calculate checksum
*
* Return: checksum value.
*****************************************************************************************/
static uint16_t calculateChecksum(uint8_t *dataPointer, uint16_t numOfBytes)
{
	uint16_t checksum = 0;

	for (uint16_t i = 0; i < numOfBytes; i += 2)
	{
		if ((i + 1) == numOfBytes)
		{

		}
		else
		{
			checksum += (dataPointer[i] << 8) | dataPointer[i + 1];
		}
	}

	checksum = (checksum & 0xFFFF) + (checksum >> 16);

	return ~checksum;
}

/*****************************************************************************************
* ReadSensorDataFromEeprom() - function read data from EEPROM memory to memory pointed by
* sensorActivationParametersPointer pointer. This pointer is used as input parameter. Data
* copied from EEPROM memory contained information about which key must be send when laser
* sensor event will be passed, value of threshold to decide that event occur and type of
* event(single press or hold). Function support multiply activation parameters structure
* load from EEPROM. Chose structure is performed via second parameter of function -
* sensorNumber which mean index of sensor number.
*
* Parameters:
* @sensorActivationParametersPointer: pointer to structure with sensor activation parameter
* @sensorNumber: sonsor number value used as index in EEPROM memory
*
* Return: Return true if checksum in loaded structure from EEPPROM contain correct
* checksum calculated from data otherwise false.
*****************************************************************************************/
bool ReadSensorDataFromEeprom(SensorActivationParametersEepromStoreType *sensorActivationParametersPointer, uint8_t sensorNumber)
{
	uint16_t sensorChecksum = 0;
	bool onlyZeroInStructure = true;

	__disable_irq();

	Chip_EEPROM_Read(LASER_SENSOR_CONFIG_BEGINNING + (sensorNumber * sizeof(SensorActivationParametersEepromStoreType))
			, (uint8_t*)sensorActivationParametersPointer, sizeof(SensorActivationParametersEepromStoreType));

	__enable_irq();

	sensorChecksum = calculateChecksum((uint8_t*)sensorActivationParametersPointer,
				offsetof(SensorActivationParametersEepromStoreType, checksum));

	for (int i = 0; i < sizeof(SensorActivationParametersType); i++)
	{
		if (((uint8_t*)sensorActivationParametersPointer)[i] != 0)
		{
			onlyZeroInStructure = false;
		}
	}

	if (onlyZeroInStructure)
	{
		return false;
	}

	if (sensorChecksum == sensorActivationParametersPointer->checksum)
	{
		return true;
	}
	else
	{
		return false;
	}
}

/*****************************************************************************************
* WriteSensorDataToEeprom() - function write data from memory pointed by
* sensorActivationParametersPointer pointer to EEPROM memory. This pointer is used as
* input parameter. Data copied to EEPROM memory contained information about which key must
* be send when laser sensor event will be passed, value of threshold to decide that event
* occur and type of event(single press or hold). Function support multiply activation
* parameters structure store in EEPROM. Chose structure is performed via second parameter
* of function - sensorNumber which mean index of sensor number.
*
* Parameters:
* @sensorActivationParametersPointer: pointer to structure with sensor activation parameter
* @sensorNumber: sonsor number value used as index in EEPROM memory
*
*****************************************************************************************/
void WriteSensorDataToEeprom(SensorActivationParametersEepromStoreType *sensorActivationParametersPointer, uint8_t sensorNumber)
{
	sensorActivationParametersPointer->counter = 0;
	sensorActivationParametersPointer->checksum = calculateChecksum((uint8_t*)sensorActivationParametersPointer,
			offsetof(SensorActivationParametersEepromStoreType, checksum));

	__disable_irq();

	Chip_EEPROM_Write(LASER_SENSOR_CONFIG_BEGINNING + (sensorNumber * sizeof(SensorActivationParametersEepromStoreType))
			, (uint8_t*)sensorActivationParametersPointer, sizeof(SensorActivationParametersEepromStoreType));

	__enable_irq();
}
