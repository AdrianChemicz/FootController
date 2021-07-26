/*******************************************************************************
Copyright � 2015, STMicroelectronics International N.V.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of STMicroelectronics nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
NON-INFRINGEMENT OF INTELLECTUAL PROPERTY RIGHTS ARE DISCLAIMED.
IN NO EVENT SHALL STMICROELECTRONICS INTERNATIONAL N.V. BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************/

/**
 * @file VL53L0X_i2c.c
 *
 * Copyright (C) 2014 ST MicroElectronics
 *
 * provide variable word size byte/Word/dword VL6180x register access via i2c
 *
 */
#include "vl53l0x_platform.h"
#include "vl53l0x_api.h"
#include "vl53l0x_i2c.h"
#include "I2C_Driver.h"

#define LOG_FUNCTION_START(fmt, ... )           _LOG_FUNCTION_START(TRACE_MODULE_PLATFORM, fmt, ##__VA_ARGS__)
#define LOG_FUNCTION_END(status, ... )          _LOG_FUNCTION_END(TRACE_MODULE_PLATFORM, status, ##__VA_ARGS__)
#define LOG_FUNCTION_END_FMT(status, fmt, ... ) _LOG_FUNCTION_END_FMT(TRACE_MODULE_PLATFORM, status, fmt, ##__VA_ARGS__)

/**
 * @def I2C_BUFFER_CONFIG
 *
 * @brief Configure Device register I2C access
 *
 * @li 0 : one GLOBAL buffer \n
 *   Use one global buffer of MAX_I2C_XFER_SIZE byte in data space \n
 *   This solution is not multi-Device compliant nor multi-thread cpu safe \n
 *   It can be the best option for small 8/16 bit MCU without stack and limited ram  (STM8s, 80C51 ...)
 *
 * @li 1 : ON_STACK/local \n
 *   Use local variable (on stack) buffer \n
 *   This solution is multi-thread with use of i2c resource lock or mutex see VL6180x_GetI2CAccess() \n
 *
 * @li 2 : User defined \n
 *    Per Device potentially dynamic allocated. Requires VL6180x_GetI2cBuffer() to be implemented.
 * @ingroup Configuration
 */


VL53L0X_Error VL53L0X_i2c_init(uint8_t portNumber)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;

	I2C_DriverInit(portNumber);
	
	//init also timer

	//connect CT16B1 to AHB
	LPC_SYSCTL->SYSAHBCLKCTRL |= (1<<8);

	//configure prescaller
	LPC_TIMER16_1->PR = 800;

	//this register will max value of counter
	LPC_TIMER16_1->MR[0] = TIMER_COUNTED_VALUE;

	//Stop on MR0(set MR0S bit)
	LPC_TIMER16_1->MCR = 4;

	//Reset Timer
	LPC_TIMER16_1->TCR = 0x2;

	return Status;
}

VL53L0X_Error VL53L0X_LockSequenceAccess(VL53L0X_DEV Dev){
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    return Status;
}

VL53L0X_Error VL53L0X_UnlockSequenceAccess(VL53L0X_DEV Dev){
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    return Status;
}

// the ranging_sensor_comms.dll will take care of the page selection
VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count){

    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

	Status = VL53L0X_write_multi(Dev->portNumber, Dev->I2cDevAddr, index, pdata, count);

    return Status;
}

// the ranging_sensor_comms.dll will take care of the page selection
VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count){
    //VL53L0X_I2C_USER_VAR
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

	Status = VL53L0X_read_multi(Dev->portNumber, Dev->I2cDevAddr, index, pdata, count);

    return Status;
}

VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV Dev, uint8_t index, uint8_t data){
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

	Status = VL53L0X_write_multi(Dev->portNumber, Dev->I2cDevAddr, index, &data, 1);

	return Status;
}

VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV Dev, uint8_t index, uint16_t data){

	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	uint8_t buffer[BYTES_PER_WORD];

	// Split 16-bit word into MS and LS uint8_t
	buffer[0] = (uint8_t)(data >> 8);
	buffer[1] = (uint8_t)(data & 0x00FF);

	Status = VL53L0X_write_multi(Dev->portNumber, Dev->I2cDevAddr, index, buffer, BYTES_PER_WORD);

    return Status;
}

VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t data){
    
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	uint8_t  buffer[BYTES_PER_DWORD];

	// Split 32-bit word into MS ... LS bytes
	buffer[0] = (uint8_t)(data >> 24);
	buffer[1] = (uint8_t)((data & 0x00FF0000) >> 16);
	buffer[2] = (uint8_t)((data & 0x0000FF00) >> 8);
	buffer[3] = (uint8_t)(data & 0x000000FF);

	Status = VL53L0X_write_multi(Dev->portNumber, Dev->I2cDevAddr, index, buffer, BYTES_PER_DWORD);

    return Status;
}

VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV Dev, uint8_t index, uint8_t AndData, uint8_t OrData){

	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    int32_t status_int;
    //uint8_t deviceAddress;
    uint8_t data;

    //deviceAddress = Dev->I2cDevAddr;

	status_int = VL53L0X_RdByte(Dev, index, &data);

    if (status_int != 0)
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;

    if (Status == VL53L0X_ERROR_NONE) {
        data = (data & AndData) | OrData;
		status_int = VL53L0X_WrByte(Dev, index, data);

        if (status_int != 0)
            Status = VL53L0X_ERROR_CONTROL_INTERFACE;
    }

    return Status;
}

VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV Dev, uint8_t index, uint8_t *data){
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

	Status = VL53L0X_read_multi(Dev->portNumber, Dev->I2cDevAddr, index, data, 1);

	return Status;
}

VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV Dev, uint8_t index, uint16_t *data){
    
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	uint8_t  buffer[BYTES_PER_WORD];

	Status = VL53L0X_read_multi(Dev->portNumber, Dev->I2cDevAddr, index, buffer, BYTES_PER_WORD);
	*data = ((uint16_t)buffer[0]<<8) + (uint16_t)buffer[1];
    return Status;
}

VL53L0X_Error  VL53L0X_RdDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t *data){
    
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	uint8_t  buffer[BYTES_PER_DWORD];

	Status = VL53L0X_read_multi(Dev->portNumber, Dev->I2cDevAddr, index, buffer, BYTES_PER_DWORD);
	*data = ((uint32_t)buffer[0] << 24) + ((uint32_t)buffer[1] << 16) + ((uint32_t)buffer[2] << 8) + (uint32_t)buffer[3];

    return Status;
}

VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev){
    
	VL53L0X_Error status = VL53L0X_ERROR_NONE;

	status = VL53L0X_delay();

	return status;
}
