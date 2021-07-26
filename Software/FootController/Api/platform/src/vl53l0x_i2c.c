#include "vl53l0x_i2c.h"
#include "I2C_Driver.h"


VL53L0X_Error VL53L0X_write_multi(uint8_t portNumber, uint8_t address, uint8_t index, uint8_t  *pdata, int32_t count)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;

	uint8_t temporaryBuffer[READ_WRITE_BUFFER_SIZE + 1];

	temporaryBuffer[0] = index;

	memcpy(&temporaryBuffer[1], pdata, count);

	I2C_SendReadData(portNumber, address, temporaryBuffer, count + 1, 0);

	volatile uint8_t statusFlag = I2C_CheckStatus(portNumber);

	for (; I2C_CheckStatus(portNumber) != I2C_WAITING_FOR_DATA;)
	{

	}

	return Status;
}

VL53L0X_Error VL53L0X_read_multi(uint8_t portNumber, uint8_t address, uint8_t index, uint8_t *pdata, int32_t count)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;

	I2C_SendReadData(portNumber, address, &index, 1, count);

	for (; I2C_CheckStatus(portNumber) != I2C_WAITING_FOR_DATA;)
	{

	}

	memcpy(pdata, I2C_PointerToInternalReadBuffer(portNumber), count);

	return Status;
}

VL53L0X_Error VL53L0X_delay(void)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	volatile uint16_t timerValueTmp = 0;
	LPC_TIMER16_1->TCR = 1;

	for(bool endFlag = false; endFlag == false; )
	{
		timerValueTmp = LPC_TIMER16_1->TC;
		if(LPC_TIMER16_1->TC >= LPC_TIMER16_1->MR[0])
		{
			endFlag = true;
		}
	}

	//Reset Timer
	LPC_TIMER16_1->TCR = 0x2;

	return Status;
}
