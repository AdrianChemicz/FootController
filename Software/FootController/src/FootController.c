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

#include "USB.h"
#include <stdbool.h>
#include <string.h>
#include "Descriptors.h"
#include "chip.h"
#include "GPIO_Driver.h"
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"

#include "FootController.h"
#include "EEPROM_Driver.h"
#include "HW_Driver.h"
#include "StringGenerateParseModule.h"

/*****************************************************************************
*	String used by virtual serial USB in at command as responses
*****************************************************************************/
const char *helpString = "\r\nFoot controller serial port is used to configure device and diagnose that device work properly\r\n\
Configuration is performed using AT command. When serial port will be opened send button event will be blocked on 30s from serial port opening.\r\n\
Device support below AT command.\r\n\r\n\
AT_SENSOR_ - command used to configure mode of laser sensor to decide how and when press keyboard button. Configuration of button is also possible.\r\n\
Command expect parameters in order the same as order of described parameters.\r\n\r\n\
1. Sensor number - parameter describe about to which sensor number will be assigned key code, mode and threshold value. Value is stored in decimal format.\r\n\r\n\
Format of parameter:\r\n\
<Sensor number>\r\n\r\n\
2. Mode of pressing button - parameter decide how button will be pressed. Two options is possible:\r\n\
PRESS - when event from laser sensor occur button will be pressed only one time even if event will be all time present.\r\n\
HOLD - when event from laser sensor occur button will be pressed and will be present to moment when laser sensor detect that event disappear.\r\n\
Format of parameter:\r\n\
MODE_PRESS\r\n\r\n\
3. Key code - parameter decide which key will be pressed when event occur. Value is stored in decimal format.\r\n\
Format of parameter:\r\n\
KEYCODE_6\r\n\r\n\
Few key code example:\r\n\
17 - N\r\n\
6 - C\r\n\
20 - Q\r\n\r\n\
4. Threshold - parameter describe distance how far foot must be move from laser sensor to occur event. Distance is stored in milimiters and in decimal formats.\r\n\r\n\
Format of parameter:\r\n\
THRESHOLD_20\r\n\r\n\
Example complete AT command formats:\r\n\
AT_SENSOR_0_MODE_PRESS_KEYCODE_6_THRESHOLD_20\r\n\
AT_SENSOR_0_MODE_HOLD_KEYCODE_17_THRESHOLD_20\r\n\r\n\
AT_GET_RANGE_COUNT_ - command used to check range measured by laser sensor. Data displayed by this command is as raw milimeter without any preprocesing.\r\n\
Command need only one parameter which mean number of get range samples. Number of samples must be set in decimal format.\r\n\r\n\
Example complete AT command formats:\r\n\
AT_GET_RANGE_COUNT_99\r\n\r\n\
AT_STATUS - command used to verify that device work correctly.\r\n\
Command return information about device current status, number of errors last errors code.\r\n\
Status and error code equal 0 mean no errors\r\n\r\n\
Example complete AT command format:\r\n\
AT_STATUS\r\n\r\n\
AT_EVENT_SUSPEND - command used to block button event send by device. This command is active to moment when serial port will be closed.\r\n\
Example complete AT command format:\r\n\
AT_EVENT_SUSPEND\r\n\r\n\
AT_EVENT_PARAMETERS - command used to display parameters used by sensors. Command will display important information like:\r\n\
key mode, key code, threshold need for cause key event, offset value(value used to compare with threshold is - offset value + raw laser sensor range)\r\n";

const char *invalidParametersString = " - invalid parameters\r\n";
const char *correctParametersString = " - correct parameters\r\n";
const char *invalidCommandString = " - invalid command\r\n";
const char *errorProgramExecutionString = " - error in code execution\r\n";

/*****************************************************************************
*	Data for virtual serial port to control method of responses for at
*	commmand.
*****************************************************************************/
uint8_t receiveTransmitCdcBuffer[CDC_TXRX_EPSIZE];
uint8_t stringCdcTransmitBuffer[CDC_TXRX_EPSIZE*4];
CDC_MessageStructureType CDC_MessageStructure;

//global structure available in many modules. It is provided by FootController.h file
FootControllerStructureType FootControllerStructure;

static uint8_t PrevKeyboardHIDReportBuffer[sizeof(USB_KeyboardReport_Data_t)];

/*****************************************************************************
*	Structures used by NXP USB library. Structure is pass as reference for
*	NXP USB library functions. In below code was presenets structure for
*	HID and CDC interfaces.
*****************************************************************************/
//this structure is availiable externaly - is provided by FootController.h file
USB_ClassInfo_HID_Device_t Keyboard_HID_Interface = {
	.Config = {
		.InterfaceNumber              = 0,

		.ReportINEndpointNumber       = KEYBOARD_EPNUM,
		.ReportINEndpointSize         = KEYBOARD_EPSIZE,
		.ReportINEndpointDoubleBank   = false,

		.PrevReportINBuffer           = PrevKeyboardHIDReportBuffer,
		.PrevReportINBufferSize       = sizeof(PrevKeyboardHIDReportBuffer),
		.PortNumber             = 0,
	},
};

USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface = {
	.Config = {
		.ControlInterfaceNumber         = 1,

		.DataINEndpointNumber           = CDC_TX_EPNUM,
		.DataINEndpointSize             = CDC_TXRX_EPSIZE,
		.DataINEndpointDoubleBank       = false,

		.DataOUTEndpointNumber          = CDC_RX_EPNUM,
		.DataOUTEndpointSize            = CDC_TXRX_EPSIZE,
		.DataOUTEndpointDoubleBank      = false,

		.NotificationEndpointNumber     = CDC_NOTIFICATION_EPNUM,
		.NotificationEndpointSize       = CDC_NOTIFICATION_EPSIZE,
		.NotificationEndpointDoubleBank = false,
		.PortNumber             = 0,
	},
};

/*****************************************************************************************
* ProcessCdc() - main task of this function is process virtual serial USB request send by
* PC. In this part of code request is checked that it is handle by device and parameters
* added to request is correct. Request send to device use at command format. After
* validation request function also start appropriate process of command. Function set field
* in FootControllerStructure and later those data is used in other functions to generate
* data for response or decide how device will work. State how execution should be performed
* is stored in structure which is pass to function via structure pointer set as parameter.
*
* Parameters:
* @CDC_MessageStructurePointer: pointer to structure which contain parameters with state
* of processed command
*****************************************************************************************/
static void ProcessCdc(CDC_MessageStructureType *CDC_MessageStructurePointer)
{
	uint16_t recvByteCountTmp = CDC_Device_BytesReceived(&VirtualSerial_CDC_Interface);

	if (recvByteCountTmp > 0)
	{
		FootControllerStructure.processedCommand = (uint8_t)UNKNOWN_COMMAND;
		FootControllerStructure.correctCommandParametersFlag = true;

		CDC_MessageStructurePointer->duringSendTransactionFlag = false;
		CDC_MessageStructurePointer->transactionSendWaitCounter = NUMBER_OF_SEND_WAIT_CYCLE;

		memset(receiveTransmitCdcBuffer , 0, CDC_TXRX_EPSIZE);
		for (int i = 0; i < recvByteCountTmp; i++)
		{
			receiveTransmitCdcBuffer[i] = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
		}

		//check that payload contain one of below strings on beginning
		//verify HELP COMMAND
		{
			if ((strstr(receiveTransmitCdcBuffer, "AT_HELP") != NULL && strstr(receiveTransmitCdcBuffer, "AT_HELP") == receiveTransmitCdcBuffer && receiveTransmitCdcBuffer[sizeof("AT_HELP")] == 0)
				|| (strstr(receiveTransmitCdcBuffer, "HELP") != NULL && strstr(receiveTransmitCdcBuffer, "HELP") == receiveTransmitCdcBuffer && receiveTransmitCdcBuffer[sizeof("HELP")] == 0))
			{
				FootControllerStructure.processedCommand = (uint8_t)HELP_COMAMND;
			}
		}
		//verify AT_STATUS
		{
			if (strstr(receiveTransmitCdcBuffer, "AT_STATUS") != NULL && strstr(receiveTransmitCdcBuffer, "AT_STATUS") == receiveTransmitCdcBuffer && receiveTransmitCdcBuffer[sizeof("AT_STATUS")] == 0)
			{
				FootControllerStructure.processedCommand = (uint8_t)STATUS_COMAMND;
			}
		}
		//verify AT_SENSOR_
		{
			if (strstr(receiveTransmitCdcBuffer, "AT_SENSOR_") != NULL && strstr(receiveTransmitCdcBuffer, "AT_SENSOR_") == receiveTransmitCdcBuffer)
			{
				SensorActivationParametersType sensorActivationParametersTmp;
				uint8_t sensorNumber;
				uint8_t copyOfCheckedBuffer[CDC_TXRX_EPSIZE];

				memcpy(copyOfCheckedBuffer, receiveTransmitCdcBuffer, CDC_TXRX_EPSIZE);

				FootControllerStructure.processedCommand = (uint8_t)CONFIGURE_SENSOR_COMAMND;

				if (parseConfigureSensorCommandParameters(strtok(&copyOfCheckedBuffer[sizeof("AT_SENSOR_") - 1], "_"), &sensorActivationParametersTmp, &sensorNumber))
				{
					if (sensorNumber > (MAX_SUPPORTED_SENSOR - 1))
					{
						FootControllerStructure.correctCommandParametersFlag = false;
					}
					else
					{
						FootControllerStructure.sensorActivationParametersTable[sensorNumber] = sensorActivationParametersTmp;

						SensorActivationParametersEepromStoreType sensorActivationParametersEepromContent;

						sensorActivationParametersEepromContent.sensorActivationParameters = sensorActivationParametersTmp;

						WriteSensorDataToEeprom(&sensorActivationParametersEepromContent, sensorNumber);
					}
				}
				else
				{
					FootControllerStructure.correctCommandParametersFlag = false;
				}
			}/* if (strstr(receiveTransmitCdcBuffer, "AT_SENSOR_") != NULL && strstr(receiveTransmitCdcBuffer, "AT_SENSOR_") == receiveTransmitCdcBuffer) */
		}
		//verify AT_GET_RANGE_COUNT_
		{
			if (strstr(receiveTransmitCdcBuffer, "AT_GET_RANGE_COUNT_") != NULL
				&& strstr(receiveTransmitCdcBuffer, "AT_GET_RANGE_COUNT_") == receiveTransmitCdcBuffer)
			{
				uint16_t numberOfRangeMeasurementTmp = 0;

				FootControllerStructure.processedCommand = (uint8_t)GET_RANGE_COMAMND;

				//verfy parameter
				numberOfRangeMeasurementTmp = atoi(&receiveTransmitCdcBuffer[sizeof("AT_GET_RANGE_COUNT_") - 1]);

				if(!onlyDigitsExistInString(&receiveTransmitCdcBuffer[sizeof("AT_GET_RANGE_COUNT_") - 1]))
				{
					FootControllerStructure.correctCommandParametersFlag = false;
				}

				if(numberOfRangeMeasurementTmp == 0 || numberOfRangeMeasurementTmp > MAX_SUPPORTED_LASER_MEASUREMENT)
				{
					FootControllerStructure.correctCommandParametersFlag = false;
				}

				FootControllerStructure.numberOfRangeMeasurement = numberOfRangeMeasurementTmp;

				FootControllerStructure.laserMeasurementIndex = 0;
				FootControllerStructure.laserMeasurementCurrentStoreIndex = 0;
				FootControllerStructure.laserMeasurementCurrentSendIndex = 0;
			}/* if (strstr(receiveTransmitCdcBuffer, "AT_GET_RANGE_COUNT_") != NULL
					&& strstr(receiveTransmitCdcBuffer, "AT_GET_RANGE_COUNT_") == receiveTransmitCdcBuffer) */
		}
		//verify AT_EVENT_SUSPEND
		{
			if (strstr(receiveTransmitCdcBuffer, "AT_EVENT_SUSPEND") != NULL && strstr(receiveTransmitCdcBuffer, "AT_EVENT_SUSPEND") == receiveTransmitCdcBuffer && receiveTransmitCdcBuffer[sizeof("AT_EVENT_SUSPEND")] == 0)
			{
				FootControllerStructure.processedCommand = (uint8_t)SUSPEND_EVENT_COMMAND;
			}
		}
		//verify AT_EVENT_PARAMETERS
		{
			if (strstr(receiveTransmitCdcBuffer, "AT_EVENT_PARAMETERS") != NULL && strstr(receiveTransmitCdcBuffer, "AT_EVENT_PARAMETERS") == receiveTransmitCdcBuffer && receiveTransmitCdcBuffer[sizeof("AT_EVENT_PARAMETERS")] == 0)
			{
				FootControllerStructure.processedCommand = (uint8_t)EVENT_PARAMETERS_COMMAND;
			}
		}

		CDC_MessageStructurePointer->duringSendTransactionFlag = true;
		CDC_MessageStructurePointer->stringOffset = 0;

		if (FootControllerStructure.correctCommandParametersFlag)
		{
			FootControllerStructure.stringFunctionGenerator = NULL;

			switch (FootControllerStructure.processedCommand)
			{
			case HELP_COMAMND:
				CDC_MessageStructurePointer->stringPointer = helpString;
				break;

			case STATUS_COMAMND:
				memset(stringCdcTransmitBuffer, 0, sizeof(stringCdcTransmitBuffer));
				CDC_MessageStructurePointer->stringPointer = stringCdcTransmitBuffer;
				generateStatusString(CDC_MessageStructurePointer->stringPointer);
				break;

			case CONFIGURE_SENSOR_COMAMND:
				CDC_MessageStructurePointer->stringPointer = correctParametersString;
				break;

			case GET_RANGE_COMAMND:
				memset(stringCdcTransmitBuffer, 0, sizeof(stringCdcTransmitBuffer));
				CDC_MessageStructurePointer->stringPointer = stringCdcTransmitBuffer;
				FootControllerStructure.stringFunctionGenerator = atGetRangeCountStringGenerator;
				break;

			case SUSPEND_EVENT_COMMAND:
				CDC_MessageStructurePointer->stringPointer = correctParametersString;
				FootControllerStructure.suspendSendEventByCdc = true;
				FootControllerStructure.suspendSerialCommandCounter = 0;
				FootControllerStructure.suspendSendEventPermanentlyWitOpenSerial = true;
				break;

			case EVENT_PARAMETERS_COMMAND:
				memset(stringCdcTransmitBuffer, 0, sizeof(stringCdcTransmitBuffer));
				CDC_MessageStructurePointer->stringPointer = stringCdcTransmitBuffer;
				generateEventParametersString(CDC_MessageStructurePointer->stringPointer);
				break;

			case UNKNOWN_COMMAND:
				CDC_MessageStructurePointer->stringPointer = invalidCommandString;
				break;

			default:
				CDC_MessageStructurePointer->stringPointer = errorProgramExecutionString;
				break;

			}/* switch (FootControllerStructure.processedCommand) */
		}
		else
		{
			CDC_MessageStructurePointer->stringPointer = invalidParametersString;
		}
	}/* if (recvByteCountTmp > 0) */

	if (CDC_MessageStructurePointer->duringSendTransactionFlag)
	{
		if (CDC_MessageStructurePointer->transactionSendWaitCounter >= NUMBER_OF_SEND_WAIT_CYCLE)
		{
			uint16_t bufferOffsetTmp = 0;
			uint16_t numberOfFreeBytesInBufferTmp = 0;
			uint16_t partOfStringLength = 0;

			if (CDC_MessageStructurePointer->stringOffset == 0)
			{
				bufferOffsetTmp = recvByteCountTmp;
			}

			numberOfFreeBytesInBufferTmp = CDC_TXRX_EPSIZE - bufferOffsetTmp;
			partOfStringLength = strlen(&CDC_MessageStructurePointer->stringPointer[CDC_MessageStructurePointer->stringOffset]);

			//case when last part of string is send
			if (numberOfFreeBytesInBufferTmp > partOfStringLength)
			{
				if ((numberOfFreeBytesInBufferTmp != CDC_TXRX_EPSIZE) || (partOfStringLength > 0))
				{
					memcpy(&receiveTransmitCdcBuffer[bufferOffsetTmp], &CDC_MessageStructurePointer->stringPointer[CDC_MessageStructurePointer->stringOffset], partOfStringLength);
					CDC_Device_SendData(&VirtualSerial_CDC_Interface, receiveTransmitCdcBuffer, bufferOffsetTmp + partOfStringLength);
				}

				CDC_MessageStructurePointer->stringOffset = 0;

				if (FootControllerStructure.stringFunctionGenerator == NULL || FootControllerStructure.stringFunctionGenerator(CDC_MessageStructurePointer->stringPointer))
					CDC_MessageStructurePointer->duringSendTransactionFlag = false;
			}
			else
			{
				memcpy(&receiveTransmitCdcBuffer[bufferOffsetTmp], &CDC_MessageStructurePointer->stringPointer[CDC_MessageStructurePointer->stringOffset], numberOfFreeBytesInBufferTmp);
				CDC_MessageStructurePointer->stringOffset += numberOfFreeBytesInBufferTmp;
				CDC_Device_SendData(&VirtualSerial_CDC_Interface, receiveTransmitCdcBuffer, CDC_TXRX_EPSIZE);
			}

			CDC_MessageStructurePointer->transactionSendWaitCounter = 0;
		}
		else
		{
			CDC_MessageStructurePointer->transactionSendWaitCounter++;
		}
	}/* if(CDC_MessageStructurePointer->duringSendTransactionFlag) */
}/* static void ProcessCdc(CDC_MessageStructureType *CDC_MessageStructurePointer) */

/*****************************************************************************************
* TIMER16_0_IRQHandler() - function called as intterupt generated by uC HW TIMER16_0.
* Event is called every 83.5 us. According to measure with logic analyzer and GPIO change
* state when interrupt occur time can drift in small range like +/-0.5us.
*****************************************************************************************/
void TIMER16_0_IRQHandler(void)
{
	USB_USBTask(Keyboard_HID_Interface.Config.PortNumber, USB_MODE_Device);
	ProcessCdc(&CDC_MessageStructure);

	HID_Device_USBTask(&Keyboard_HID_Interface);
	USB_USBTask(Keyboard_HID_Interface.Config.PortNumber, USB_MODE_Device);

	if (FootControllerStructure.clearErrorCounter >= NUMBER_OF_CYCLE_TO_CLEAR_ERROR)
	{
		FootControllerStructure.deviceStatus = NO_FAULT_ERROR_CODE;
		FootControllerStructure.clearErrorCounter = 0;
	}
	else
	{
		FootControllerStructure.clearErrorCounter++;
	}

	if (FootControllerStructure.suspendSendEventByCdc)
	{
		if (FootControllerStructure.suspendSendEventPermanentlyWitOpenSerial == false)
		{
			if (FootControllerStructure.suspendSerialCommandCounter >= 30*NUMBER_OF_CYCLE_TO_REACH_1S_BY_TIMER)
			{
				FootControllerStructure.suspendSendEventByCdc = false;
			}
			else
			{
				FootControllerStructure.suspendSerialCommandCounter++;
			}
		}
	}/* if (FootControllerStructure.suspendSendEventByCdc) */

	LPC_TIMER16_0->IR = 1;
}/* void TIMER16_0_IRQHandler(void) */

/*****************************************************************************************
* main() - called after HW initialization. In this function data in FootControllerStructure
* is initialized. HW like USB port, GPIO and timer which check USB port are initialized.
* Data from EEPROM is loaded and verfied. Laser sensor is initialized and in never ending
* loop laser measurement are read.
*
* Return: nothing - wasn't call
*****************************************************************************************/
int main(void)
{
	SensorActivationParametersEepromStoreType sensorActivationParametersEepromContent;
	VL53L0X_Dev_t SensorDevice;

	//initialization of FootControllerStructure
	FootControllerStructure.laserSensorEventTable[0].previousLaserMeasurement = 0xFFFF;

	FootControllerStructure.stringFunctionGenerator = NULL;

	for (int i = 0; i < NUMBER_OF_STORED_ERRORS; i++)
	{
		if (i == (NUMBER_OF_STORED_ERRORS - 1))
		{
			//last element point to first element
			FootControllerStructure.lastErrorCodeTable[i][INDEX_CODE_POSITION] = 0;
		}
		else
		{
			FootControllerStructure.lastErrorCodeTable[i][INDEX_CODE_POSITION] = (i + 1);
		}

		FootControllerStructure.lastErrorCodeTable[i][ERROR_CODE_POSITION] = 0;
	}/* for (int i = 0; i < NUMBER_OF_STORED_ERRORS; i++) */

	FootControllerStructure.pointerToCurrentErrorCode = FootControllerStructure.lastErrorCodeTable[0];

	SetupHardware();

	USB_Init(Keyboard_HID_Interface.Config.PortNumber, USB_MODE_Device);

	if (ReadSensorDataFromEeprom(&sensorActivationParametersEepromContent, 0))
	{
		FootControllerStructure.sensorActivationParametersTable[0] = sensorActivationParametersEepromContent.sensorActivationParameters;
	}
	else
	{
		FootControllerStructure.sensorActivationParametersTable[0].keyCode = HID_KEYBOARD_SC_C;
		FootControllerStructure.sensorActivationParametersTable[0].keyMode = PRESS_KEY;
		FootControllerStructure.sensorActivationParametersTable[0].laserRangeThreshold = 20;
	}

	SensorDevice.portNumber = 0;
	SensorDevice.I2cDevAddr = 0x52;

	VL53L0X_i2c_init(SensorDevice.portNumber);
	VL53L0X_DataInit(&SensorDevice);

	FootControllerStructure.deviceStatus = INITIALIZATION_ERROR_CODE;

	for ( ; ; )
	{
		FootControllerStructure.deviceStatus = LaserProcess(&SensorDevice);

		FootControllerStructure.pointerToCurrentErrorCode[ERROR_CODE_POSITION] = FootControllerStructure.deviceStatus;
		FootControllerStructure.pointerToCurrentErrorCode = FootControllerStructure.lastErrorCodeTable[FootControllerStructure.pointerToCurrentErrorCode[INDEX_CODE_POSITION]];
		FootControllerStructure.errorCounter++;
	}
}/* int main(void) */

/*****************************************************************************************
* EVENT_USB_Device_Connect() - called by NXP USB library more information is in library
* header file.
*****************************************************************************************/
void EVENT_USB_Device_Connect(void)
{

}

/*****************************************************************************************
* EVENT_USB_Device_Disconnect() - called by NXP USB library more information is in library
* header file.
*****************************************************************************************/
void EVENT_USB_Device_Disconnect(void)
{

}

/*****************************************************************************************
* EVENT_USB_Device_ConfigurationChanged() - called by NXP USB library. Configure endpoints
* for HID device, virtual serial port and enable start of frame events(events triggered
* by SOF frame on USB port send cyclically by HOST).
*****************************************************************************************/
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	ConfigSuccess &= HID_Device_ConfigureEndpoints(&Keyboard_HID_Interface);
	ConfigSuccess &= CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);

	USB_Device_EnableSOFEvents();
}

/*****************************************************************************************
* EVENT_USB_Device_ControlRequest() - called by NXP USB library more information is in
* library header file.
*****************************************************************************************/
void EVENT_USB_Device_ControlRequest(void)
{
	HID_Device_ProcessControlRequest(&Keyboard_HID_Interface);
	CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
}

/*****************************************************************************************
* EVENT_USB_Device_Disconnect() - called by NXP USB library when start of frame occur on
* bus.
*****************************************************************************************/
void EVENT_USB_Device_StartOfFrame(void)
{
	HID_Device_MillisecondElapsed(&Keyboard_HID_Interface);
}

/*****************************************************************************************
* CALLBACK_HID_Device_CreateHIDReport() - called by NXP USB library more information is in
* library header file. In this function is created HID report for host with information
* that keyboard button was pressed.
*****************************************************************************************/
bool CALLBACK_HID_Device_CreateHIDReport(USB_ClassInfo_HID_Device_t *const HIDInterfaceInfo, uint8_t *const ReportID,
										 const uint8_t ReportType, void *ReportData, uint16_t *const ReportSize)
{
	USB_KeyboardReport_Data_t *KeyboardReport = (USB_KeyboardReport_Data_t *) ReportData;

	for (int i = 0, keyPressedNumbers = 0; i < MAX_SUPPORTED_SENSOR; i++)
	{
		if ((FootControllerStructure.laserSensorEventTable[i].thresholdState == false)
			&& FootControllerStructure.laserSensorEventTable[i].eventKeyWasSend
			&& FootControllerStructure.laserSensorEventTable[i].eventKeyState)
		{
			FootControllerStructure.laserSensorEventTable[i].eventKeyState = false;
		}

		if((GPIO_GetState(BLOCK_SEND_EVENT_PORT, BLOCK_SEND_EVENT_PIN) == true)
			&& (FootControllerStructure.laserSensorEventTable[i].eventKeyState == true)
			&& (FootControllerStructure.suspendSendEventByCdc == false))
		{
			KeyboardReport->KeyCode[keyPressedNumbers++] = FootControllerStructure.sensorActivationParametersTable[i].keyCode;
			FootControllerStructure.laserSensorEventTable[i].eventKeyWasSend = true;
		}
	}

	*ReportSize = sizeof(USB_KeyboardReport_Data_t);

	return true;
}

/*****************************************************************************************
* CALLBACK_HID_Device_ProcessHIDReport() - called by NXP USB library more information is
* in library header file.
*****************************************************************************************/
void CALLBACK_HID_Device_ProcessHIDReport(USB_ClassInfo_HID_Device_t *const HIDInterfaceInfo,
										  const uint8_t ReportID,
										  const uint8_t ReportType,
										  const void *ReportData,
										  const uint16_t ReportSize)
{

}

/*****************************************************************************************
* EVENT_CDC_Device_LineEncodingChanged() - called by NXP USB library more information is
* in library header file.
*****************************************************************************************/
void EVENT_CDC_Device_LineEncodingChanged(USB_ClassInfo_CDC_Device_t *const CDCInterfaceInfo)
{

}

/*****************************************************************************************
* EVENT_CDC_Device_ControLineStateChanged() - called by NXP USB library more information is
* in library header file. Function indicate when virtual serial port was opened or closed.
* This event is used to indicate on red LED that serial port is open or close. Call this
* event amd indication that port was opened cause suppres on 30s HID key event.
*****************************************************************************************/
void EVENT_CDC_Device_ControLineStateChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo)
{
	//port was opened
	if (CDCInterfaceInfo->State.ControlLineStates.HostToDevice & CDC_CONTROL_LINE_OUT_DTR)
	{
		GPIO_SetState(CDC_STATE_INDICATOR_PORT, CDC_STATE_INDICATOR_PIN, true);
		FootControllerStructure.suspendSendEventByCdc = true;
	}
	else//port was closed
	{
		GPIO_SetState(CDC_STATE_INDICATOR_PORT, CDC_STATE_INDICATOR_PIN, false);

		FootControllerStructure.suspendSendEventByCdc = false;
	}

	FootControllerStructure.suspendSerialCommandCounter = 0;
	FootControllerStructure.suspendSendEventPermanentlyWitOpenSerial = false;
}
