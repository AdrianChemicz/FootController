/* No license header added because some part of code base on NXP code(part of code
 * responsible for clock settings) other base on ST VL53L0X API example. */
#include "HW_Driver.h"
#include "chip.h"
#include "GPIO_Driver.h"
#include "FootController.h"

/*****************************************************************************************
* setUsbTimer() - configure timer which will in interrupt handle USB port. After call this
* code TIMER16_0_IRQHandler function will cal cyclically every 83.5us.
*****************************************************************************************/
static void setUsbTimer(void)
{
	/**********************************
	*	init timer
	***********************************/
	//connect CT16B0 to AHB
	LPC_SYSCTL->SYSAHBCLKCTRL |= (1<<7);

	//83.5us
	//configure prescaller
	LPC_TIMER16_0->PR = 800;

	//this register will max value of counter
	LPC_TIMER16_0->MR[0] = 4;

	//Reset on MR0 and generate interrupt(set MR0R bit and MR0I)
	LPC_TIMER16_0->MCR = 3;

	//Reset Timer
	LPC_TIMER16_0->TCR = 0x2;

	//Enable timer (set CEN bit)
	LPC_TIMER16_0->TCR = 1;

	//allow for interruptions from timer
	NVIC_EnableIRQ(TIMER_16_0_IRQn);
}

/*****************************************************************************************
* systemSetupClocking() - configure clock for microcontroller. This function change source
* of clock for microcontroller from IRC to PLL which use quarz. In this function also
* USB PLL is configured.
*****************************************************************************************/
static void systemSetupClocking(void)
{
	volatile int i;

	/* Powerup main oscillator */
	Chip_SYSCTL_PowerUp(SYSCTL_POWERDOWN_SYSOSC_PD);

	/* Wait 200us for OSC to be stablized, no status
	   indication, dummy wait. */
	for (i = 0; i < 0x100; i++) {}

	/* Set system PLL input to main oscillator */
	Chip_Clock_SetSystemPLLSource(SYSCTL_PLLCLKSRC_MAINOSC);

	/* Power down PLL to change the PLL divider ratio */
	Chip_SYSCTL_PowerDown(SYSCTL_POWERDOWN_SYSPLL_PD);

	/* Setup PLL for main oscillator rate (FCLKIN = 12MHz) * 4 = 48MHz
	   MSEL = 3 (this is pre-decremented), PSEL = 1 (for P = 2)
	   FCLKOUT = FCLKIN * (MSEL + 1) = 12MHz * 4 = 48MHz
	   FCCO = FCLKOUT * 2 * P = 48MHz * 2 * 2 = 192MHz (within FCCO range) */
	Chip_Clock_SetupSystemPLL(3, 1);

	/* Powerup system PLL */
	Chip_SYSCTL_PowerUp(SYSCTL_POWERDOWN_SYSPLL_PD);

	/* Wait for PLL to lock */
	while (!Chip_Clock_IsSystemPLLLocked()) {}

	/* Set system clock divider to 1 */
	Chip_Clock_SetSysClockDiv(1);

	/* Setup FLASH access to 3 clocks */
	Chip_FMC_SetFLASHAccess(FLASHTIM_50MHZ_CPU);

	/* Set main clock source to the system PLL. This will drive 48MHz
	   for the main clock and 48MHz for the system clock */
	Chip_Clock_SetMainClockSource(SYSCTL_MAINCLKSRC_PLLOUT);

	/* Set USB PLL input to main oscillator */
	Chip_Clock_SetUSBPLLSource(SYSCTL_PLLCLKSRC_MAINOSC);
	/* Setup USB PLL  (FCLKIN = 12MHz) * 4 = 48MHz
	   MSEL = 3 (this is pre-decremented), PSEL = 1 (for P = 2)
	   FCLKOUT = FCLKIN * (MSEL + 1) = 12MHz * 4 = 48MHz
	   FCCO = FCLKOUT * 2 * P = 48MHz * 2 * 2 = 192MHz (within FCCO range) */
	Chip_Clock_SetupUSBPLL(3, 1);

	/* Powerup USB PLL */
	Chip_SYSCTL_PowerUp(SYSCTL_POWERDOWN_USBPLL_PD);

	/* Wait for PLL to lock */
	while (!Chip_Clock_IsUSBPLLLocked()) {}
}

/*****************************************************************************************
* LaserProcess() - only one function responsible for operate laser sensor. Before call
* this function is required just I2C init and data init which mean that structure used
* by this function as argument will be initialized. Data initialization is performed by
* VL53L0X_DataInit function. This function before init sensor process prepare reset via
* GPIO connected to NPN transitor which gurantee that multiply LaserProcess call don't
* require device power cycle. This is blocking function and in correct device operation
* this function never stop execution. Return value by this function mean that error occur
* and returned value mean error code. This function access to FootControllerStructure
* and according to laser range measurement and other settings decide that key should be
* pressed.
*
* Parameters:
* @pMyDevice: pointer to structure which hold information about I2C sensor laser address
* and parameters for correct working VL53L0X API.
*
* Return: Return error code consist of two bytes first byte is status of error second
* bytes is step when error occur.
*****************************************************************************************/
uint16_t LaserProcess(VL53L0X_Dev_t *pMyDevice)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	VL53L0X_RangingMeasurementData_t    RangingMeasurementData;
	uint32_t refSpadCount;
	uint8_t isApertureSpads;
	uint8_t VhvSettings;
	uint8_t PhaseCal;
	uint16_t returnCode = 0;

	//set low state on NPN transistor to force reset on laser sensor
	GPIO_SetState(FIRST_SENSOR_CONTROL_LINE_PORT, FIRST_SENSOR_CONTROL_LINE_PIN, false);

	VL53L0X_PollingDelay(pMyDevice);

	//set high state on NPN transistor to run laser sensor
	GPIO_SetState(FIRST_SENSOR_CONTROL_LINE_PORT, FIRST_SENSOR_CONTROL_LINE_PIN, true);

	VL53L0X_PollingDelay(pMyDevice);

	if (Status == VL53L0X_ERROR_NONE)
	{
		Status = VL53L0X_StaticInit(pMyDevice); // Device Initialization
	}

	if (Status == VL53L0X_ERROR_NONE)
	{
		Status = VL53L0X_PerformRefCalibration(pMyDevice,
			&VhvSettings, &PhaseCal); // Device Initialization
	}
	else
	{
		returnCode = 1<<8|(uint8_t)Status;
	}

	if (Status == VL53L0X_ERROR_NONE)
	{
		Status = VL53L0X_PerformRefSpadManagement(pMyDevice,
			&refSpadCount, &isApertureSpads); // Device Initialization
	}
	else
	{
		returnCode = 2<<8|(uint8_t)Status;
	}

	if (Status == VL53L0X_ERROR_NONE)
	{
		Status = VL53L0X_SetDeviceMode(pMyDevice, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
	}
	else
	{
		returnCode = 3<<8|(uint8_t)Status;
	}

	if (Status == VL53L0X_ERROR_NONE)
	{
		Status = VL53L0X_StartMeasurement(pMyDevice);
	}
	else
	{
		returnCode = 4<<8|(uint8_t)Status;
	}

	if(returnCode != 0)
	{
		return returnCode;
	}

	if (Status == VL53L0X_ERROR_NONE)
	{
		for ( ; ; )
		{
			for ( ; ; )
			{
				uint8_t dataStatus = 0;
				Status = VL53L0X_GetMeasurementDataReady(pMyDevice, &dataStatus);

				if ((dataStatus == 0x01) || Status != VL53L0X_ERROR_NONE)
				{
					break;
				}
			}

			Status = VL53L0X_GetRangingMeasurementData(pMyDevice, &RangingMeasurementData);

			VL53L0X_ClearInterruptMask(pMyDevice, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
			VL53L0X_PollingDelay(pMyDevice);

			//case which check that measurement isn't performed first time then previous value will be not specified
			if (FootControllerStructure.laserSensorEventTable[0].previousLaserMeasurement == 0xFFFF)
			{
				FootControllerStructure.laserSensorEventTable[0].previousLaserMeasurement = RangingMeasurementData.RangeMilliMeter;
			}

			if (FootControllerStructure.sensorActivationParametersTable[0].keyMode == PRESS_KEY)
			{
				if (RangingMeasurementData.RangeMilliMeter > MAX_SUPPORTED_RANGE_IN_MM)
				{
					FootControllerStructure.laserSensorEventTable[0].eventKeyState = false;
					FootControllerStructure.laserSensorEventTable[0].clearLaserOffsetCounter = 0;
					FootControllerStructure.laserSensorEventTable[0].laserRangeOffset = 0;
					FootControllerStructure.laserSensorEventTable[0].distanceAssignedToRangeOffset = 0;
					FootControllerStructure.laserSensorEventTable[0].thresholdState = false;
				}
				else if (RangingMeasurementData.RangeMilliMeter > (FootControllerStructure.laserSensorEventTable[0].previousLaserMeasurement + LASER_JITTER_VALUE))
				{
					if(FootControllerStructure.laserSensorEventTable[0].distanceAssignedToRangeOffset == 0
							|| (FootControllerStructure.laserSensorEventTable[0].distanceAssignedToRangeOffset > RangingMeasurementData.RangeMilliMeter))
					{
						FootControllerStructure.laserSensorEventTable[0].distanceAssignedToRangeOffset = RangingMeasurementData.RangeMilliMeter;
					}

					FootControllerStructure.laserSensorEventTable[0].laserRangeOffset
					= RangingMeasurementData.RangeMilliMeter - FootControllerStructure.laserSensorEventTable[0].distanceAssignedToRangeOffset;

					if (FootControllerStructure.laserSensorEventTable[0].laserRangeOffset
							>= FootControllerStructure.sensorActivationParametersTable[0].laserRangeThreshold)
					{
						FootControllerStructure.laserSensorEventTable[0].thresholdState = true;
					}
				}
				else if (FootControllerStructure.laserSensorEventTable[0].thresholdState
						&& ((RangingMeasurementData.RangeMilliMeter + LASER_JITTER_SUSPEND_STATE_VALUE)
								> FootControllerStructure.laserSensorEventTable[0].previousLaserMeasurement))
				{

				}
				else if ((RangingMeasurementData.RangeMilliMeter + LASER_JITTER_VALUE)
						> FootControllerStructure.laserSensorEventTable[0].previousLaserMeasurement)
				{
					//if device too long is hold in one possition and this possition isn't move enough to raise event then clear offset value
					if (FootControllerStructure.laserSensorEventTable[0].thresholdState == false)
					{
						if (FootControllerStructure.laserSensorEventTable[0].clearLaserOffsetCounter > NUMBER_OF_COUNT_TO_CLEAR_LASER_OFFSET)
						{
							FootControllerStructure.laserSensorEventTable[0].clearLaserOffsetCounter = 0;
							FootControllerStructure.laserSensorEventTable[0].laserRangeOffset = 0;
							FootControllerStructure.laserSensorEventTable[0].distanceAssignedToRangeOffset = 0;
						}
					}
				}
				else
				{
					FootControllerStructure.laserSensorEventTable[0].laserRangeOffset = 0;
					FootControllerStructure.laserSensorEventTable[0].distanceAssignedToRangeOffset = 0;
					FootControllerStructure.laserSensorEventTable[0].clearLaserOffsetCounter = 0;

					if (FootControllerStructure.laserSensorEventTable[0].thresholdState == true
							&& FootControllerStructure.laserSensorEventTable[0].eventKeyWasSend == true)
					{
						FootControllerStructure.laserSensorEventTable[0].thresholdState = false;
						FootControllerStructure.laserSensorEventTable[0].eventKeyWasSend = false;
					}
				}
			}
			else
			{
				uint16_t laserRangeTmp = RangingMeasurementData.RangeMilliMeter;

				if (laserRangeTmp < MINIMAL_LASER_SENSOR_RANGE_VALUE)
				{
					laserRangeTmp = MINIMAL_LASER_SENSOR_RANGE_VALUE;
				}

				if ((RangingMeasurementData.RangeMilliMeter <= MAX_SUPPORTED_RANGE_IN_MM)
					&& ((laserRangeTmp - MINIMAL_LASER_SENSOR_RANGE_VALUE)
					> FootControllerStructure.sensorActivationParametersTable[0].laserRangeThreshold))
				{
					FootControllerStructure.laserSensorEventTable[0].thresholdState = true;
				}
				else
				{
					FootControllerStructure.laserSensorEventTable[0].thresholdState = false;
				}
			}

			/* Part of code necessary to disable key if event was send and device work in PRESS_KEY mode(send
			 event only one time even if threshold case is set) */
			if (FootControllerStructure.laserSensorEventTable[0].thresholdState)
			{
				if (FootControllerStructure.laserSensorEventTable[0].eventKeyWasSend
					&& (FootControllerStructure.sensorActivationParametersTable[0].keyMode == PRESS_KEY))
				{
					FootControllerStructure.laserSensorEventTable[0].eventKeyState = false;
				}
				else
				{
					FootControllerStructure.laserSensorEventTable[0].eventKeyState = true;
				}
			}

			//
			if ((FootControllerStructure.laserSensorEventTable[0].thresholdState == false)
				&& FootControllerStructure.laserSensorEventTable[0].eventKeyWasSend
				&& FootControllerStructure.laserSensorEventTable[0].eventKeyState)
			{
				FootControllerStructure.laserSensorEventTable[0].eventKeyState = false;
			}

			FootControllerStructure.laserSensorEventTable[0].previousLaserMeasurement = RangingMeasurementData.RangeMilliMeter;

			//Code responsible for gather sensor measurement necessary for command AT_GET_RANGE_MEASUREMENT
			if (FootControllerStructure.numberOfRangeMeasurement > 0)
			{
				FootControllerStructure.lockTableFlag = true;

				if (FootControllerStructure.laserMeasurementCurrentStoreIndex <= (LASER_MEASUREMENT_BUFFER_DEPH - 1))
				{
					FootControllerStructure.laserMeasurementBuffer[FootControllerStructure.laserMeasurementCurrentStoreIndex] = RangingMeasurementData.RangeMilliMeter;

					FootControllerStructure.laserMeasurementCurrentStoreIndex++;
				}

				FootControllerStructure.lockTableFlag = false;
			}/* if (FootControllerStructure.numberOfRangeMeasurement > 0) */

			if (Status != VL53L0X_ERROR_NONE)
			{
				//if error occur then cleare key event to be sure that key will be not pressed constantly and others fields
				FootControllerStructure.laserSensorEventTable[0].eventKeyState = false;
				FootControllerStructure.laserSensorEventTable[0].previousLaserMeasurement = 0xFFFF;
				FootControllerStructure.laserSensorEventTable[0].clearLaserOffsetCounter = 0;
				FootControllerStructure.laserSensorEventTable[0].laserRangeOffset = 0;
				FootControllerStructure.laserSensorEventTable[0].distanceAssignedToRangeOffset = 0;
				FootControllerStructure.laserSensorEventTable[0].thresholdState = false;

				returnCode = 5<<8|(uint8_t)Status;

				return returnCode;
			}
		}/* for ( ; ; ) */
	}/* if (Status == VL53L0X_ERROR_NONE) */
}/* void rangingTest(VL53L0X_Dev_t *pMyDevice) */

/*****************************************************************************************
* SetupHardware() - function available outside and it is used to call configure clock,
* configure GPIO and call configure timer.
*****************************************************************************************/
void SetupHardware(void)
{
	systemSetupClocking();

	GPIO_Init();

	GPIO_Direction(CDC_STATE_INDICATOR_PORT, CDC_STATE_INDICATOR_PIN, GPIO_DIR_OUTPUT);
	GPIO_Direction(FIRST_SENSOR_CONTROL_LINE_PORT, FIRST_SENSOR_CONTROL_LINE_PIN, GPIO_DIR_OUTPUT);
	GPIO_Direction(SECOND_SENSOR_CONTROL_LINE_PORT, SECOND_SENSOR_CONTROL_LINE_PIN, GPIO_DIR_OUTPUT);
	GPIO_Direction(DEBUG_PORT, DEBUG_PIN, GPIO_DIR_OUTPUT);

	GPIO_SetState(CDC_STATE_INDICATOR_PORT, CDC_STATE_INDICATOR_PIN, false);

	setUsbTimer();

	SystemCoreClockUpdate();
}
