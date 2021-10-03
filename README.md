# FootController

## 1.Introduction

Main idea behind Footcontroller project was create device which allow user send keyboard event via foot move. This type of press key can be used in game in some circumstances it can be convienient more then standard press key on typical keyboard. When hold one button is required and press other parallel is required then footcontroller is good solution.

![Device overview](/Doc/OveralDeviceImage.png)

## 2.Mechanical description

Device is connected to pc via USB cable. On device side is used USB mini connector.<br />
Device detect foot move via laser sensor and appropriate place for sensor was selected. <br />
In housing was prepared holes for power led, USB serial led, laser sensor and for USB connector. On below schematic was presented all part of device with descriptions.<br />

![Device view](/Doc/DeviceInstructions.png)

Index | Part of device description
------------ | -------------
1 | Laser sensor
2 | Power ON green LED
3 | Debug port connector
4 | USB serial port LED indicator
5 | Mini USB B port

On below schematic was presented all parts used in device.

![Device view](/Doc/PartsView.png)

## 3.System usage

Device after connect to system is visible as composite device it mean that in single device can be present more than one function. In footcontroller was implemented HID keyboard and CDC(USB serial interface). HID keyboard functionality send event when laser event will be detected. Configuration which key must be presed, mode of pressing and threshold can be configure via USB serial interface.<br /><br />
USB serial interface driver of this device was added to this project and is available in folder "Driver".
When device is connected to PC and serial port will be open then key event will be suppres on 30s. On serial port device can display manual with supported command description. To display manual please send AT_HELP or HELP. Below was added help manual which will be send by device:
>Foot controller serial port is used to configure device and diagnose that device work properly<br />
>Configuration is performed using AT command. When serial port will be opened send button event will be blocked on 30s from serial port opening.<br />
>Device support below AT command<br />
>AT_SENSOR_ - command used to configure mode of laser sensor to decide how and when press keyboard button. Configuration of button is also possible.<br />
>Command expect parameters in order the same as order of described parameters.<br />
>1\. Sensor number - parameter describe about to which sensor number will be assigned key code, mode and threshold value. Value is stored in decimal format.<br />
>Format of parameter:<br />
>\<Sensor number\><br />
>2. Mode of pressing button - parameter decide how button will be pressed. Two options is possible:<br />
>PRESS - when event from laser sensor occur button will be pressed only one time even if event will be all time present.<br />
>HOLD - when event from laser sensor occur button will be pressed and will be present to moment when laser sensor detect that event disappear.<br />
>Format of parameter:<br />
>MODE_PRESS<br />
>3. Key code - parameter decide which key will be pressed when event occur. Value is stored in decimal format.<br />
>Format of parameter:<br />
>KEYCODE_6<br />
>Few key code example:<br />
>17 - N<br />
>6 - C<br />
>20 - Q<br />
>4. Threshold - parameter describe distance how far foot must be move from laser sensor to occur event. Distance is stored in milimiters and in decimal formats.<br />
>Format of parameter:<br />
>THRESHOLD_20<br />
>Example complete AT command formats:<br />
>AT_SENSOR_0_MODE_PRESS_KEYCODE_6_THRESHOLD_20<br />
>AT_SENSOR_0_MODE_HOLD_KEYCODE_17_THRESHOLD_20<br />
>AT_GET_RANGE_COUNT_ - command used to check range measured by laser sensor. Data displayed by this command is as raw milimeter without any preprocesing.<br />
>Command need only one parameter which mean number of get range samples. Number of samples must be set in decimal format.<br />
>Example complete AT command formats:<br />
>AT_GET_RANGE_COUNT_99<br />
>AT_STATUS - command used to verify that device work correctly.<br />
>Command return information about device current status, number of errors last errors code.<br />
>Status and error code equal 0 mean no errors<br />
>Example complete AT command format:<br />
>AT_STATUS<br />
>AT_EVENT_SUSPEND - command used to block button event send by device. This command is active to moment when serial port will be closed.<br />
>Example complete AT command format:<br />
>AT_EVENT_SUSPEND<br />
>AT_EVENT_PARAMETERS - command used to display parameters used by sensors. Command will display important information like:<br />
>key mode, key code, threshold need for cause key event, offset value(value used to compare with threshold is - offset value + raw laser sensor range)<br />

## 4.Electrical schematic

In device was used PCB with microcontroller LPC11U24. This PCB was created in KiCad application and all files created during board preparation was added to PCB project folder.
Below was added connection between main microcontroller and other part of system. In device was used pololu module with ST VL53L0X laser sensor(dev code of board: irs11a). Laser sensor is reset via BC817-25W NPN transistor.

![Device view](/Doc/Schematic.png)

Index | Pin function | Pin usage
------------ | ------------ | -------------
1 | GND | Ground for LED
2 | GND | Ground connected to BC817-25W transistor emiter pin
3 | GPIO | Output port use to control CDC LED indicator
4 | GPIO | Output port use to control BC817-25W transistor base pin
5 | GPIO | Output port use to debug durring development process
6 | 3V3 | 3.3V power supply for laser sensor
7 | SDA | I2C data pin use to comunication with laser sensor
8 | SCL | I2C clock pin use to comunication with laser sensor
9 | GPIO | Input port use to block send key event from laser. Connect to ground to block event.

## 5.Housing of device

Housing of device was prepared in Blender application. Complete housing consist of two element main element and electronic board holder. To mount complete device is require put three or two nuts for screw and threaded rod to nut holes in main element. 

![Main element in blender](/Doc/ViewMainElement.png)

In Housing project folder was added blender files with single element and all steps(before using boolean modifier) necessary to create final element.

## 6.Code design

Footcontroller source code firmware was divided into few modules which was divided according to purpose. In below table was added all modules in project:

Module name | Module description
------------ | ------------
Descriptors.h | Contain USB things like structures and information which will be visible on PC after device connection
EEPROM_Driver.h | Provide function to read and write functions to store information about sensor activation parameters
FootController.h | Main module with main function. Module handle USB communication like serial and HID interfaces.
GPIO_Driver.h | Provide API for GPIO microcontrollers ports
HW_Driver.h | Module responsible for clock microcontroller clock settings and communication with laser sensor.
I2C_Driver.h | Provide API for I2C microcontroller interface
iap.h | Provide API to manage EEPROM memory and flash.
StringGenerateParseModule.h | Provide API to parse or generate strings. Functions in module are used by USB serial communication.

FootController.h module between other modules share FootControllerStructure which is visible as global variable. Many functions from other modules access to field in this structure directly.

To build firmware also was used ST VL53L0X API library to handle VL53L0X full sensor i2c comunication. ST library provide all necessary API to comunicate with sensor without manual call I2C API. API is available on below location:
https://www.st.com/en/embedded-software/stsw-img005.html
Project also use two other library which not exist in project directory but to correct compile file must be added outside. Those library are lpc_chip_11uxx_lib and nxp_lpcxpresso_11u14_usblib_device. 

Project to compile without errors require from user additional steps. Assign VID and PID number in Descriptors.c file because author of project isn't usb.org member or didn't buy VID number. The same thing must be performed with FootControllerSerialDriver.inf driver file in Driver project folder. Below was added screenshot both file with marked code lines when changes must be performed. In both file VID and PID must be the same.

![VID and PID](/Doc/VIDandPID.png)

## 7.Execution sequence diagram

On below schematic was presented how application running on footcontroller to provide overal information before deeper code analisis.

![Sequence diagram](/Doc/SequenceDiagram.png)
  
In main function after initialization is executed in infinite loop laser sensor handling. USB process is performed in TIMER16_0_IRQHandler timer interrupt which is call every 83.5 us.

## 8.Known issues

-Very often device send null signs in string to PC on serial port. Issue can be cause by incorrect use NXP USB library.

