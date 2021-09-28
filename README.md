# FootController

## 1.Introduction

Main idea behind Footcontroller project was create device which allow user send keyboard event via foot move. This type of press key can be used in game in some circumstances it can be convienient more then standard press key on typical keyboard. When hold one button is required and press other parallel is required then footcontroller is good solution.

   OveralDeviceImage.png

## 2.Mechanical description

Device is connected to pc via USB cable. On device side is used USB mini connector. 
Device detect foot move via laser sensor and appropriate place for sensor was selected. 
In housing was prepared holes for power led, USB serial led, laser sensor and for USB connector. On below schematic was presented all part of device with descriptions.

   DeviceInstructions2.png
Index | Part of device description
------------ | -------------
1 | Laser sensor
2 | Power ON green LED
3 | Debug port connector
4 | USB serial port LED indicator
5 | Mini USB B port

On below schematic was presented all parts used in device.

PartsView.png

## 3.System usage

Device after connect to system is visible as composite device it mean that in single device can be present more than one function. In footcontroller was implemented HID keyboard and CDC(USB serial interface). HID keyboard functionality send event when laser event will be detected. Configuration which key must be presed, mode of pressing and threshold can be configure via USB serial interface.
USB serial interface driver of this device was added to this project and is available in folder "Driver".
When device is connected to PC and serial port will be open then key event will be suppres on 30s. On serial port device can display manual with supported command description. To display manual please send AT_HELP or HELP. Below was added help manual which will be send by device:
_Foot controller serial port is used to configure device and diagnose that device work properly
Configuration is performed using AT command. When serial port will be opened send button event will be blocked on 30s from serial port opening.
Device support below AT command.
AT_SENSOR_ - command used to configure mode of laser sensor to decide how and when press keyboard button. Configuration of button is also possible.
Command expect parameters in order the same as order of described parameters.
1. Sensor number - parameter describe about to which sensor number will be assigned key code, mode and threshold value. Value is stored in decimal format.
Format of parameter:
<Sensor number>
2. Mode of pressing button - parameter decide how button will be pressed. Two options is possible:
PRESS - when event from laser sensor occur button will be pressed only one time even if event will be all time present.
HOLD - when event from laser sensor occur button will be pressed and will be present to moment when laser sensor detect that event disappear.
Format of parameter:
MODE_PRESS
3. Key code - parameter decide which key will be pressed when event occur. Value is stored in decimal format.
Format of parameter:
KEYCODE_6
Few key code example:
17 - N
6 - C
20 - Q
4. Threshold - parameter describe distance how far foot must be move from laser sensor to occur event. Distance is stored in milimiters and in decimal formats.
Format of parameter:
THRESHOLD_20
Example complete AT command formats:
AT_SENSOR_0_MODE_PRESS_KEYCODE_6_THRESHOLD_20
AT_SENSOR_0_MODE_HOLD_KEYCODE_17_THRESHOLD_20
AT_GET_RANGE_COUNT_ - command used to check range measured by laser sensor. Data displayed by this command is as raw milimeter without any preprocesing.
Command need only one parameter which mean number of get range samples. Number of samples must be set in decimal format.
Example complete AT command formats:
AT_GET_RANGE_COUNT_99
AT_STATUS - command used to verify that device work correctly.
Command return information about device current status, number of errors last errors code.
Status and error code equal 0 mean no errors
Example complete AT command format:
AT_STATUS
AT_EVENT_SUSPEND - command used to block button event send by device. This command is active to moment when serial port will be closed.
Example complete AT command format:
AT_EVENT_SUSPEND
AT_EVENT_PARAMETERS - command used to display parameters used by sensors. Command will display important information like:
key mode, key code, threshold need for cause key event, offset value(value used to compare with threshold is - offset value + raw laser sensor range)_

## 4.Electrical schematic

In device was used PCB with microcontroller LPC11U24. This PCB was created in KiCad application and all files created during board preparation was added to PCB project folder.
Below was added connection between main microcontroller and other part of system. In device was used pololu module with ST VL53L0X laser sensor(dev code of board: irs11a). Laser sensor is reset via BC817-25W NPN transistor.

  Schematic.png

