/* This file base on original NXP file and was modify by author of project */
/*
 * @brief USB Device Descriptors, for library use when in USB device mode. Descriptors are special
 *        computer-readable structures which the host requests upon device enumeration, to determine
 *        the device's capabilities and functions
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2012
 * Copyright(C) Dean Camera, 2011, 2012
 * All rights reserveAAd.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#include "Descriptors.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/** HID class report descriptor. This is a special descriptor constructed with values from the
 *  USBIF HID class specification to describe the reports and capabilities of the HID device. This
 *  descriptor is parsed by the host and its contents used to determine what data (and in what encoding)
 *  the device will send, and what it may be sent back from the host. Refer to the HID specification for
 *  more details on HID report descriptors.
 */
USB_Descriptor_HIDReport_Datatype_t KeyboardReport[] = {
	/* Use the HID class driver's standard Keyboard report.
	 *   Max simultaneous keys: 6
	 */
	HID_DESCRIPTOR_KEYBOARD(6)
};

/** Device descriptor structure. This descriptor, located in FLASH memory, describes the overall
 *  device characteristics, including the supported USB version, control endpoint size and the
 *  number of device configurations. The descriptor is read out by the USB host when the enumeration
 *  process begins.
 */
USB_Descriptor_Device_t DeviceDescriptor = {
	.Header                 = {.Size = sizeof(USB_Descriptor_Device_t), .Type = DTYPE_Device},

	.USBSpecification       = VERSION_BCD(01.10),
	.Class                  = USB_CSCP_IADDeviceClass,
	.SubClass               = USB_CSCP_IADDeviceSubclass,
	.Protocol               = USB_CSCP_IADDeviceProtocol,

	.Endpoint0Size          = FIXED_CONTROL_ENDPOINT_SIZE,

	.VendorID               = 0xXXXX,
	.ProductID              = 0xXXXX,
	.ReleaseNumber          = VERSION_BCD(00.01),

	.ManufacturerStrIndex   = 0x01,
	.ProductStrIndex        = 0x02,
	.SerialNumStrIndex      = NO_DESCRIPTOR,

	.NumberOfConfigurations = FIXED_NUM_CONFIGURATIONS
};

/** Configuration descriptor structure. This descriptor, located in FLASH memory, describes the usage
 *  of the device in one of its supported configurations, including information about any device interfaces
 *  and endpoints. The descriptor is read out by the USB host during the enumeration process when selecting
 *  a configuration so that the host may correctly communicate with the USB device.
 */
USB_Descriptor_Configuration_t ConfigurationDescriptor = {
	.Config = {
		.Header                 = {.Size = sizeof(USB_Descriptor_Configuration_Header_t), .Type = DTYPE_Configuration},
		.TotalConfigurationSize = sizeof(USB_Descriptor_Configuration_t) - 1,		// termination byte not included in size
		.TotalInterfaces        = 3,
		.ConfigurationNumber    = 1,
		.ConfigurationStrIndex  = NO_DESCRIPTOR,
		.ConfigAttributes       = (USB_CONFIG_ATTR_BUSPOWERED | USB_CONFIG_ATTR_SELFPOWERED),
		.MaxPowerConsumption    = USB_CONFIG_POWER_MA(100)
	},

	.HID_Interface = {
		.Header                 = {.Size = sizeof(USB_Descriptor_Interface_t), .Type = DTYPE_Interface},
		.InterfaceNumber        = 0x00,
		.AlternateSetting       = 0x00,
		.TotalEndpoints         = 1,
		.Class                  = HID_CSCP_HIDClass,
		.SubClass               = HID_CSCP_BootSubclass,
		.Protocol               = HID_CSCP_KeyboardBootProtocol,
		.InterfaceStrIndex      = NO_DESCRIPTOR
	},

	.HID_KeyboardHID = {
		.Header                 = {.Size = sizeof(USB_HID_Descriptor_HID_t), .Type = HID_DTYPE_HID},
		.HIDSpec                = VERSION_BCD(01.11),
		.CountryCode            = 0x00,
		.TotalReportDescriptors = 1,
		.HIDReportType          = HID_DTYPE_Report,
		.HIDReportLength        = sizeof(KeyboardReport)
	},

	.HID_ReportINEndpoint = {
		.Header                 = {.Size = sizeof(USB_Descriptor_Endpoint_t), .Type = DTYPE_Endpoint},
		.EndpointAddress        = (ENDPOINT_DIR_IN | KEYBOARD_EPNUM),
		.Attributes             = (EP_TYPE_INTERRUPT | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA),
		.EndpointSize           = KEYBOARD_EPSIZE,
		.PollingIntervalMS      = 0x01
	},
	//CDC
	.CDC_IAD_Header = {
			.Header = {.Size = sizeof(USB_Descriptor_Interface_Association_t), .Type = DTYPE_InterfaceAssociation},

			.FirstInterfaceIndex  = 1,
			.TotalInterfaces = 2,
			.Class = CDC_CSCP_CDCClass,
			.SubClass = CDC_CSCP_NoSpecificSubclass,
			.Protocol = CDC_CSCP_NoSpecificProtocol,
			.IADStrIndex = 3
	},

	.CDC_CCI_Interface = {
		.Header                 = {.Size = sizeof(USB_Descriptor_Interface_t), .Type = DTYPE_Interface},
		.InterfaceNumber        = 0x01,
		.AlternateSetting       = 0,
		.TotalEndpoints         = 1,
		.Class                  = CDC_CSCP_CDCClass,
		.SubClass               = CDC_CSCP_ACMSubclass,
		.Protocol               = CDC_CSCP_ATCommandProtocol,
		.InterfaceStrIndex      = NO_DESCRIPTOR
	},

	.CDC_Functional_Header = {
		.Header                 = {.Size = sizeof(USB_CDC_Descriptor_FunctionalHeader_t), .Type = DTYPE_CSInterface},
		.Subtype                = CDC_DSUBTYPE_CSInterface_Header,
		.CDCSpecification       = VERSION_BCD(01.10),
	},

	.CDC_Functional_ACM = {
		.Header                 = {.Size = sizeof(USB_CDC_Descriptor_FunctionalACM_t), .Type = DTYPE_CSInterface},
		.Subtype                = CDC_DSUBTYPE_CSInterface_ACM,
		.Capabilities           = 0x06,
	},

	.CDC_Functional_Union = {
		.Header                 = {.Size = sizeof(USB_CDC_Descriptor_FunctionalUnion_t), .Type = DTYPE_CSInterface},
		.Subtype                = CDC_DSUBTYPE_CSInterface_Union,
		.MasterInterfaceNumber  = 0,
		.SlaveInterfaceNumber   = 1,
	},

	.CDC_NotificationEndpoint = {
		.Header                 = {.Size = sizeof(USB_Descriptor_Endpoint_t), .Type = DTYPE_Endpoint},
		.EndpointAddress        = (ENDPOINT_DIR_IN | CDC_NOTIFICATION_EPNUM),
		.Attributes             = (EP_TYPE_INTERRUPT | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA),
		.EndpointSize           = CDC_NOTIFICATION_EPSIZE,
		.PollingIntervalMS      = 0xFF
	},

	.CDC_DCI_Interface = {
		.Header                 = {.Size = sizeof(USB_Descriptor_Interface_t), .Type = DTYPE_Interface},
		.InterfaceNumber        = 0x02,
		.AlternateSetting       = 0,
		.TotalEndpoints         = 2,
		.Class                  = CDC_CSCP_CDCDataClass,
		.SubClass               = CDC_CSCP_NoDataSubclass,
		.Protocol               = CDC_CSCP_NoDataProtocol,
		.InterfaceStrIndex      = NO_DESCRIPTOR
	},

	.CDC_DataOutEndpoint = {
		.Header                 = {.Size = sizeof(USB_Descriptor_Endpoint_t), .Type = DTYPE_Endpoint},
		.EndpointAddress        = (ENDPOINT_DIR_OUT | CDC_RX_EPNUM),
		.Attributes             = (EP_TYPE_BULK | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA),
		.EndpointSize           = CDC_TXRX_EPSIZE,
		.PollingIntervalMS      = 0x01
	},

	.CDC_DataInEndpoint = {
		.Header                 = {.Size = sizeof(USB_Descriptor_Endpoint_t), .Type = DTYPE_Endpoint},
		.EndpointAddress        = (ENDPOINT_DIR_IN | CDC_TX_EPNUM),
		.Attributes             = (EP_TYPE_BULK | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA),
		.EndpointSize           = CDC_TXRX_EPSIZE,
		.PollingIntervalMS      = 0x01
	},

	.CDC_Termination = 0x00
};

/** Language descriptor structure. This descriptor, located in FLASH memory, is returned when the host requests
 *  the string descriptor with index 0 (the first index). It is actually an array of 16-bit integers, which indicate
 *  via the language ID table available at USB.org what languages the device supports for its string descriptors.
 */
uint8_t LanguageString[] = {
	USB_STRING_LEN(1),
	DTYPE_String,
	WBVAL(LANGUAGE_ID_ENG),
};
USB_Descriptor_String_t *LanguageStringPtr = (USB_Descriptor_String_t *) LanguageString;

/** Manufacturer descriptor string. This is a Unicode string containing the manufacturer's details in human readable
 *  form, and is read out upon request by the host when the appropriate string ID is requested, listed in the Device
 *  Descriptor.
 */
uint8_t ManufacturerString[] = {
	USB_STRING_LEN(5),
	DTYPE_String,
	WBVAL('H'),
	WBVAL('e'),
	WBVAL('s'),
	WBVAL('u'),
	WBVAL('s'),
};
USB_Descriptor_String_t *ManufacturerStringPtr = (USB_Descriptor_String_t *) ManufacturerString;

/** Product descriptor string. This is a Unicode string containing the product's details in human readable form,
 *  and is read out upon request by the host when the appropriate string ID is requested, listed in the Device
 *  Descriptor.
 */
uint8_t ProductString[] = {
	USB_STRING_LEN(15),
	DTYPE_String,
	WBVAL('F'),
	WBVAL('o'),
	WBVAL('o'),
	WBVAL('t'),
	WBVAL(' '),
	WBVAL('c'),
	WBVAL('o'),
	WBVAL('n'),
	WBVAL('t'),
	WBVAL('r'),
	WBVAL('o'),
	WBVAL('l'),
	WBVAL('l'),
	WBVAL('e'),
	WBVAL('r'),
};
USB_Descriptor_String_t *ProductStringPtr = (USB_Descriptor_String_t *) ProductString;

uint8_t SerialConfiguratorString[] = {
	USB_STRING_LEN(19),
	DTYPE_String,
	WBVAL('S'),
	WBVAL('e'),
	WBVAL('r'),
	WBVAL('i'),
	WBVAL('a'),
	WBVAL('l'),
	WBVAL(' '),
	WBVAL('c'),
	WBVAL('o'),
	WBVAL('n'),
	WBVAL('f'),
	WBVAL('i'),
	WBVAL('g'),
	WBVAL('u'),
	WBVAL('r'),
	WBVAL('a'),
	WBVAL('t'),
	WBVAL('o'),
	WBVAL('r'),
};
USB_Descriptor_String_t *SerialConfiguratorStringPtr = (USB_Descriptor_String_t *) SerialConfiguratorString;


/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/** This function is called by the library when in device mode, and must be overridden (see library "USB Descriptors"
 *  documentation) by the application code so that the address and size of a requested descriptor can be given
 *  to the USB library. When the device receives a Get Descriptor request on the control endpoint, this function
 *  is called so that the descriptor details can be passed back and the appropriate descriptor sent back to the
 *  USB host.
 */
uint16_t CALLBACK_USB_GetDescriptor(uint8_t corenum,
									const uint16_t wValue,
									const uint8_t wIndex,
									const void * *const DescriptorAddress)
{
	const uint8_t  DescriptorType   = (wValue >> 8);
	const uint8_t  DescriptorNumber = (wValue & 0xFF);

	const void *Address = NULL;
	uint16_t    Size    = NO_DESCRIPTOR;

	switch (DescriptorType) {
	case DTYPE_Device:
		Address = &DeviceDescriptor;
		Size    = sizeof(USB_Descriptor_Device_t);
		break;

	case DTYPE_Configuration:
		Address = &ConfigurationDescriptor;
		Size    = ConfigurationDescriptor.Config.TotalConfigurationSize;
		break;

	case DTYPE_String:
		switch (DescriptorNumber) {
		case 0x00:
			Address = LanguageStringPtr;
			Size    = pgm_read_byte(&LanguageStringPtr->Header.Size);
			break;

		case 0x01:
			Address = ManufacturerStringPtr;
			Size    = pgm_read_byte(&ManufacturerStringPtr->Header.Size);
			break;

		case 0x02:
			Address = ProductStringPtr;
			Size    = pgm_read_byte(&ProductStringPtr->Header.Size);
			break;

		case 0x03:
			Address = SerialConfiguratorStringPtr;
			Size    = pgm_read_byte(&SerialConfiguratorStringPtr->Header.Size);
			break;
		}

		break;

	case HID_DTYPE_HID:
		Address = &ConfigurationDescriptor.HID_KeyboardHID;
		Size    = sizeof(USB_HID_Descriptor_HID_t);
		break;

	case HID_DTYPE_Report:
		Address = &KeyboardReport;
		Size    = sizeof(KeyboardReport);
		break;
	}

	*DescriptorAddress = Address;
	return Size;
}
