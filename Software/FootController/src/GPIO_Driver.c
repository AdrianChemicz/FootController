#include "GPIO_Driver.h"

#ifdef __LPC11XX__

static uint32_t* GPIO_GetBaseAddress(uint8_t portNumber)
{
	uint32_t* basePointer = 0;

	switch(portNumber)
	{
	case 0:
		basePointer = (uint32_t*)LPC_GPIO0;
		break;
	case 1:
		basePointer = (uint32_t*)LPC_GPIO1;
		break;
	case 2:
		basePointer = (uint32_t*)LPC_GPIO2;
		break;
	case 3:
		basePointer = (uint32_t*)LPC_GPIO3;
		break;
	default:
		basePointer = 0;
		break;
	}

	return basePointer;
}

#endif

void GPIO_Init(void)
{
	//configure pin PIO0_3/USB_VBUS to work as USB_VBUS
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0,  3, (IOCON_FUNC1 | IOCON_MODE_INACT));
}

void GPIO_Direction(uint8_t port, uint8_t pin, GPIO_DIRECTION dir)
{
#ifdef __LPC11XX__
	uint32_t tmpDir = LPC_GPIO0->DIR;
	uint32_t dirMask = ~(1<<pin);
	LPC_GPIO_TypeDef *GPIO_Port = (LPC_GPIO_TypeDef*)GPIO_GetBaseAddress(port);

	GPIO_Port->DIR = (tmpDir & dirMask)|(dir<<pin);
#endif

#if defined(__LPC11UXX__) || defined(__LPC11E6X__)
	uint32_t tmpDir = LPC_GPIO->DIR[port];
	uint32_t dirMask = ~(1<<pin);

	LPC_GPIO->DIR[port] = (tmpDir & dirMask)|(dir<<pin);
#endif
}

void GPIO_SetState(uint8_t port, uint8_t pin, bool state)
{
#ifdef __LPC11XX__
	LPC_GPIO_TypeDef *GPIO_Port = (LPC_GPIO_TypeDef*)GPIO_GetBaseAddress(port);
	GPIO_Port->MASKED_ACCESS[1<<pin] = (state<<pin);
#endif

#if defined(__LPC11UXX__) || defined(__LPC11E6X__)
	LPC_GPIO->B[port][pin] = state;
#endif
}

bool GPIO_GetState(uint8_t port, uint8_t pin)
{	
#ifdef __LPC11XX__
	LPC_GPIO_TypeDef *GPIO_Port = (LPC_GPIO_TypeDef*)GPIO_GetBaseAddress(port);
	return ((GPIO_Port->MASKED_ACCESS[1<<pin])>>pin);
#endif

#if defined(__LPC11UXX__) || defined(__LPC11E6X__)
	return LPC_GPIO->B[port][pin];
#endif
}

