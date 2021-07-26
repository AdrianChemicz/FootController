#ifndef _VL53L0X_I2C_H_
#define _VL53L0X_I2C_H_

#include "vl53l0x_platform.h"
#include <stdint.h>

VL53L0X_Error VL53L0X_write_multi(uint8_t portNumber, uint8_t address, uint8_t index, uint8_t  *pdata, int32_t count);

VL53L0X_Error VL53L0X_read_multi(uint8_t portNumber, uint8_t address, uint8_t index, uint8_t  *pdata, int32_t count);

VL53L0X_Error VL53L0X_delay(void);

#endif /* _VL53L0X_I2C_H_ */