# DRV8214 Library

## Overview

This library provides a high level multiplatform C interface for Texas Instruments **DRV8214** brushed DC motor driver.

## Platform

To work with this library, 2 files must be implemented :

- `drv8214_structure.h` which contains the definition of the structure passed through the library. This structure include platform specific fields such as the I2C peripheral.

```c
#ifndef DRV8214_STRUCTURE_H
#define DRV8214_STRUCTURE_H

#ifdef __cplusplus
extern "C" {
#endif

// Includes

#include <my_mcu_defs.h>

// Structures

struct Drv8214 {

    // I2C interface

    MY_MCU_I2C_Handle_t *i2c;
    uint8_t address;

    // GPIO fault pin 

    MY_MCU_GPIO_Handle_t *faultGpio;

};

#ifdef __cplusplus
}
#endif

#endif /* DRV8214_STRUCTURE_H */
```

- `drv8214_platform.c`, which contains functions definitions to read and write registers of the driver through I2C, and possibly read the fault pin status.

```c
// Includes

#include <my_mcu_defs.h>
#include "drv8214_platform.h"

// Definitions

bool drv8214_is_fault_pin_active(Drv8214 *driver) {
    return my_mcu_gpio_read(driver->faultGpio) == 0;
}

uint8_t drv8214_i2c_read(Drv8214 *driver, uint8_t reg) {
    uint8_t data;
    my_mcu_i2c_read(driver->i2c, driver->address, driver->reg, &data, 1);
    return data;
}

void drv8214_i2c_write(Drv8214 *driver, uint8_t reg, uint8_t value) {
    my_mcu_i2c_write(driver->i2c, driver->address, driver->reg, &value, 1);
}
```

## Credits

- Theo Heng, original creator.
- Killian Baillifard, current reworked version.
