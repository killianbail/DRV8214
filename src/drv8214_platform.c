/*
 * Copyright (c) 2025 Théo Heng
 *
 * This file is part of the drv8214_multiplatform library.
 *
 * Licensed under the MIT License. See the LICENSE file in the project root for full license information.
 */

// Includes

#include "drv8214_platform.h"
#include "i2c.h"
#include "tca9548.h"

// Definitions

uint8_t drv8214_read(Drv8214 *driver, uint8_t reg) {
    uint8_t data;
    i2c_rtos_lock(&hi2c1); // TODO select mux
    i2c_mux_select(?, driver->channel);
    i2c_rtos_transmit(&hi2c1, (uint16_t)(driver->address << 1), &reg, 1);
    i2c_rtos_receive(&hi2c1, (uint16_t)(driver->address << 1), &data, 1);
    i2c_rtos_unlock(&hi2c1);
    return data;
}

void drv8214_write(Drv8214 *driver, uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    i2c_rtos_lock(&hi2c1); // TODO select mux
    i2c_rtos_transmit(&hi2c1, (uint16_t)(driver->address << 1), data, 2);
    i2c_rtos_unlock(&hi2c1);
}
