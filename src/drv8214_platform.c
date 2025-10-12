/*
 * Copyright (c) 2025 Théo Heng
 *
 * This file is part of the drv8214_multiplatform library.
 *
 * Licensed under the MIT License. See the LICENSE file in the project root for full license information.
 */

// Includes

#include <cmsis_os2.h>
#include "drv8214_platform.h"
#include "i2c_dma.h"

// Definitions

uint8_t drv8214_read(Drv8214 *driver, uint8_t reg) {
    uint8_t data;
    i2c_dma_lock(driver->hi2c);
    pca9546a_select_single_channel(driver->mux, driver->muxChannel);
    i2c_dma_read_register(driver->hi2c, (uint16_t)(driver->address << 1), reg, &data, 1);
    i2c_dma_unlock(driver->hi2c);
    return data;
}

void drv8214_write(Drv8214 *driver, uint8_t reg, uint8_t value) {
    i2c_dma_lock(driver->hi2c);
    pca9546a_select_single_channel(driver->mux, driver->muxChannel);
    i2c_dma_write_register(driver->hi2c, (uint16_t)(driver->address << 1), reg, &value, 1);
    i2c_dma_unlock(driver->hi2c);
}
