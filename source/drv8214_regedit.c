/**
 * @file drv8214_regedit.c
 * @author Killian Baillifard
 * @date 09.10.2025
 * @brief Registers edition functions.
 */

// Includes

#include "drv8214_regedit.h"

// Implementations

bool drv8214_read_flags(Drv8214 *driver, uint8_t reg, uint8_t mask) {
    return (drv8214_i2c_read(driver, reg) & mask) != 0 ? true : false;
}

void drv8214_write_flags(Drv8214 *driver, uint8_t reg, uint8_t mask, bool state) {
    uint8_t value = drv8214_i2c_read(driver, reg);
    if(state)
        value |= mask;
    else
        value &= ~mask;
    drv8214_i2c_write(driver, reg, value);
}

void drv8214_masked_write(Drv8214 *driver, uint8_t reg, uint8_t mask, uint8_t value) {
    uint8_t content = drv8214_i2c_read(driver, reg);
    content |= (mask & value);
    content &= ~(mask & ~value);
    drv8214_i2c_write(driver, reg, content);
}

uint8_t drv8214_masked_read(Drv8214 *driver, uint8_t reg, uint8_t mask) {
    return drv8214_i2c_read(driver, reg) & mask;
}
