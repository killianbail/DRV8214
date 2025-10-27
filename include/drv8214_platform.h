/**
 * @file drv8214_platform.h
 * @author Killian Baillifard
 * @date 07.10.2025
 * @brief Plateform configuration for I2C interface of driver.
 */

#ifndef DRV8214_PLATFORM_H
#define DRV8214_PLATFORM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "drv8214_structure.h"

/**
 * @brief Driver structure, fields can be customized as needed.
 */
typedef struct Drv8214 Drv8214;

/**
 * @brief Read driver's fault pin logic level.
 * @param driver Driver handle.
 * @return True if fault pin is active, false otherwise.
 */
bool drv8214_is_fault_pin_active(Drv8214 *driver);

/**
 * @brief Write register of a DRV8214 driver.
 * @param driver Driver handle.
 * @param reg Address of the target register.
 * @param value Value to be written into the register.
 */
void drv8214_i2c_write(Drv8214 *driver, uint8_t reg, uint8_t value);

/**
 * @brief Read register of a DRV8214 driver.
 * @param driver Driver handle.
 * @param reg Address of the target register.
 * @return Register value.
 */
uint8_t drv8214_i2c_read(Drv8214 *driver, uint8_t reg);

#ifdef __cplusplus
}
#endif

#endif // DRV8214_PLATFORM_H
