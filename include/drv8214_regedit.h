/**
 * @file drv8214_regedit.h
 * @author Killian Baillifard
 * @date 12.10.2025
 * @brief Driver register edit functions. Use the platform functions from drv8214_platform.c file under the hood.
 */

#ifndef DRV8214_REGEDIT_H
#define DRV8214_REGEDIT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "drv8214_platform.h"

/**
 * @brief Check for flags of a DRV8214 driver.
 * @param driver Driver handle.
 * @param reg Address of the target register.
 * @param flags Truth mask applied on the register value.
 * @return True if any one of the bits of the masks is set.
 */
bool drv8214_read_flags(Drv8214 *driver, uint8_t reg, uint8_t flags);

/**
 * @brief Write bits of given register.
 * @param driver Driver handle.
 * @param reg Address of the target register.
 * @param flags Masked bits of target register.
 * @param state Set mask bits if true, reset mask bits otherwise.
 */
void drv8214_write_flags(Drv8214 *driver, uint8_t reg, uint8_t flags, bool state);

/**
 * @brief Read masked register part of a DRV8214 driver.
 * @param driver Driver handle.
 * @param reg Address of the target register.
 * @param mask Masked bits of target register.
 * @return Masked value.
 */
uint8_t drv8214_masked_read(Drv8214 *driver, uint8_t reg, uint8_t mask);

/**
 * @brief Write masked register part of a DRV8214 driver.
 * @param driver Driver handle.
 * @param reg Address of the target register.
 * @param mask Masked bits of target register.
 * @param value Value to be masked and written in the register.
 */
void drv8214_masked_write(Drv8214 *driver, uint8_t reg, uint8_t mask, uint8_t value);

#ifdef __cplusplus
}
#endif

#endif // DRV8214_REGEDIT_H
