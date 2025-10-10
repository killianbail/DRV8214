/**
 * @file drv8214_platform.h
 * @author Killian Baillifard
 * @date 07.10.2025
 * @brief Configuration registers access. These registers are read / write.
 */

#ifndef DRV8214_PLATFORM_H
#define DRV8214_PLATFORM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stm32wb5mxx.h>

/**
 * @brief Driver structure, can be customized as needed. This structure is only passed
 * as a handle until it reaches the two functions below, no field is touched while it is
 * passed through the intermediate functions.
 */
typedef struct Drv8214 {
    uint8_t address;
    uint8_t channel;
    GPIO_TypeDef *faultPort;
    uint16_t faultPin;
} Drv8214;

/**
 * @brief Write register of a DRV8214 driver.
 * @param driver Handle to the driver structure.
 * @param reg Address of the target register.
 * @param value Value to be written into the register.
 */
void drv8214_write(Drv8214 *driver, uint8_t reg, uint8_t value);

/**
 * @brief Read register of a DRV8214 driver.
 * @param driver Handle to the driver structure.
 * @param reg Address of the target register.
 * @return Register value.
 */
uint8_t drv8214_read(Drv8214 *driver, uint8_t reg);

#ifdef __cplusplus
}
#endif

#endif // DRV8214_PLATFORM_H
