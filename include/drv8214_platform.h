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
#include <stm32wb5mxx.h>
#include <stm32wbxx_hal.h>
#include "pca9546a.h"

/**
 * @brief Driver structure, fields can be customized as needed.
 */
typedef struct Drv8214 {

    // I2C interface

    I2C_HandleTypeDef *hi2c;
    uint8_t address;
    Pca9546a *mux;
    uint8_t muxChannel;

    // GPIO fault pin 

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
