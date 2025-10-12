/**
 * @file drv8214_dump.h
 * @author Killian Baillifard
 * @date 12.10.2025
 * @brief Debug dump functions. Dump the content of registers in a human readable format.
 */

#ifndef DRV8214_DUMP_H
#define DRV8214_DUMP_H

#ifdef __cplusplus
extern "C" {
#endif

// Includes

#include "log.h"
#include "drv8214_platform.h"

// Definitions

/**
 * @brief Print macro. Override with the desired print function (e.g. 'printf').
 * @param format String with format identifiers (%d, %.3f and so on).
 * @param ... Variable number of arguments to format.
 */
#define PRINT log_message

// Prototypes

/**
 * @brief Dump current DRV8214 faults.
 */
void drv8214_dump_faults(Drv8214 *driver);

#ifdef __cplusplus
}
#endif

#endif // DRV8214_DUMP_H
