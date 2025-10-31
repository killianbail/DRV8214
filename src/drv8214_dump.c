/**
 * @file drv8214_dump.c
 * @author Killian Baillifard
 * @date 12.10.2025
 * @brief Debug dump functions implementation.
 */

// Includes

#include <stdio.h>
#include "drv8214_dump.h"
#include "drv8214_status.h"

// Definitions

void drv8214_dump_faults(Drv8214 *driver, char *buffer, size_t size) {

    // Get pin state and faults flags

    bool faultPinLow = drv8214_is_fault_pin_active(driver);
    uint8_t faults = drv8214_get_faults(driver);

    // Print dump

    snprintf(buffer, size,
        "---      I/O        ---\n"
        "Fault pin : %s\n"
        "--- Faults register ---\n"
        "FAULT     : %s\n"
        "STALL     : %s\n"
        "OCP       : %s\n"
        "OVP       : %s\n"
        "TSD       : %s\n"
        "NPOR      : %s\n"
        "CNT_DONE  : %s\n",
        faultPinLow                     ? "Active"  : "Inactive",
        faults & DRV8214_FAULT_FAULT    ? "Yes"     : "No",
        faults & DRV8214_FAULT_STALL    ? "Yes"     : "No",
        faults & DRV8214_FAULT_OCP      ? "Yes"     : "No",
        faults & DRV8214_FAULT_OVP      ? "Yes"     : "No",
        faults & DRV8214_FAULT_TSD      ? "Yes"     : "No",
        faults & DRV8214_FAULT_NPOR     ? "Yes"     : "No",
        faults & DRV8214_FAULT_CNT_DONE ? "Yes"     : "No"
    );
}
