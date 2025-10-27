/**
 * @file drv8214_status.c
 * @authors Killian Baillifard
 * @date 07.10.2025
 * @brief DRV8214 status register access implementation.
 */

// Includes

#include <math.h>
#include "drv8214_status.h"
#include "drv8214_control.h"
#include "drv8214_regedit.h"
#include "main.h"
#include "gpio.h"

// Definitions

uint8_t drv8214_get_fault_flags(Drv8214 *driver) {
    return drv8214_i2c_read(driver, DRV8214_FAULT);
}

bool drv8214_is_fault_set(Drv8214 *driver) {
    return drv8214_read_flags(driver, DRV8214_FAULT, DRV8214_FAULT_FAULT);
}

bool drv8214_is_stall_set(Drv8214 *driver) {
    return drv8214_read_flags(driver, DRV8214_FAULT, DRV8214_FAULT_STALL);
}

bool drv8214_is_overcurrent_protection_set(Drv8214 *driver) {
    return drv8214_read_flags(driver, DRV8214_FAULT, DRV8214_FAULT_OCP);
}

bool drv8214_is_overvoltage_protection_set(Drv8214 *driver) {
    return drv8214_read_flags(driver, DRV8214_FAULT, DRV8214_FAULT_OVP);
}

bool drv8214_is_thermal_shutdown_set(Drv8214 *driver) {
    return drv8214_read_flags(driver, DRV8214_FAULT, DRV8214_FAULT_TSD);
}

bool drv8214_is_power_on_reset_set(Drv8214 *driver) {
    return drv8214_read_flags(driver, DRV8214_FAULT, DRV8214_FAULT_NPOR);
}

bool drv8214_is_count_done_set(Drv8214 *driver) {
    return drv8214_read_flags(driver, DRV8214_FAULT, DRV8214_FAULT_CNT_DONE);
}

float drv8214_get_speed(Drv8214 *driver) {
    const float SPEED = (float)drv8214_masked_read(driver, DRV8214_RC_STATUS1, DRV8214_RC_STATUS1_SPEED);
    switch(drv8214_get_speed_scaling_factor(driver)) {
        case DRV8214_W_SCALE_16:    return SPEED * 16.0f;
        case DRV8214_W_SCALE_32:    return SPEED * 32.0f;
        case DRV8214_W_SCALE_64:    return SPEED * 64.0f;
        case DRV8214_W_SCALE_128:   return SPEED * 128.0f;
        default:                    return NAN;
    }
}

uint16_t drv8214_get_ripple_count(Drv8214 *driver) {
    uint16_t rippleCount = 0;
    rippleCount |= drv8214_masked_read(driver, DRV8214_RC_STATUS2, DRV8214_RC_STATUS2_CNT_LOW) << 0;
    rippleCount |= drv8214_masked_read(driver, DRV8214_RC_STATUS3, DRV8214_RC_STATUS3_CNT_HIGH) << 8;
    return rippleCount;
}

float drv8214_get_motor_voltage(Drv8214 *driver) {
    uint8_t volts = drv8214_masked_read(driver, DRV8214_REG_STATUS1, DRV8214_REG_STATUS1_VMTR);
    return ((float)volts) * DRV8214_VMTR_VOLTS_PER_LSB;
}

float drv8214_get_motor_current(Drv8214 *driver) {
    const float CURRENT_RAW_VALUE = (float)drv8214_masked_read(driver, DRV8214_REG_STATUS2, DRV8214_REG_STATUS2_IMTR);
    switch(drv8214_get_current_mirror_gain(driver)) {
        case DRV8214_4_A_MAX_225_UA_PER_A:      return CURRENT_RAW_VALUE * AMPERE_PER_LSB(4.0f);
        case DRV8214_2_A_MAX_225_UA_PER_A:      return CURRENT_RAW_VALUE * AMPERE_PER_LSB(2.0f);
        case DRV8214_1_A_MAX_1125_UA_PER_A:     return CURRENT_RAW_VALUE * AMPERE_PER_LSB(1.0f);
        case DRV8214_500_MA_MAX_1125_UA_PER_A:  return CURRENT_RAW_VALUE * AMPERE_PER_LSB(0.5f);
        case DRV8214_250_MA_MAX_5560_UA_PER_A:  return CURRENT_RAW_VALUE * AMPERE_PER_LSB(0.25f);
        case DRV8214_125_MA_MAX_5560_UA_PER_A:  return CURRENT_RAW_VALUE * AMPERE_PER_LSB(0.125f);
        default:                                return NAN;
    }
}

float drv8214_get_auto_duty_cycle(Drv8214 *driver) {
    return ((float)drv8214_masked_read(driver, DRV8214_REG_STATUS3, DRV8214_REG_STATUS3_IN_DUTY)) * DRV8214_IN_DUTY_PERCENT_PER_LSB;
}
