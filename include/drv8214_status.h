/**
 * @file drv8214_status.h
 * @author Killian Baillifard
 * @date 07.10.2025
 * @brief Status registers access. These registers are read only.
 */

#ifndef DRV8214_STATUS_H
#define DRV8214_STATUS_H

#ifdef __cplusplus
extern "C" {
#endif

// Includes

#include "drv8214_platform.h"

// Status registers addresses

#define DRV8214_FAULT                   0x00                            // Various faults register.
#define DRV8214_RC_STATUS1              0x01                            // Ripple counting status register 1.
#define DRV8214_RC_STATUS2              0x02                            // Ripple counting status register 2.
#define DRV8214_RC_STATUS3              0x03                            // Ripple counting status register 3.
#define DRV8214_REG_STATUS1             0x04                            // Regulation status register 1.
#define DRV8214_REG_STATUS2             0x05                            // Regulation status register 2.
#define DRV8214_REG_STATUS3             0x06                            // Regulation status register 3.

// Bit masks for status registers

#define DRV8214_FAULT_FAULT             0b1000'0000                     // FAULT bit, set when a fault occurred.
#define DRV8214_FAULT_STALL             0b0010'0000                     // STALL bit, indicate motor stall.
#define DRV8214_FAULT_OCP               0b0001'0000                     // OCP bit, set if over-current protection was triggered.
#define DRV8214_FAULT_OVP               0b0000'1000                     // OVP bit, set if over-voltage protection was triggered.
#define DRV8214_FAULT_TSD               0b0000'0100                     // TSD bit, set if thermal shut down was triggered.
#define DRV8214_FAULT_NPOR              0b0000'0010                     // NPOR bit, set after a power on reset.
#define DRV8214_FAULT_CNT_DONE          0b0000'0001                     // CNT_DONE bit, set when RC_CNT exceeds the ripple counting threshold.
#define DRV8214_RC_STATUS1_SPEED        0b1111'1111                     // Motor speed estimated by the ripple counting algorithm.
#define DRV8214_RC_STATUS2_CNT_LOW      0b1111'1111                     // 8 LSBs of the 16 bits of the ripple counter.
#define DRV8214_RC_STATUS3_CNT_HIGH     0b1111'1111                     // 8 MSBs of the 16 bits of the ripple counter.
#define DRV8214_REG_STATUS1_VMTR        0b1111'1111                     // Voltage across the motor terminals, with 0x00 -> 0 V and 0xB0 -> 11 V, or 62.5 mV / LSB.
#define DRV8214_REG_STATUS2_IMTR        0b1111'1111                     // Current flowing through the motor, with 0x00 -> 0 A and 0xC0 -> CS_GAIN_SEL bits.
#define DRV8214_REG_STATUS3_IN_DUTY     0b0011'1111                     // When speed or voltage regulation activated, show duty cycle with 0b00'0000 -> 0 % and 0b11'1111 -> 100 %. When regulation is disabled, is set to 0b00'0001 and duty cycle is shown in EXT_DUTY.

// Status conversion constants

#define DRV8214_VMTR_VOLTS_PER_LSB      (11.0f / ((float)0xB0))         // 0x00 -> 0 V and 0xB0 -> 11 V, or 62.5 mV / LSB.
#define DRV8214_IN_DUTY_PERCENT_PER_LSB (100.0f / ((float)0b0011'1111)) // 0b00'0000 -> 0 % and 0b11'1111 -> 100 %.
#define AMPERE_PER_LSB(max)             (max / ((float)0xC0))           // 0x00 -> 0 A and 0xC0 -> CS_GAIN_SEL associated max current

// Getters

/**
 * @brief Read the fault bit of the fault status register.
 * @param driver Handle to the driver structure.
 * @return True if the fault bit is set, false otherwise.
 */
bool drv8214_status_is_fault_set(Drv8214 *driver);

/**
 * @brief Read the stall bit of the fault status register.
 * @param driver Handle to the driver structure.
 * @return True if the stall bit is set, false otherwise.
 */
bool drv8214_status_is_stall_set(Drv8214 *driver);

/**
 * @brief Read the OCP bit of the fault status register.
 * @param driver Handle to the driver structure.
 * @return True if over-current protection was triggered, false otherwise.
 */
bool drv8214_status_is_overcurrent_potection_set(Drv8214 *driver);

/**
 * @brief Read the OVP bit of the fault status register.
 * @param driver Handle to the driver structure.
 * @return True if over-voltage protection was triggered, false otherwise.
 */
bool drv8214_status_is_overvoltage_protection_set(Drv8214 *driver);

/**
 * @brief Read the TSD bit of the fault status register.
 * @param driver Handle to the driver structure.
 * @return True if thermal shut down protection was triggered, false otherwise.
 */
bool drv8214_status_is_thermal_shutdown_set(Drv8214 *driver);

/**
 * @brief Read the NPOR bit of the fault status register.
 * @param driver Handle to the driver structure.
 * @return True if power on reset occurred, false otherwise.
 */
bool drv8214_status_is_power_on_reset_set(Drv8214 *driver);

/**
 * @brief Read the CNT_DONE bit of the fault status register.
 * @param driver Handle to the driver structure.
 * @return True if the ripple counter reached the threshold, false otherwise.
 */
bool drv8214_status_is_count_done_set(Drv8214 *driver);

/**
 * @brief Read the motor speed estimated by the ripple counting algorithm.
 * @param driver Handle to the driver structure.
 * @return Motor speed in radians per seconds.
 */
float drv8214_status_get_speed(Drv8214 *driver);

/**
 * @brief Read the ripple counter.
 * @param driver Handle to the driver structure.
 * @return Ripple count.
 */
uint16_t drv8214_status_get_ripple_count(Drv8214 *driver);

/**
 * @brief Read the voltage across motor terminals.
 * @param driver Handle to the driver structure.
 * @return Voltage in volts.
 */
float drv8214_status_get_motor_voltage(Drv8214 *driver);

/**
 * @brief Read the current flowing through the motor.
 * @param driver Handle to the driver structure.
 * @return Current in amperes.
 */
float drv8214_status_get_motor_current(Drv8214 *driver);

/**
 * @brief Read the duty cycle applied by the built-in speed or voltage regulation.
 * @param driver Handle to the driver structure.
 * @return Duty cycle in percents if speed of voltage regulation is activated, or a placeholder value (~1.59 %) otherwise.
 */
float drv8214_status_get_auto_duty_cycle(Drv8214 *driver);

#ifdef __cplusplus
}
#endif

#endif // DRV8214_STATUS_H
