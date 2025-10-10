/**
 * @file drv8214_config.h
 * @author Killian Baillifard
 * @date 07.10.2025
 * @brief Configuration registers access. These registers are read / write.
 */

#ifndef DRV8214_CONFIG_H
#define DRV8214_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

// Includes

#include "drv8214_platform.h"

// Configuration registers addresses

#define DRV8214_CONFIG0                     0x09        // Configuration register 1 (faults).
#define DRV8214_CONFIG1                     0x0A        // Configuration register 2.
#define DRV8214_CONFIG2                     0x0B        // Configuration register 3.
#define DRV8214_CONFIG3                     0x0C        // Configuration register 4.
#define DRV8214_CONFIG4                     0x0D        // Configuration register 5.

// Bit masks for configuration registers

#define DRV8214_CONFIG0_EN_OUT              0b10000000  // Enable the driver outputs if set.
#define DRV8214_CONFIG0_EN_OVP              0b01000000  // Enable the over-voltage protection if set (set by default).
#define DRV8214_CONFIG0_EN_STALL            0b00100000  // Enable the stall detection feature if set (set by default).
#define DRV8214_CONFIG0_VSNS_SEL            0b00010000  // Enable the digital low-pass filter for voltage regulation if set. Use the analog low-pass filter otherwise (recommended).
#define DRV8214_CONFIG0_VM_GAIN_SEL         0b00001000  // Select to voltage range used during small voltage regulation (0 V to 15.7 V by default, 0 V to 3.92 V when set).
#define DRV8214_CONFIG0_CLR_CNT             0b00000100  // Reset the ripple counter to 0 and clear CNT_DONE when set, also release nFAULT when RC_REP = 0b10 (automatically reset after a set).
#define DRV8214_CONFIG0_CLR_FLT             0b00000010  // Clear all latched faults when set (automatically reset after a set).
#define DRV8214_CONFIG0_DUTY_CTRL           0b00000001  // When speed regulation is disabled, setting this bit allow writing the desired PWM in EXT_DUTY.
#define DRV8214_CONFIG1_TINRUSH             0b11111111  // 8 LSBs of inrush time, for which the stall detection will not trigger.
#define DRV8214_CONFIG2_TINRUSH             0b11111111  // 8 MSBs of inrush time, for which the stall detection will not trigger.
#define DRV8214_CONFIG3_IMODE               0b11000000  // Determine the behavior of current regulation (0b10 by default).
#define DRV8214_CONFIG3_SMODE               0b00100000  // Program the device response to a stall condition (set by default).
#define DRV8214_CONFIG3_INT_VREF            0b00010000  // Fix internal reference voltage to 0.5 V internally when set, voltage is not fixed internally otherwise.
#define DRV8214_CONFIG3_TBLANK              0b00001000  // Set current sense blanking time (1.8 μs by default, 1.0 μs when set).
#define DRV8214_CONFIG3_TDEG                0b00000100  // Set the current regulation and stall detection deglitch time (2 μs by default, 1 μs when set).
#define DRV8214_CONFIG3_OCP_MODE            0b00000010  // Program the device response to an overcurrent event (device perform auto-retry by default, device is latched off when set).
#define DRV8214_CONFIG3_TSD_MODE            0b00000001  // Program device response to an over-temperature event (device perform auto-retry by default, device is latched off when set).
#define DRV8214_CONFIG4_RC_REP              0b11000000  // Determines whether nFAULT is pulled low when RC_CNT exceeds threshold, and the behavior of RC_CNT when it reaches maximum value of 65535.
#define DRV8214_CONFIG4_STALL_REP           0b00100000  // Determines whether stall is reported on the nFAULT pin. When set, nFAULT is low whenever stall is detected. When reset, stall is not reported on nFAULT output (set by default).
#define DRV8214_CONFIG4_CBC_REP             0b00010000  // In cycle-by-cycle current regulation mode, when set, nFAULT is pulled low when H-Bridge enter internal current regulation, and is not otherwise (set by default).
#define DRV8214_CONFIG4_PMODE               0b00001000  // Change between phase / enable mode (PH/EN) when reset or PWM mode when set (set by default).
#define DRV8214_CONFIG4_I2C_BC              0b00000100  // Bridge control is confgured by INx pins when reset or by I2C bits in I2C_EN_IN1 and I2C_PH_IN2 when set.
#define DRV8214_CONFIG4_I2C_BC_MODE         0b00000011  // PH / EN or PWM input bits for internal bridge control when I2C_BC is set, ignored if I2C_BC is reset.

// Configuration options shifts

#define DRV8214_CONFIG0_VSNS_SEL_SHIFT      4
#define DRV8214_CONFIG0_VM_GAIN_SEL_SHIFT   3
#define DRV8214_CONFIG3_IMODE_SHIFT         6
#define DRV8214_CONFIG3_SMODE_SHIFT         5
#define DRV8214_CONFIG3_INT_VREF_SHIFT      4
#define DRV8214_CONFIG3_TBLANK_SHIFT        3
#define DRV8214_CONFIG3_TDEG_SHIFT          2
#define DRV8214_CONFIG3_OCP_MODE_SHIFT      1
#define DRV8214_CONFIG4_RC_REP_SHIFT        6
#define DRV8214_CONFIG4_PMODE_SHIFT         3
#define DRV8214_CONFIG4_I2C_BC_SHIFT        2

// Configuration conversion constants

#define DRV8214_TINRUSH_MIN_S               0.005f  // Inrush time is at least 5 ms when TINRUSH = 0x0000.
#define DRV8214_TINRUSH_MAX_S               6.7f    // Inrush time is at most 6.7 s when TINRUSH = 0xFFFF.
#define DRV8214_TINRUSH_S_PER_LSB           (DRV8214_TINRUSH_MAX_S / ((float)UINT16_MAX)) // When EN_SS is reset, Inrush time goes from 5 ms when TINRUSH = 0x0000 to 6.7 s when TINRUSH = 0xFFFF. When EN_SS is set, inrush time will be multiplied by a factor of WSET_VSET.

// Configuration options enumerations

typedef enum Drv8214FilterType {
    DRV8214_ANALOG = 0b0,
    DRV8214_DIGITAL = 0b1
} Drv8214FilterType;

typedef enum Drv8214RegulationVoltageRange {
    DRV8214_0_V_TO_15_7_V = 0b0,
    DRV8214_0_V_TO_3_82_V = 0b1
} Drv8214RegulationVoltageRange;

typedef enum Drv8214CurrentRegBehavior {
    DRV8214_CURRENT_REG_DISABLED = 0b00,
    DRV8214_CURRENT_REG_WHILE_INRUSH = 0b01, // Work only if stall detection is also enabled
    DRV8214_CURRENT_REG_ENABLED = 0b10
} Drv8214CurrentRegBehavior;

typedef enum Drv8214StallBehavour {
    DRV8214_STALL_CURRENT_CUTOUT = 0b0,
    DRV8214_STALL_INDICATION_ONLY = 0b1
} Drv8214StallBehavour;

typedef enum Drv8214FixRefVoltage {
    DRV8214_REF_VOLTAGE_NOT_FIXED = 0b0,
    DRV8214_FIX_REF_VOLTAGE_TO_0_5_V = 0b1
} Drv8214FixRefVoltage;

typedef enum Drv8214BlankTime {
    DRV8214_BLANK_TIME_1_8_US = 0b0,
    DRV8214_BLANK_TIME_1_0_US = 0b1
} Drv8214BlankTime;

typedef enum Drv8214DeglitchTime {
    DRV8214_DEGLITCH_TIME_2_US = 0b0,
    DRV8214_DEGLITCH_TIME_1_US = 0b1
} Drv8214DeglitchTime;

typedef enum Drv8214AutoRetry {
    DRV8214_AUTO_RETRY = 0b0,
    DRV8214_LATCHED_OFF = 0b1
} Drv8214AutoRetry;

typedef enum Drv8214RippleCounterReporting {
    DRV8214_RC_REP_NO_NFAULT_RC_CNT_CEILLED = 0b00,             // Ripple counting has no effect on nFAULT. The value of RC_CNT is ceilled at 0xFFFF.
    DRV8214_RC_REP_NO_NFAULT_RC_CNT_OVERFLOWABLE = 0b01,        // Ripple counting has no effect on nFAULT. The value of RC_CNT is allowed to overflow from 0xFFFF to 0x0000.
    DRV8214_RC_REP_NFAULT_LOW_RC_CNT_CEILLED = 0b10,            // nFAULT is pulled low if RC_CNT exceeds threshold. The value of RC_CNT is ceilled at 0xFFFF.
    DRV8214_RC_REP_NFAULT_LOW_50_US_RC_CNT_OVERFLOWABLE = 0b11  // nFAULT is pulled low for 50 μs if RC_CNT reach 0xFFFF. The value of RC_CNT is allowed to overflow from 0xFFFF to 0x0000.
} Drv8214RippleCounterReporting;

typedef enum Drv8214ControlMode {
    DRV8214_PH_EN_MODE = 0b0,
    DRV8214_PWM_MODE = 0b1
} Drv8214ControlMode;

typedef enum Drv8214BridgeControlSource {
    DRV8214_BRIDGE_CONTROL_BY_INX_PINS = 0b0,
    DRV8214_BRIDGE_CONTROL_BY_I2C_BITS = 0b1
} Drv8214BridgeControlSource;

typedef enum Drv8214BridgePhEnPolarity {
    DRV8214_PH_EN_BRAKE = 0b00,
    DRV8214_PH_EN_REVERSE = 0b10,
    DRV8214_PH_EN_FORWARD = 0b11,
    DRV8214_PH_EN_NA = 0xFF
} Drv8214BridgePhEnPolarity;

typedef enum Drv8214BridgePwmPolarity {
    DRV8214_PWM_COAST = 0b00,
    DRV8214_PWM_REVERSE = 0b01,
    DRV8214_PWM_FORWARD = 0b10,
    DRV8214_PWM_BRAKE = 0b11,
    DRV8214_PWM_NA = 0xFF
} Drv8214BridgePwmPolarity;

// Getters

bool drv8214_is_driver_outputs_enabled(Drv8214 *driver);
bool drv8214_is_overvoltage_protection_enabled(Drv8214 *driver);
bool drv8214_is_stall_enabled(Drv8214 *driver);
Drv8214FilterType drv8214_get_filter_type(Drv8214 *driver);
Drv8214RegulationVoltageRange drv8214_get_regulation_range(Drv8214 *driver);
bool drv8214_is_manual_pwm_enabled(Drv8214 *driver);
float drv8214_get_inrush_duration(Drv8214 *driver);
Drv8214CurrentRegBehavior drv8214_get_current_regulation_behavior(Drv8214 *driver);
Drv8214StallBehavour drv8214_get_stall_behaviour(Drv8214 *driver);
Drv8214FixRefVoltage drv8214_get_internal_ref_voltage(Drv8214 *driver);
Drv8214BlankTime drv8214_get_current_blanking_time(Drv8214 *driver);
Drv8214DeglitchTime drv8214_get_deglitch_time(Drv8214 *driver);
Drv8214AutoRetry drv8214_get_overcurrent_protection_mode(Drv8214 *driver);
Drv8214AutoRetry drv8214_get_thermal_shutdown_mode(Drv8214 *driver);
Drv8214RippleCounterReporting drv8214_get_ripple_counter_reporting(Drv8214 *driver);
bool drv8214_get_stall_reporting(Drv8214 *driver);
bool drv8214_get_hbridge_current_regultion_reporting(Drv8214 *driver);
Drv8214ControlMode drv8214_get_control_mode(Drv8214 *driver);
Drv8214BridgeControlSource drv8214_get_i2c_bridge_control(Drv8214 *driver);
Drv8214BridgePhEnPolarity drv8214_get_i2c_ph_en_mode(Drv8214 *driver);
Drv8214BridgePwmPolarity drv8214_get_i2c_pwm_mode(Drv8214 *driver);

// Setters

/**
 * @brief Enable or disable the driver output.
 * @param driver Handle to the driver structure.
 * @param state State of the ouput.
 */
void drv8214_set_driver_outputs_enabled(Drv8214 *driver, bool state);

/**
 * @brief Enable or disable the over-voltage protection.
 * @param driver Handle to the driver structure.
 * @param state State of the over-voltage protection.
 */
void drv8214_set_overvoltage_protection_enabled(Drv8214 *driver, bool state);

/**
 * @brief Enable or disable stall detection.
 * @param driver Handle to the driver structure.
 * @param state State of the stall detection.
 */
void drv8214_set_stall_detection_enabled(Drv8214 *driver, bool state);

/**
 * @brief Allow to change low-pass filter used for regulation between analog and digital.
 * @param driver Handle to the driver structure.
 * @param filterType Type of filter to use.
 */
void drv8214_set_filter_type(Drv8214 *driver, Drv8214FilterType filterType);

/**
 * @brief Change the voltage range used during small voltage regulation.
 * @param driver Handle to the driver structure.
 * @param voltageRange Voltage range to use for small voltage regulation.
 */
void drv8214_set_regulation_range(Drv8214 *driver, Drv8214RegulationVoltageRange voltageRange);

/**
 * @brief Reset the ripple counter.
 * @param driver Handle to the driver structure.
 */
void drv8214_reset_ripple_counter(Drv8214 *driver);

/**
 * @brief Clear all faults.
 * @param driver Handle to the driver structure.
 */
void drv8214_clear_faults(Drv8214 *driver);

/**
 * @brief Enable or disable manual PWM control.
 * @param driver Handle to the driver structure.
 * @param state State of the manual PWM control.
 */
void drv8214_set_manual_pwm_enabled(Drv8214 *driver, bool state);

/**
 * @brief Set the inrush duration for which stall detection is ignored.
 * @param driver Handle to the driver structure.
 * @param duration Duration in seconds.
 */
void drv8214_set_inrush_duration(Drv8214 *driver, float duration);

/**
 * @brief Set the current regulation behavior.
 * @param driver Handle to the driver structure.
 * @param currentRegBehavior Current regulation behavior to be used.
 */
void drv8214_set_current_regulation_behavior(Drv8214 *driver, Drv8214CurrentRegBehavior currentRegBehavior);

/**
 * @brief Set the stall behavior.
 * @param driver Handle to the driver structure.
 * @param stallBehaviour Stall behavior to be used.
 */
void drv8214_set_stall_behavior(Drv8214 *driver, Drv8214StallBehavour stallBehaviour);

/**
 * @brief Allow to fix the internal reference voltage to 0.5 V.
 * @param driver Handle to the driver structure.
 * @param fixRefVoltage State of the internal reference voltage.
 */
void drv8214_set_internal_ref_voltage(Drv8214 *driver, Drv8214FixRefVoltage fixRefVoltage);

/**
 * @brief Set the current sense blanking time.
 * @param driver Handle to the driver structure.
 * @param blankTime Blanking time to be used.
 */
void drv8214_set_current_blanking_time(Drv8214 *driver, Drv8214BlankTime blankTime);

/**
 * @brief Set the current regulation and stall detection deglitch time.
 * @param driver Handle to the driver structure.
 * @param deglitchTime Deglitch time to be used.
 */
void drv8214_set_deglitch_time(Drv8214 *driver, Drv8214DeglitchTime deglitchTime);

/**
 * @brief Set the driver response to an over-current event.
 * @param driver Handle to the driver structure.
 * @param autoRetry Behavior in case of an over-current event.
 */
void drv8214_set_overcurrent_protection_mode(Drv8214 *driver, Drv8214AutoRetry autoRetry);

/**
 * @brief Set the driver response to an thermal shut down event.
 * @param driver Handle to the driver structure.
 * @param autoRetry Behavior in case of an thermal shut down event.
 */
void drv8214_set_thermal_shutdown_mode(Drv8214 *driver, Drv8214AutoRetry autoRetry);

/**
 * @brief Set the ripple counter behavior when reaching its threshold or its maximum.
 * @param driver Handle to the driver structure.
 * @param rippleCoutingReporting Ripple counter reporting mode.
 */
void drv8214_set_ripple_counter_reporting(Drv8214 *driver, Drv8214RippleCounterReporting rippleCoutingReporting);

/**
 * @brief Determine whether stall is reported on the nFAULT pin.
 * @param driver Handle to the driver structure.
 * @param state State of stall reporting on nFAULT pin.
 */
void drv8214_set_stall_reporting_enabled(Drv8214 *driver, bool state);

/**
 * @brief When in cycle-by-cycle mode of current regulation, enable or disable reporting of internal current regulation on nFAULT pin.
 * @param driver Handle to the driver structure.
 * @param state State of internal current regulation reporting.
 */
void drv8214_set_hbridge_current_regultion_reporting(Drv8214 *driver, bool state);

/**
 * @brief Allow to use phase / enable or PWM mode to control the motor.
 * @param driver Handle to the driver structure.
 * @param controlMode Control mode to by used on the motor.
 */
void drv8214_set_control_mode(Drv8214 *driver, Drv8214ControlMode controlMode);

/**
 * @brief Allow to control the bridge from external pins of from I2C registers.
 * @param driver Handle to the driver structure.
 * @param bridgeControl Type of bridge control to use.
 */
void drv8214_set_bridge_control_source(Drv8214 *driver, Drv8214BridgeControlSource bridgeControl);

/**
 * @brief When in phase / enable mode, set the bridge polarity. Ignored otherwise.
 * @param driver Handle to the driver structure.
 * @param phEnBridgePolarity Polarity to use on the bridge.
 */
void drv8214_set_bridge_ph_en_polarity(Drv8214 *driver, Drv8214BridgePhEnPolarity phEnBridgePolarity);

/**
 * @brief When in PWM mode, set the bridge polarity. Ignored otherwise.
 * @param driver Handle to the driver structure.
 * @param pwmBridgePolarity Polarity to use on the bridge.
 */
void drv8214_set_bridge_pwm_polarity(Drv8214 *driver, Drv8214BridgePwmPolarity pwmBridgePolarity);

#ifdef __cplusplus
}
#endif

#endif // DRV8214_CONFIG_H
