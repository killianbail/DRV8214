/**
 * @file drv8214_config.c
 * @author Killian Baillifard
 * @date 09.10.2025
 * @brief Configuration registers access. These registers are read / write.
 */

// Includes

#include <math.h>
#include "drv8214_config.h"
#include "drv8214_control.h"
#include "drv8214_regedit.h"

// Implementations

bool drv8214_is_driver_outputs_enabled(Drv8214 *driver) {
    return drv8214_read_flags(driver, DRV8214_CONFIG0, DRV8214_CONFIG0_EN_OUT);
}

bool drv8214_is_overvoltage_protection_enabled(Drv8214 *driver) {
    return drv8214_read_flags(driver, DRV8214_CONFIG0, DRV8214_CONFIG0_EN_OVP);
}

bool drv8214_is_stall_enabled(Drv8214 *driver) {
    return drv8214_read_flags(driver, DRV8214_CONFIG0, DRV8214_CONFIG0_EN_STALL);
}

Drv8214FilterType drv8214_get_filter_type(Drv8214 *driver) {
    return drv8214_masked_read(driver, DRV8214_CONFIG0, DRV8214_CONFIG0_VSNS_SEL) >> DRV8214_CONFIG0_VSNS_SEL_SHIFT;
}

Drv8214RegulationVoltageRange drv8214_get_regulation_range(Drv8214 *driver) {
    return drv8214_masked_read(driver, DRV8214_CONFIG0, DRV8214_CONFIG0_VM_GAIN_SEL) >> DRV8214_CONFIG0_VM_GAIN_SEL_SHIFT;
}

bool drv8214_is_manual_pwm_enabled(Drv8214 *driver) {
    return drv8214_read_flags(driver, DRV8214_CONFIG0, DRV8214_CONFIG0_DUTY_CTRL);
}

float drv8214_get_inrush_duration(Drv8214 *driver) {

    // Read inrush value
    uint16_t inrush = 0;
    inrush |= drv8214_masked_read(driver, DRV8214_CONFIG1, DRV8214_CONFIG1_TINRUSH);
    inrush |= drv8214_masked_read(driver, DRV8214_CONFIG2, DRV8214_CONFIG2_TINRUSH) << 8;

    // First case, soft start / stop disabled, inrush time goes from 0x0000 -> 5 ms to 0xFFFF -> 6.7 ms.
    if(!drv8214_is_soft_start_stop_enabled(driver))
        return ((float)inrush) * DRV8214_TINRUSH_NO_SS_RESOLUTION + DRV8214_TINRUSH_NO_SS_MIN;
    
    // Second case, soft start / stop enabled, inrush time is badly defined between datasheet and TI forums
    else {
        return NAN;
    }
}

Drv8214CurrentRegBehavior drv8214_get_current_regulation_behavior(Drv8214 *driver) {
    return drv8214_masked_read(driver, DRV8214_CONFIG3, DRV8214_CONFIG3_IMODE) >> DRV8214_CONFIG3_IMODE_SHIFT;
}

Drv8214StallBehavour drv8214_get_stall_behaviour(Drv8214 *driver) {
    return drv8214_masked_read(driver, DRV8214_CONFIG3, DRV8214_CONFIG3_SMODE) >> DRV8214_CONFIG3_SMODE_SHIFT;
}

Drv8214FixRefVoltage drv8214_get_internal_ref_voltage(Drv8214 *driver) {
    return drv8214_masked_read(driver, DRV8214_CONFIG3, DRV8214_CONFIG3_INT_VREF) >> DRV8214_CONFIG3_INT_VREF_SHIFT;
}

Drv8214BlankTime drv8214_get_current_blanking_time(Drv8214 *driver) {
    return drv8214_masked_read(driver, DRV8214_CONFIG3, DRV8214_CONFIG3_TBLANK) >> DRV8214_CONFIG3_TBLANK_SHIFT;
}

Drv8214DeglitchTime drv8214_get_deglitch_time(Drv8214 *driver) {
    return drv8214_masked_read(driver, DRV8214_CONFIG3, DRV8214_CONFIG3_TDEG) >> DRV8214_CONFIG3_TDEG_SHIFT;
}

Drv8214AutoRetry drv8214_get_overcurrent_protection_mode(Drv8214 *driver) {
    return drv8214_masked_read(driver, DRV8214_CONFIG3, DRV8214_CONFIG3_OCP_MODE) >> DRV8214_CONFIG3_OCP_MODE_SHIFT;
}

Drv8214AutoRetry drv8214_get_thermal_shutdown_mode(Drv8214 *driver) {
    return drv8214_masked_read(driver, DRV8214_CONFIG3, DRV8214_CONFIG3_TSD_MODE);
}

Drv8214RippleCounterReporting drv8214_get_ripple_counter_reporting(Drv8214 *driver) {
    return drv8214_masked_read(driver, DRV8214_CONFIG4, DRV8214_CONFIG4_RC_REP) >> DRV8214_CONFIG4_RC_REP_SHIFT;
}

bool drv8214_get_stall_reporting(Drv8214 *driver) {
    return drv8214_read_flags(driver, DRV8214_CONFIG4, DRV8214_CONFIG4_STALL_REP);
}

bool drv8214_get_hbridge_current_regultion_reporting(Drv8214 *driver) {
    return drv8214_read_flags(driver, DRV8214_CONFIG4, DRV8214_CONFIG4_CBC_REP);
}

Drv8214ControlMode drv8214_get_control_mode(Drv8214 *driver) {
    return drv8214_masked_read(driver, DRV8214_CONFIG4, DRV8214_CONFIG4_PMODE) >> DRV8214_CONFIG4_PMODE_SHIFT;
}

Drv8214BridgeControlSource drv8214_get_i2c_bridge_control(Drv8214 *driver) {
    return drv8214_masked_read(driver, DRV8214_CONFIG4, DRV8214_CONFIG4_I2C_BC) >> DRV8214_CONFIG4_I2C_BC_SHIFT;
}

Drv8214BridgePhEnPolarity drv8214_get_i2c_ph_en_mode(Drv8214 *driver) {
    if(drv8214_get_i2c_bridge_control(driver) != DRV8214_BRIDGE_CONTROL_BY_I2C_BITS)
        return DRV8214_PH_EN_NA;
    if(drv8214_get_control_mode(driver) != DRV8214_PH_EN_MODE)
        return DRV8214_PH_EN_NA;
    return drv8214_masked_read(driver, DRV8214_CONFIG4, DRV8214_CONFIG4_I2C_BC_MODE);
}

Drv8214BridgePwmPolarity drv8214_get_i2c_pwm_mode(Drv8214 *driver) {
    if(drv8214_get_i2c_bridge_control(driver) != DRV8214_BRIDGE_CONTROL_BY_I2C_BITS)
        return DRV8214_PWM_NA;
    if(drv8214_get_control_mode(driver) != DRV8214_PWM_MODE)
        return DRV8214_PWM_NA;
    return drv8214_masked_read(driver, DRV8214_CONFIG4, DRV8214_CONFIG4_I2C_BC_MODE);
}

void drv8214_set_driver_outputs_enabled(Drv8214 *driver, bool state) {
    drv8214_write_flags(driver, DRV8214_CONFIG0, DRV8214_CONFIG0_EN_OUT, state);
}

void drv8214_set_overvoltage_protection_enabled(Drv8214 *driver, bool state) {
    drv8214_write_flags(driver, DRV8214_CONFIG0, DRV8214_CONFIG0_EN_OVP, state);
}

void drv8214_set_stall_detection_enabled(Drv8214 *driver, bool state) {
    drv8214_write_flags(driver, DRV8214_CONFIG0, DRV8214_CONFIG0_EN_STALL, state);
}

void drv8214_set_voltage_filter_type(Drv8214 *driver, Drv8214FilterType filterType) {
    drv8214_masked_write(driver, DRV8214_CONFIG0, DRV8214_CONFIG0_VSNS_SEL, filterType << DRV8214_CONFIG0_VSNS_SEL_SHIFT);
}

void drv8214_set_regulation_range(Drv8214 *driver, Drv8214RegulationVoltageRange voltageRange) {
    drv8214_masked_write(driver, DRV8214_CONFIG0, DRV8214_CONFIG0_VM_GAIN_SEL, voltageRange << DRV8214_CONFIG0_VM_GAIN_SEL_SHIFT);
}

void drv8214_reset_ripple_counter(Drv8214 *driver) {
    drv8214_write_flags(driver, DRV8214_CONFIG0, DRV8214_CONFIG0_CLR_CNT, true);
}

void drv8214_clear_faults(Drv8214 *driver) {
    drv8214_write_flags(driver, DRV8214_CONFIG0, DRV8214_CONFIG0_CLR_FLT, true);
}

void drv8214_set_manual_pwm_enabled(Drv8214 *driver, bool state) {
    drv8214_write_flags(driver, DRV8214_CONFIG0, DRV8214_CONFIG0_DUTY_CTRL, state);
}

void drv8214_set_inrush_duration(Drv8214 *driver, float duration) {

    // Compute inrush value
    if(duration < DRV8214_TINRUSH_NO_SS_MIN)
        duration = DRV8214_TINRUSH_NO_SS_MIN;
    if(duration > DRV8214_TINRUSH_NO_SS_MAX)
        duration = DRV8214_TINRUSH_NO_SS_MAX;
    uint16_t inrush = (uint16_t)((duration - DRV8214_TINRUSH_NO_SS_MIN) / DRV8214_TINRUSH_NO_SS_RESOLUTION);
    
    // Write inrush value
    drv8214_masked_write(driver, DRV8214_CONFIG1, DRV8214_CONFIG1_TINRUSH, (uint8_t)(inrush >> 0));
    drv8214_masked_write(driver, DRV8214_CONFIG2, DRV8214_CONFIG2_TINRUSH, (uint8_t)(inrush >> 8));
}

void drv8214_set_current_regulation_behavior(Drv8214 *driver, Drv8214CurrentRegBehavior currentRegBehavior) {
    drv8214_masked_write(driver, DRV8214_CONFIG3, DRV8214_CONFIG3_IMODE, currentRegBehavior << DRV8214_CONFIG3_IMODE_SHIFT);
}

void drv8214_set_stall_behavior(Drv8214 *driver, Drv8214StallBehavour stallBehaviour) {
    drv8214_masked_write(driver, DRV8214_CONFIG3, DRV8214_CONFIG3_SMODE, stallBehaviour << DRV8214_CONFIG3_SMODE_SHIFT);
}

void drv8214_set_internal_ref_voltage(Drv8214 *driver, Drv8214FixRefVoltage fixRefVoltage) {
    drv8214_masked_write(driver, DRV8214_CONFIG3, DRV8214_CONFIG3_INT_VREF, fixRefVoltage << DRV8214_CONFIG3_INT_VREF_SHIFT);
}

void drv8214_set_current_blanking_time(Drv8214 *driver, Drv8214BlankTime blankTime) {
    drv8214_masked_write(driver, DRV8214_CONFIG3, DRV8214_CONFIG3_TBLANK, blankTime << DRV8214_CONFIG3_TBLANK_SHIFT);
}

void drv8214_set_deglitch_time(Drv8214 *driver, Drv8214DeglitchTime deglitchTime) {
    drv8214_masked_write(driver, DRV8214_CONFIG3, DRV8214_CONFIG3_TDEG, deglitchTime << DRV8214_CONFIG3_TDEG_SHIFT);
}

void drv8214_set_overcurrent_protection_mode(Drv8214 *driver, Drv8214AutoRetry autoRetry) {
    drv8214_masked_write(driver, DRV8214_CONFIG3, DRV8214_CONFIG3_OCP_MODE, autoRetry << DRV8214_CONFIG3_OCP_MODE_SHIFT);
}

void drv8214_set_thermal_shutdown_mode(Drv8214 *driver, Drv8214AutoRetry autoRetry) {
    drv8214_masked_write(driver, DRV8214_CONFIG3, DRV8214_CONFIG3_TSD_MODE, autoRetry);
}

void drv8214_set_ripple_counter_reporting(Drv8214 *driver, Drv8214RippleCounterReporting rippleCoutingReporting) {
    drv8214_masked_write(driver, DRV8214_CONFIG4, DRV8214_CONFIG4_RC_REP, rippleCoutingReporting << DRV8214_CONFIG4_RC_REP_SHIFT);
}

void drv8214_set_stall_reporting_enabled(Drv8214 *driver, bool state) {
    drv8214_write_flags(driver, DRV8214_CONFIG4, DRV8214_CONFIG4_STALL_REP, state);
}

void drv8214_set_hbridge_current_regultion_reporting(Drv8214 *driver, bool state) {
    drv8214_write_flags(driver, DRV8214_CONFIG4, DRV8214_CONFIG4_CBC_REP, state);
}

void drv8214_set_control_mode(Drv8214 *driver, Drv8214ControlMode controlMode) {
    drv8214_masked_write(driver, DRV8214_CONFIG4, DRV8214_CONFIG4_PMODE, controlMode << DRV8214_CONFIG4_PMODE_SHIFT);
}

void drv8214_set_bridge_control_source(Drv8214 *driver, Drv8214BridgeControlSource bridgeControl) {
    drv8214_masked_write(driver, DRV8214_CONFIG4, DRV8214_CONFIG4_I2C_BC, bridgeControl << DRV8214_CONFIG4_I2C_BC_SHIFT);
}

void drv8214_set_bridge_ph_en_polarity(Drv8214 *driver, Drv8214BridgePhEnPolarity phEnBridgePolarity) {
    if(drv8214_get_i2c_bridge_control(driver) != DRV8214_BRIDGE_CONTROL_BY_I2C_BITS)
        return;
    if(drv8214_get_control_mode(driver) != DRV8214_PH_EN_MODE)
        return;
    drv8214_masked_write(driver, DRV8214_CONFIG4, DRV8214_CONFIG4_I2C_BC_MODE, phEnBridgePolarity);
}

void drv8214_set_bridge_pwm_polarity(Drv8214 *driver, Drv8214BridgePwmPolarity pwmBridgePolarity) {
    if(drv8214_get_i2c_bridge_control(driver) != DRV8214_BRIDGE_CONTROL_BY_I2C_BITS)
        return;
    if(drv8214_get_control_mode(driver) != DRV8214_PWM_MODE)
        return;
    drv8214_masked_write(driver, DRV8214_CONFIG4, DRV8214_CONFIG4_I2C_BC_MODE, pwmBridgePolarity);
}
