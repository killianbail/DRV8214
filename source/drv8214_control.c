/**
 * @file drv8214_control.c
 * @author Killian Baillifard
 * @date 09.10.2025
 * @brief Control registers access. These registers are read / write.
 */

// Includes

#include <math.h>
#include "drv8214_control.h"
#include "drv8214_config.h"
#include "drv8214_regedit.h"

// Implementations

bool drv8214_is_soft_start_stop_enabled(Drv8214 *driver) {
    return drv8214_read_flags(driver, DRV8214_REG_CTRL0, DRV8214_REG_CTRL0_EN_SS);
}

Drv8214RegulationScheme drv8214_get_regulation_scheme(Drv8214 *driver) {
    return drv8214_masked_read(driver, DRV8214_REG_CTRL0, DRV8214_REG_CTRL0_REG_CTRL) >> DRV8214_REG_CTRL0_REG_CTRL_SHIFT;
}

Drv8214PwmFreq drv8214_get_pwm_frequency(Drv8214 *driver) {
    return drv8214_masked_read(driver, DRV8214_REG_CTRL0, DRV8214_REG_CTRL0_PWM_FREQ) >> DRV8214_REG_CTRL0_PWM_FREQ_SHIFT;
}

Drv8214SpeedScale drv8214_get_speed_scaling_factor(Drv8214 *driver) {
    return drv8214_masked_read(driver, DRV8214_REG_CTRL0, DRV8214_REG_CTRL0_W_SCALE);
}

float drv8214_get_target_motor_voltage(Drv8214 *driver) {
    if(drv8214_get_regulation_scheme(driver) != DRV8214_REG_CTRL_VOLTAGE_REG)
        return NAN;
    const float TARGET = (float)drv8214_masked_read(driver, DRV8214_REG_CTRL1, DRV8214_REG_CTRL1_WSET_VSET);
    switch(drv8214_get_regulation_range(driver)) {
        case DRV8214_0_V_TO_15_7_V: return TARGET * (255.0f / 15.7f);
        case DRV8214_0_V_TO_3_82_V: return TARGET * (255.0f / 3.92f);
        default:                    return NAN;
    }
}

float drv8214_get_target_motor_speed(Drv8214 *driver) {
    if(drv8214_get_regulation_scheme(driver) != DRV8214_REG_CTRL_SPEED_REG)
        return NAN;
    const float TARGET = (float)drv8214_masked_read(driver, DRV8214_REG_CTRL1, DRV8214_REG_CTRL1_WSET_VSET);
    switch(drv8214_get_speed_scaling_factor(driver)) {
        case DRV8214_W_SCALE_16:    return TARGET * 16.0f;
        case DRV8214_W_SCALE_32:    return TARGET * 32.0f;
        case DRV8214_W_SCALE_64:    return TARGET * 64.0f;
        case DRV8214_W_SCALE_128:   return TARGET * 128.0f;
        default:                    return NAN;
    }
}

Drv8214CutoffFreq drv8214_get_voltage_lowpass_cutoff(Drv8214 *driver) {
    return drv8214_masked_read(driver, DRV8214_REG_CTRL2, DRV8214_REG_CTRL2_OUT_FLT) >> DRV8214_REG_CTRL2_OUT_FLT_SHIFT;
}

float drv8214_get_manual_duty_cycle(Drv8214 *driver) {
    return ((float)drv8214_masked_read(driver, DRV8214_REG_CTRL2, DRV8214_REG_CTRL2_EXT_DUTY)) * DRV8214_EXT_DUTY_PERCENT_PER_LSB;
}

bool drv8214_is_ripple_counting_enabled(Drv8214 *driver) {
    return drv8214_read_flags(driver, DRV8214_RC_CTRL0, DRV8214_RC_CTRL0_EN_RC);
}

bool drv8214_is_error_correction_disabled(Drv8214 *driver) {
    return drv8214_read_flags(driver, DRV8214_RC_CTRL0, DRV8214_RC_CTRL0_DIS_EC);
}

bool drv8214_is_bridge_cutoff_on_threshold_enabled(Drv8214 *driver) {
    return drv8214_read_flags(driver, DRV8214_RC_CTRL0, DRV8214_RC_CTRL0_RC_HIZ);
}

Drv8214FilterScale drv8214_get_filter_input_scaling(Drv8214 *driver) {
    return drv8214_masked_read(driver, DRV8214_RC_CTRL0, DRV8214_RC_CTRL0_FLT_GAIN_SEL) >> DRV8214_RC_CTRL0_FLT_GAIN_SEL_SHIFT;
}

Drv8214CsGainSel drv8214_get_current_mirror_gain(Drv8214 *driver) {
    return drv8214_masked_read(driver, DRV8214_RC_CTRL0, DRV8214_RC_CTRL0_CS_GAIN_SEL);
}

void drv8214_get_ripple_counter_threshold(Drv8214 *driver, uint16_t *threshold, Drv8214RippleCounterThresholdScale *scale) {
    *threshold = 0;
    *threshold |= drv8214_masked_read(driver, DRV8214_RC_CTRL1, DRV8214_RC_CTRL1_RC_THR);
    *threshold |= drv8214_masked_read(driver, DRV8214_RC_CTRL2, DRV8214_RC_CTRL2_RC_THR) << 8;
    *scale = drv8214_masked_read(driver, DRV8214_RC_CTRL2, DRV8214_RC_CTRL2_RC_THR_SCALE) >> DRV8214_RC_CTRL2_RC_THR_SCALE_SHIFT;
}

void drv8214_get_motor_resistance(Drv8214 *driver, uint8_t *inverseResistance, Drv8214InverseMotorResistanceScale *scale) {
    *inverseResistance = drv8214_masked_read(driver, DRV8214_RC_CTRL3, DRV8214_RC_CTRL3_INV_R);
    *scale = drv8214_masked_read(driver, DRV8214_RC_CTRL2, DRV8214_RC_CTRL2_INV_R_SCALE) >> DRV8214_RC_CTRL2_INV_R_SCALE_SHIFT;
}

void drv8214_get_motor_constant(Drv8214 *driver, uint8_t *motorConstant, Drv8214MotorConstantScale *scale) {
    *motorConstant = drv8214_masked_read(driver, DRV8214_RC_CTRL4, DRV8214_RC_CTRL4_KMC);
    *scale = drv8214_masked_read(driver, DRV8214_RC_CTRL2, DRV8214_RC_CTRL2_KMC_SCALE) >> DRV8214_RC_CTRL2_KMC_SCALE_SHIFT;
}

uint8_t drv8214_get_bandpass_quality(Drv8214 *driver) {
    return drv8214_masked_read(driver, DRV8214_RC_CTRL5, DRV8214_RC_CTRL5_FLT_K) >> DRV8214_RC_CTRL5_FLT_K_SHIFT;
}

bool drv8214_is_error_correction_pulse_disabled(Drv8214 *driver) {
    return drv8214_read_flags(driver, DRV8214_RC_CTRL6, DRV8214_RC_CTRL6_EC_PULSE_DIS);
}

uint8_t drv8214_get_ripple_lowpass_cutoff(Drv8214 *driver) {
    return drv8214_masked_read(driver, DRV8214_RC_CTRL6, DRV8214_RC_CTRL6_T_MECH_FLT) >> DRV8214_RC_CTRL6_T_MECH_FLT_SHIFT;
}

Drv8214ErrorCorrectorWindow drv8214_get_error_correction_false_window(Drv8214 *driver) {
    return drv8214_masked_read(driver, DRV8214_RC_CTRL6, DRV8214_RC_CTRL6_EC_FALSE_PER) >> DRV8214_RC_CTRL6_EC_FALSE_PER_SHIFT;
}

Drv8214ErrorCorrectorWindow drv8214_get_error_correction_miss_window(Drv8214 *driver) {
    return drv8214_masked_read(driver, DRV8214_RC_CTRL6, DRV8214_RC_CTRL6_EC_MISS_PER);
}

void drv8214_get_pi_coefficients(Drv8214 *driver, uint8_t *kpNum, uint8_t *kpDen, uint8_t *kiNum, uint8_t *kiDen) {
    *kpNum = drv8214_masked_read(driver, DRV8214_RC_CTRL7, DRV8214_RC_CTRL7_KP);
    *kpDen = drv8214_masked_read(driver, DRV8214_RC_CTRL7, DRV8214_RC_CTRL7_KP_DIV) >> DRV8214_RC_CTRL7_KP_DIV_SHIFT;
    *kiNum = drv8214_masked_read(driver, DRV8214_RC_CTRL8, DRV8214_RC_CTRL8_KI);
    *kiDen = drv8214_masked_read(driver, DRV8214_RC_CTRL8, DRV8214_RC_CTRL8_KI_DIV) >> DRV8214_RC_CTRL8_KI_DIV_SHIFT;
}

void drv8214_set_soft_start_stop_enabled(Drv8214 *driver, bool state) {
    drv8214_write_flags(driver, DRV8214_REG_CTRL0, DRV8214_REG_CTRL0_EN_SS, state);
}

void drv8214_set_regulation_scheme(Drv8214 *driver, Drv8214RegulationScheme currentRegulationScheme) {
    drv8214_masked_write(driver, DRV8214_REG_CTRL0, DRV8214_REG_CTRL0_REG_CTRL, currentRegulationScheme << DRV8214_REG_CTRL0_REG_CTRL_SHIFT);
}

void drv8214_set_pwm_frequency(Drv8214 *driver, Drv8214PwmFreq pwmFreq) {
    drv8214_masked_write(driver, DRV8214_REG_CTRL0, DRV8214_REG_CTRL0_PWM_FREQ, pwmFreq << DRV8214_REG_CTRL0_PWM_FREQ_SHIFT);
}

void drv8214_set_speed_scaling_factor(Drv8214 *driver, Drv8214SpeedScale speedScale) {
    drv8214_masked_write(driver, DRV8214_REG_CTRL0, DRV8214_REG_CTRL0_W_SCALE, speedScale);
}

void drv8214_set_target_motor_voltage(Drv8214 *driver, float voltage) {
    if(drv8214_get_regulation_scheme(driver) != DRV8214_REG_CTRL_VOLTAGE_REG)
        return;
    if(voltage < 0.0f)
        voltage = 0.0f;
    if(voltage > 15.7f)
        voltage = 15.7f;
    uint8_t target = 0.0f;
    switch(drv8214_get_regulation_range(driver)) {
        case DRV8214_0_V_TO_15_7_V: target = (uint8_t)(voltage * (15.7f / 255.0f));    break;
        case DRV8214_0_V_TO_3_82_V: target = (uint8_t)(voltage * (3.82f / 255.0f));    break;
    }
    drv8214_masked_write(driver, DRV8214_REG_CTRL1, DRV8214_REG_CTRL1_WSET_VSET, target);
}

void drv8214_set_target_ripple_speed(Drv8214 *driver, float speed) {
    if(drv8214_get_regulation_scheme(driver) != DRV8214_REG_CTRL_SPEED_REG)
        return;
    if(speed < 0.0f)
        speed = 0.0f;
    uint8_t target = 0.0f;
    switch(drv8214_get_speed_scaling_factor(driver)) {
        case DRV8214_W_SCALE_16:    target = (uint8_t)(speed / 16.0f);
        case DRV8214_W_SCALE_32:    target = (uint8_t)(speed / 32.0f);
        case DRV8214_W_SCALE_64:    target = (uint8_t)(speed / 64.0f);
        case DRV8214_W_SCALE_128:   target = (uint8_t)(speed / 128.0f);
    }
    drv8214_masked_write(driver, DRV8214_REG_CTRL1, DRV8214_REG_CTRL1_WSET_VSET, target);
}

void drv8214_set_voltage_lowpass_cutoff(Drv8214 *driver, Drv8214CutoffFreq cutoffFreq) {
    drv8214_masked_write(driver, DRV8214_REG_CTRL2, DRV8214_REG_CTRL2_OUT_FLT, cutoffFreq << DRV8214_REG_CTRL2_OUT_FLT_SHIFT);
}

void drv8214_set_manual_duty_cycle(Drv8214 *driver, float percentage) {
    if(percentage < 0.0f)
        percentage = 0.0f;
    if(percentage > 100.0f)
        percentage = 100.0f;
    drv8214_masked_write(driver, DRV8214_REG_CTRL2, DRV8214_REG_CTRL2_EXT_DUTY, (uint8_t)(percentage / DRV8214_EXT_DUTY_PERCENT_PER_LSB));
}

void drv8214_set_ripple_counting_enabled(Drv8214 *driver, bool state) {
    drv8214_write_flags(driver, DRV8214_RC_CTRL0, DRV8214_RC_CTRL0_EN_RC, state);
}

void drv8214_set_error_correction_disabled(Drv8214 *driver, bool state) {
    drv8214_write_flags(driver, DRV8214_RC_CTRL0, DRV8214_RC_CTRL0_DIS_EC, state);
}

void drv8214_set_bridge_cutoff_on_threshold_enabled(Drv8214 *driver, bool state) {
    drv8214_write_flags(driver, DRV8214_RC_CTRL0, DRV8214_RC_CTRL0_RC_HIZ, state);
}

void drv8214_set_current_ripples_scaling_factor(Drv8214 *driver, Drv8214FilterScale filterScale) {
    drv8214_masked_write(driver, DRV8214_RC_CTRL0, DRV8214_RC_CTRL0_FLT_GAIN_SEL, filterScale << DRV8214_RC_CTRL0_FLT_GAIN_SEL_SHIFT);
}

void drv8214_set_current_mirror_gain(Drv8214 *driver, Drv8214CsGainSel csGainSel) {
    drv8214_masked_write(driver, DRV8214_RC_CTRL0, DRV8214_RC_CTRL0_CS_GAIN_SEL, csGainSel);
}

uint16_t drv8214_set_ripple_counter_threshold(Drv8214 *driver, uint16_t target) {

    // Available options
    static const uint32_t THRESHOLD_SCALE_VALUES[4] = { 0x07FFu, 0x1FFFu, 0x3FFFu, 0xFFFFu };
    static const uint8_t THRESHOLD_SHIFTS[4] = { 1u, 3u, 4u, 6u };
    static const Drv8214RippleCounterThresholdScale THRESHOLD_SCALE_MASKS[4] = {
        DRV8214_RC_THR_SCALE_2,
        DRV8214_RC_THR_SCALE_8,
        DRV8214_RC_THR_SCALE_16,
        DRV8214_RC_THR_SCALE_64
    };

    // Allocate local variables
    Drv8214RippleCounterThresholdScale thresholdScaleMask = DRV8214_RC_THR_SCALE_2;
    uint8_t thresholdShift = 1;
    bool ceil = false;

    // Find closest applicable threshold
    for(uint8_t i = 0; i < 4; ++i) {
        if(target <= THRESHOLD_SCALE_VALUES[i]) {
            thresholdScaleMask = THRESHOLD_SCALE_MASKS[i];
            thresholdShift = THRESHOLD_SHIFTS[i];
            uint16_t mask = (1u << thresholdShift) - 1u;
            uint16_t half = (1u << (thresholdShift - 1u));
            ceil = ( (target & mask) >= half );
            break;
        }
    }

    // Shift and round threshold to closest value
    uint16_t shiftedThreshold = target >> thresholdShift;
    if((shiftedThreshold < 0b1111111111) && ceil)
        ++shiftedThreshold;

    // Apply threshold and scale
    drv8214_masked_write(driver, DRV8214_RC_CTRL1, DRV8214_RC_CTRL1_RC_THR, shiftedThreshold & 0x00FF);
    drv8214_masked_write(driver, DRV8214_RC_CTRL2, DRV8214_RC_CTRL2_RC_THR, (shiftedThreshold & 0xFF00) >> 8);
    drv8214_masked_write(driver, DRV8214_RC_CTRL2, DRV8214_RC_CTRL2_RC_THR_SCALE, thresholdScaleMask << DRV8214_RC_CTRL2_RC_THR_SCALE_SHIFT);
    return shiftedThreshold << thresholdShift;
}

void drv8214_set_motor_resistance(Drv8214 *driver, float resistance) {

    // Available options
    static const float NUMERATORS_VALUES[4] = {8192.0f, 1024.0f, 64.0f, 2.0f};
    static const Drv8214InverseMotorResistanceScale NUMERATORS_MASKS[4] = {
        DRV8214_INV_R_SCALE_8192,
        DRV8214_INV_R_SCALE_1024,
        DRV8214_INV_R_SCALE_64,
        DRV8214_INV_R_SCALE_2
    };

    // Clamp out of bounds values
    if(resistance > (8192.0f / 1.0f))
        resistance = (8192.0f / 1.0f);
    if(resistance < (2.0f / 255.0f))
        resistance = (2.0f / 255.0f);

    // Find optimal range
    for(uint8_t i = 0; i < 4; ++i) {
        float denominator = roundf(NUMERATORS_VALUES[i] / resistance);
        if(denominator <= 255.0f) {
            drv8214_masked_write(driver, DRV8214_RC_CTRL3, DRV8214_RC_CTRL3_INV_R, (uint8_t)denominator);
            drv8214_masked_write(driver, DRV8214_RC_CTRL2, DRV8214_RC_CTRL2_INV_R_SCALE, NUMERATORS_MASKS[i] << DRV8214_RC_CTRL2_INV_R_SCALE_SHIFT);
            return;
        }
    }
}

void drv8214_set_motor_constant(Drv8214 *driver, float speedConstant, float ripplesPerRevolution) {

    // Available options
    static const Drv8214MotorConstantScale DENOMINATOR_MASKS[4] = {
        DRV8214_KMC_SCALE_24_TIME_2_POW_13,
        DRV8214_KMC_SCALE_24_TIME_2_POW_12,
        DRV8214_KMC_SCALE_24_TIME_2_POW_9,
        DRV8214_KMC_SCALE_24_TIME_2_POW_8
    };
    static const float DENOMINATOR_VALUES[4] = {
        24.0f * (float)(1 << 13),
        24.0f * (float)(1 << 12),
        24.0f * (float)(1 << 9),
        24.0f * (float)(1 << 8)
    };
    
    // Clamp out of bounds values
    float kmc = speedConstant / ripplesPerRevolution;
    if(kmc < 0)
        kmc = 0;
    if(kmc > (255.0f / DENOMINATOR_VALUES[3]))
        kmc = (255.0f / DENOMINATOR_VALUES[3]);

    // Find optimal range
    for(uint8_t i = 0; i < 4; ++i) {
        float numerator = roundf(kmc * DENOMINATOR_VALUES[i]);
        if(numerator <= 255.0f) {
            drv8214_masked_write(driver, DRV8214_RC_CTRL4, DRV8214_RC_CTRL4_KMC, (uint8_t)numerator);
            drv8214_masked_write(driver, DRV8214_RC_CTRL2, DRV8214_RC_CTRL2_KMC_SCALE, DENOMINATOR_MASKS[i] << DRV8214_RC_CTRL2_KMC_SCALE_SHIFT);
            return;
        }
    }
}

void drv8214_set_ripple_counter_bandpass_quality(Drv8214 *driver, Drv8214QFactor quality) {
    drv8214_masked_write(driver, DRV8214_RC_CTRL5, DRV8214_RC_CTRL5_FLT_K, quality << DRV8214_RC_CTRL5_FLT_K_SHIFT);
}

void drv8214_set_error_correction_pulse_disabled(Drv8214 *driver, bool state) {
    drv8214_write_flags(driver, DRV8214_RC_CTRL6, DRV8214_RC_CTRL6_EC_PULSE_DIS, state);
}

void drv8214_set_ripple_counter_out_lowpass_cutoff(Drv8214 *driver, uint8_t value) {
    drv8214_masked_write(driver, DRV8214_RC_CTRL6, DRV8214_RC_CTRL6_T_MECH_FLT, value << DRV8214_RC_CTRL6_T_MECH_FLT_SHIFT);
}

void drv8214_set_error_correction_false_window(Drv8214 *driver, Drv8214ErrorCorrectorWindow window) {
    drv8214_masked_write(driver, DRV8214_RC_CTRL6, DRV8214_RC_CTRL6_EC_FALSE_PER, window << DRV8214_RC_CTRL6_EC_FALSE_PER_SHIFT);
}

void drv8214_set_error_correction_miss_window(Drv8214 *driver, Drv8214ErrorCorrectorWindow window) {
    drv8214_masked_write(driver, DRV8214_RC_CTRL6, DRV8214_RC_CTRL6_EC_MISS_PER, window);
}

void drv8214_set_pi_coefficients(Drv8214 *driver, float kp, float ki) {

    // Available options
    static const float DENOMINATORS_VALUES[7] = {
        512.0f,
        256.0f,
        128.0f,
        64.0f,
        32.0f,
        16.0f,
        1.0f
    };
    static const Drv8214KDiv DENOMINATORS_MASKS[7] = {
        DRV8214_K_DIV_512,
        DRV8214_K_DIV_256,
        DRV8214_K_DIV_128,
        DRV8214_K_DIV_64,
        DRV8214_K_DIV_32,
        DRV8214_K_DIV_16,
        DRV8214_K_DIV_1
    };

    // Clamp values
    if(kp < 0)
        kp = 0;
    if(kp > (31.0f / DENOMINATORS_VALUES[6]))
        kp = (31.0f / DENOMINATORS_VALUES[6]);
    if(ki < 0)
        ki = 0;
    if(ki > (31.0f / DENOMINATORS_VALUES[6]))
        ki = (31.0f / DENOMINATORS_VALUES[6]);

    // Find optimal KP
    for(uint8_t i = 0; i < 7; ++i) {
        float numerator = roundf(kp * DENOMINATORS_VALUES[i]);
        if(numerator <= 31.0f) {
            drv8214_masked_write(driver, DRV8214_RC_CTRL7, DRV8214_RC_CTRL7_KP, (uint8_t)numerator);
            drv8214_masked_write(driver, DRV8214_RC_CTRL7, DRV8214_RC_CTRL7_KP_DIV, DENOMINATORS_MASKS[i] << DRV8214_RC_CTRL7_KP_DIV_SHIFT);
            break;
        }
    }

    // Find optimal KI
    for(uint8_t i = 0; i < 7; ++i) {
        float numerator = roundf(ki * DENOMINATORS_VALUES[i]);
        if(numerator <= 31.0f) {
            drv8214_masked_write(driver, DRV8214_RC_CTRL8, DRV8214_RC_CTRL8_KI, (uint8_t)numerator);
            drv8214_masked_write(driver, DRV8214_RC_CTRL8, DRV8214_RC_CTRL8_KI_DIV, DENOMINATORS_MASKS[i] << DRV8214_RC_CTRL8_KI_DIV_SHIFT);
            break;
        }
    }
}
