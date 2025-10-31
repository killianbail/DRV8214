/**
 * @file drv8214_control.h
 * @author Killian Baillifard
 * @date 09.10.2025
 * @brief Control registers access. These registers are read / write.
 */

#ifndef DRV8214_CONTROL_H
#define DRV8214_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

// Includes

#include "drv8214_platform.h"

// Control registers addresses

#define DRV8214_REG_CTRL0                   0x0E        // Regulation control register 0.
#define DRV8214_REG_CTRL1                   0x0F        // Regulation control register 1.
#define DRV8214_REG_CTRL2                   0x10        // Regulation control register 2.
#define DRV8214_RC_CTRL0                    0x11        // Ripple counting control register 0.
#define DRV8214_RC_CTRL1                    0x12        // Ripple counting control register 1.
#define DRV8214_RC_CTRL2                    0x13        // Ripple counting control register 2.
#define DRV8214_RC_CTRL3                    0x14        // Ripple counting control register 3.
#define DRV8214_RC_CTRL4                    0x15        // Ripple counting control register 4.
#define DRV8214_RC_CTRL5                    0x16        // Ripple counting control register 5.
#define DRV8214_RC_CTRL6                    0x17        // Ripple counting control register 6.
#define DRV8214_RC_CTRL7                    0x18        // Ripple counting control register 7.
#define DRV8214_RC_CTRL8                    0x19        // Ripple counting control register 8.

// Bit masks for control registers

#define DRV8214_REG_CTRL0_EN_SS             0b00100000  // Soft start / stop when set, disable it otherwise (set by default).
#define DRV8214_REG_CTRL0_REG_CTRL          0b00011000  // Current regulation scheme.
#define DRV8214_REG_CTRL0_PWM_FREQ          0b00000100  // PWM frequency is 25 kHz when set and 50 kHz when reset (set by default).
#define DRV8214_REG_CTRL0_W_SCALE           0b00000011  // Speed scaling factor used for target ripple speed.
#define DRV8214_REG_CTRL1_WSET_VSET         0b11111111  // Target motor voltage or ripple speed.
#define DRV8214_REG_CTRL2_OUT_FLT           0b11000000  // Program the cut-off frequency of the output voltage low-pass filter.
#define DRV8214_REG_CTRL2_EXT_DUTY          0b00111111  // Manual duty cycle when external bridge control is disabled and duty control is enabled.
#define DRV8214_RC_CTRL0_EN_RC              0b10000000  // Enable ripple counting when set, disable it otherwise (set by default).
#define DRV8214_RC_CTRL0_DIS_EC             0b01000000  // Disable error correction when set, enable it otherwise.
#define DRV8214_RC_CTRL0_RC_HIZ             0b00100000  // Disable bridge when ripple count exceed threshold when set, keep it enabled otherwise.
#define DRV8214_RC_CTRL0_FLT_GAIN_SEL       0b00011000  // Filter input scaling factor for current ripple detection.
#define DRV8214_RC_CTRL0_CS_GAIN_SEL        0b00000111  // Current mirror gain
#define DRV8214_RC_CTRL1_RC_THR             0b11111111  // 8 LSBs of the ripple counter threshold value.
#define DRV8214_RC_CTRL2_INV_R_SCALE        0b11000000  // Scaling factor of the inverse motor resistance.
#define DRV8214_RC_CTRL2_KMC_SCALE          0b00110000  // Scaling factor of the motor constant.
#define DRV8214_RC_CTRL2_RC_THR_SCALE       0b00001100  // Scaling factor of the ripple counter threshold value.
#define DRV8214_RC_CTRL2_RC_THR             0b00000011  // 2 MSBs of the ripple counter threshold value.
#define DRV8214_RC_CTRL3_INV_R              0b11111111  // Inverse resistance of the motor coil scaled (must not be set to 0).
#define DRV8214_RC_CTRL4_KMC                0b11111111  // Motor constant scaled.
#define DRV8214_RC_CTRL5_FLT_K              0b11110000  // Bandpass filter inverse quality factor, set its bandwidth.
#define DRV8214_RC_CTRL6_EC_PULSE_DIS       0b10000000  // Disable error correction pulses when set, does not otherwise.
#define DRV8214_RC_CTRL6_T_MECH_FLT         0b01110000  // Determine the cut-off frequency of the low-pass filter at the output of the ripple counter. Decreasing this value gives a faster response.
#define DRV8214_RC_CTRL6_EC_FALSE_PER       0b00001100  // Window during which the error corrector classifies a current ripple as an extra ripple.
#define DRV8214_RC_CTRL6_EC_MISS_PER        0b00000011  // Window during which the error corrector adds a missed ripple.
#define DRV8214_RC_CTRL7_KP_DIV             0b11100000  // Denominator of the proportional gain of the regulator.
#define DRV8214_RC_CTRL7_KP                 0b00011111  // Numerator of the proportional gain of the regulator.
#define DRV8214_RC_CTRL8_KI_DIV             0b11100000  // Denominator of the integral gain of the regulator.
#define DRV8214_RC_CTRL8_KI                 0b00011111  // Numerator of the integral gain of the regulator.

// Control options shifts

#define DRV8214_REG_CTRL0_REG_CTRL_SHIFT    3
#define DRV8214_REG_CTRL0_PWM_FREQ_SHIFT    2
#define DRV8214_REG_CTRL2_OUT_FLT_SHIFT     6
#define DRV8214_RC_CTRL0_FLT_GAIN_SEL_SHIFT 3
#define DRV8214_RC_CTRL2_INV_R_SCALE_SHIFT  6
#define DRV8214_RC_CTRL2_KMC_SCALE_SHIFT    4
#define DRV8214_RC_CTRL2_RC_THR_SCALE_SHIFT 2
#define DRV8214_RC_CTRL5_FLT_K_SHIFT        4
#define DRV8214_RC_CTRL6_T_MECH_FLT_SHIFT   4
#define DRV8214_RC_CTRL6_EC_FALSE_PER_SHIFT 2
#define DRV8214_RC_CTRL7_KP_DIV_SHIFT       5
#define DRV8214_RC_CTRL8_KI_DIV_SHIFT       5

// Control conversion constants

#define DRV8214_EXT_DUTY_PERCENT_PER_LSB    (100.0f / ((float)0b00111111))  // 0b000000 -> 0 % and 0b111111 -> 100 %.

// Control options enumerations

typedef enum Drv8214RegulationScheme {
    DRV8214_REG_CTRL_FIXED_OFF_TIME = 0b00,
    DRV8214_REG_CTRL_CYCLE_BY_CYCLE = 0b01,
    DRV8214_REG_CTRL_SPEED_REG = 0b10,
    DRV8214_REG_CTRL_VOLTAGE_REG = 0b11
} Drv8214RegulationScheme;

typedef enum Drv8214PwmFreq {
    DRV8214_PWM_FREQ_50_KHZ = false,
    DRV8214_PWM_FREQ_25_KHZ = true
} Drv8214PwmFreq;

typedef enum Drv8214SpeedScale {
    DRV8214_W_SCALE_16 = 0b00,
    DRV8214_W_SCALE_32 = 0b01,
    DRV8214_W_SCALE_64 = 0b10,
    DRV8214_W_SCALE_128 = 0b11
} Drv8214SpeedScale;

typedef enum Drv8214CutoffFreq {
    DRV8214_CUTOFF_FREQ_250_KHZ = 0b00,
    DRV8214_CUTOFF_FREQ_500_KHZ = 0b01,
    DRV8214_CUTOFF_FREQ_750_KHZ = 0b10,
    DRV8214_CUTOFF_FREQ_1000_KHZ = 0b11
} Drv8214CutoffFreq;

typedef enum Drv8214FilterScale {
    DRV8214_FILTER_SCALE_2 = 0b00,
    DRV8214_FILTER_SCALE_4 = 0b01,
    DRV8214_FILTER_SCALE_8 = 0b10,
    DRV8214_FILTER_SCALE_16 = 0b11
} Drv8214FilterScale;

typedef enum Drv8214CsGainSel {
    DRV8214_4_A_MAX_225_UA_PER_A = 0b000,
    DRV8214_2_A_MAX_225_UA_PER_A = 0b001,
    DRV8214_1_A_MAX_1125_UA_PER_A = 0b010,
    DRV8214_500_MA_MAX_1125_UA_PER_A = 0b011,
    DRV8214_250_MA_MAX_5560_UA_PER_A = 0b100,
    DRV8214_125_MA_MAX_5560_UA_PER_A = 0b101
} Drv8214CsGainSel;

typedef enum Drv8214InverseMotorResistanceScale {
    DRV8214_INV_R_SCALE_2 = 0b00,
    DRV8214_INV_R_SCALE_64 = 0b01,
    DRV8214_INV_R_SCALE_1024 = 0b10,
    DRV8214_INV_R_SCALE_8192 = 0b11
} Drv8214InverseMotorResistanceScale;

typedef enum Drv8214MotorConstantScale {
    DRV8214_KMC_SCALE_24_TIME_2_POW_8 = 0b00,
    DRV8214_KMC_SCALE_24_TIME_2_POW_9 = 0b01,
    DRV8214_KMC_SCALE_24_TIME_2_POW_12 = 0b10,
    DRV8214_KMC_SCALE_24_TIME_2_POW_13 = 0b11
} Drv8214MotorConstantScale;

typedef enum Drv8214RippleCounterThresholdScale {
    DRV8214_RC_THR_SCALE_2 = 0b00,
    DRV8214_RC_THR_SCALE_8 = 0b01,
    DRV8214_RC_THR_SCALE_16 = 0b10,
    DRV8214_RC_THR_SCALE_64 = 0b11
} Drv8214RippleCounterThresholdScale;

typedef enum Drv8214ErrorCorrectorWindow {
    DRV8214_EC_WINDOW_20_PERCENT = 0b00,
    DRV8214_EC_WINDOW_30_PERCENT = 0b01,
    DRV8214_EC_WINDOW_40_PERCENT = 0b10,
    DRV8214_EC_WINDOW_50_PERCENT = 0b11
} Drv8214ErrorCorrectorWindow;

typedef enum Drv8214KDiv {
    DRV8214_K_DIV_32 = 0b000,
    DRV8214_K_DIV_64 = 0b001,
    DRV8214_K_DIV_128 = 0b010,
    DRV8214_K_DIV_256 = 0b011,
    DRV8214_K_DIV_512 = 0b100,
    DRV8214_K_DIV_16 = 0b101,
    DRV8214_K_DIV_1 = 0b110
} Drv8214KDiv;

// Getters

/**
 * @brief Check whether soft-start/soft-stop is enabled.
 * @param driver Driver handle.
 * @return true if soft-start/stop is enabled, false otherwise.
 */
bool drv8214_is_soft_start_stop_enabled(Drv8214 *driver);

/**
 * @brief Get the active regulation scheme.
 * @param driver Driver handle.
 * @return Regulation scheme (fixed off-time, cycle-by-cycle, speed, or voltage regulation).
 */
Drv8214RegulationScheme drv8214_get_regulation_scheme(Drv8214 *driver);

/**
 * @brief Get the PWM carrier frequency.
 * @param driver Driver handle.
 * @return PWM frequency selector (25 kHz or 50 kHz).
 */
Drv8214PwmFreq drv8214_get_pwm_frequency(Drv8214 *driver);

/**
 * @brief Get the speed scaling factor (W_SCALE) used in speed targets.
 * @param driver Driver handle.
 * @return Speed scale (16, 32, 64, 128).
 */
Drv8214SpeedScale drv8214_get_speed_scaling_factor(Drv8214 *driver);

/**
 * @brief Read the target motor voltage when voltage regulation is active.
 * @param driver Driver handle.
 * @return Target voltage in volts.
 * @note Valid only when regulation scheme is voltage regulation. Range depends on VM gain selection.
 */
float drv8214_get_target_motor_voltage(Drv8214 *driver);

/**
 * @brief Read the target motor speed when speed regulation is active.
 * @param driver Driver handle.
 * @return Target speed in rad/s (interpreted with the configured W_SCALE).
 * @note Valid only when regulation scheme is speed regulation.
 */
float drv8214_get_target_motor_speed(Drv8214 *driver);

/**
 * @brief Get the output voltage low-pass filter cutoff selection.
 * @param driver Driver handle.
 * @return Cutoff frequency code.
 */
Drv8214CutoffFreq drv8214_get_voltage_lowpass_cutoff(Drv8214 *driver);

/**
 * @brief Get the manual duty cycle value (EXT_DUTY).
 * @param driver Driver handle.
 * @return Duty cycle percentage in [0, 100].
 * @note Applicable when manual duty control is enabled and internal bridge control is used.
 */
float drv8214_get_manual_duty_cycle(Drv8214 *driver);

/**
 * @brief Check whether ripple counting is enabled.
 * @param driver Driver handle.
 * @return true if ripple counting is enabled, false otherwise.
 */
bool drv8214_is_ripple_counting_enabled(Drv8214 *driver);

/**
 * @brief Check whether ripple error correction is disabled.
 * @param driver Driver handle.
 * @return true if error correction is disabled, false if enabled.
 */
bool drv8214_is_error_correction_disabled(Drv8214 *driver);

/**
 * @brief Check whether the bridge is forced Hi-Z when RC count exceeds threshold.
 * @param driver Driver handle.
 * @return true if bridge cutoff on RC threshold is enabled, false otherwise.
 */
bool drv8214_is_bridge_cutoff_on_threshold_enabled(Drv8214 *driver);

/**
 * @brief Get the input scaling applied to the ripple detection filter.
 * @param driver Driver handle.
 * @return Filter input scaling selection.
 */
Drv8214FilterScale drv8214_get_filter_input_scaling(Drv8214 *driver);

/**
 * @brief Get the current mirror gain selection.
 * @param driver Driver handle.
 * @return Current mirror gain / OCP range setting.
 */
Drv8214CsGainSel drv8214_get_current_mirror_gain(Drv8214 *driver);

/**
 * @brief Read the ripple counter threshold (10-bit effective) and its scale.
 * @param driver Driver handle.
 * @param[out] threshold Raw 10-bit threshold value reconstructed to 16-bit space.
 * @param[out] scale Threshold scaling factor (division applied). Can be NULL.
 */
void drv8214_get_ripple_counter_threshold(Drv8214 *driver, uint16_t *threshold, Drv8214RippleCounterThresholdScale *scale);

/**
 * @brief Get the inverse motor resistance setting and scale.
 * @param driver Driver handle.
 * @param[out] inverseResistance Raw 8-bit inverse resistance code (must not be 0).
 * @param[out] scale Scale applied to the inverse resistance. Can be NULL.
 */
void drv8214_get_motor_resistance(Drv8214 *driver, uint8_t *inverseResistance, Drv8214InverseMotorResistanceScale *scale);

/**
 * @brief Get the motor constant (Kmc) code and its scale.
 * @param driver Driver handle.
 * @param[out] motorConstant Raw 8-bit Kmc code.
 * @param[out] scale Scale applied to Kmc. Can be NULL.
 */
void drv8214_get_motor_constant(Drv8214 *driver, uint8_t *motorConstant, Drv8214MotorConstantScale *scale);

/**
 * @brief Get the band-pass filter inverse quality factor (Q⁻¹) code.
 * @param driver Driver handle.
 * @return 4-bit quality code (0..15).
 */
uint8_t drv8214_get_bandpass_quality(Drv8214 *driver);

/**
 * @brief Check whether error correction pulses are disabled.
 * @param driver Driver handle.
 * @return true if EC pulses are disabled, false otherwise.
 */
bool drv8214_is_error_correction_pulse_disabled(Drv8214 *driver);

/**
 * @brief Get ripple speed low-pass cutoff selection (mechanical filter).
 * @param driver Driver handle.
 * @return 3-bit cutoff selector (implementation-defined mapping).
 */
uint8_t drv8214_get_ripple_lowpass_cutoff(Drv8214 *driver);

/**
 * @brief Get the error-correction false-ripple window.
 * @param driver Driver handle.
 * @return Window percentage selector.
 */
Drv8214ErrorCorrectorWindow drv8214_get_error_correction_false_window(Drv8214 *driver);

/**
 * @brief Get the error-correction miss-ripple window.
 * @param driver Driver handle.
 * @return Window percentage selector.
 */
Drv8214ErrorCorrectorWindow drv8214_get_error_correction_miss_window(Drv8214 *driver);

/**
 * @brief Read PI regulator coefficients for speed/voltage regulation.
 * @param driver Driver handle.
 * @param[out] kpNum Kp numerator (5-bit).
 * @param[out] kpDen Kp denominator (K_DIV).
 * @param[out] kiNum Ki numerator (5-bit).
 * @param[out] kiDen Ki denominator (K_DIV).
 */
void drv8214_get_pi_coefficients(Drv8214 *driver, uint8_t *kpNum, uint8_t *kpDen, uint8_t *kiNum, uint8_t *kiDen);

// Setters

/**
 * @brief Enable or disable soft-start/soft-stop.
 * @param driver Driver handle.
 * @param state true to enable, false to disable.
 */
void drv8214_set_soft_start_stop_enabled(Drv8214 *driver, bool state);

/**
 * @brief Set the regulation scheme.
 * @param driver Driver handle.
 * @param currentRegulationScheme Desired scheme (fixed off-time, cycle-by-cycle, speed, voltage).
 */
void drv8214_set_regulation_scheme(Drv8214 *driver, Drv8214RegulationScheme currentRegulationScheme);

/**
 * @brief Set the PWM carrier frequency.
 * @param driver Driver handle.
 * @param pwmFreq Frequency selection (25 kHz or 50 kHz).
 */
void drv8214_set_pwm_frequency(Drv8214 *driver, Drv8214PwmFreq pwmFreq);

/**
 * @brief Set the speed scaling factor (W_SCALE).
 * @param driver Driver handle.
 * @param speedScale W_SCALE selection (16/32/64/128).
 */
void drv8214_set_speed_scaling_factor(Drv8214 *driver, Drv8214SpeedScale speedScale);

/**
 * @brief Set the target motor voltage for voltage regulation.
 * @param driver Driver handle.
 * @param voltage Target voltage in volts.
 * @note Range is limited by the configured VM gain selection.
 */
void drv8214_set_target_motor_voltage(Drv8214 *driver, float voltage);

/**
 * @brief Set the target motor speed for speed regulation.
 * @param driver Driver handle.
 * @param speed Target ripple speed in rad/s (interpreted with W_SCALE).
 */
void drv8214_set_target_ripple_speed(Drv8214 *driver, float speed);

/**
 * @brief Set the output voltage low-pass filter cutoff.
 * @param driver Driver handle.
 * @param cutoffFreq Cutoff selection.
 */
void drv8214_set_voltage_lowpass_cutoff(Drv8214 *driver, Drv8214CutoffFreq cutoffFreq);

/**
 * @brief Set manual duty cycle (EXT_DUTY).
 * @param driver Driver handle.
 * @param percentage Duty cycle in percent [0, 100].
 * @note Requires manual duty control to be enabled and internal bridge control.
 */
void drv8214_set_manual_duty_cycle(Drv8214 *driver, float percentage);

/**
 * @brief Enable or disable ripple counting.
 * @param driver Driver handle.
 * @param state true to enable, false to disable.
 */
void drv8214_set_ripple_counting_enabled(Drv8214 *driver, bool state);

/**
 * @brief Enable or disable ripple error correction.
 * @param driver Driver handle.
 * @param state true to disable error correction, false to enable it.
 */
void drv8214_set_error_correction_disabled(Drv8214 *driver, bool state);

/**
 * @brief Enable or disable forcing the bridge to Hi-Z when RC count exceeds threshold.
 * @param driver Driver handle.
 * @param state true to enable Hi-Z on threshold, false to keep bridge active.
 */
void drv8214_set_bridge_cutoff_on_threshold_enabled(Drv8214 *driver, bool state);

/**
 * @brief Set the input scaling factor for ripple detection filtering.
 * @param driver Driver handle.
 * @param filterScale Filter input scale.
 */
void drv8214_set_current_ripples_scaling_factor(Drv8214 *driver, Drv8214FilterScale filterScale);

/**
 * @brief Set current mirror gain / OCP range.
 * @param driver Driver handle.
 * @param csGainSel Gain range selection.
 * @warning Adjust your current calculations and protections accordingly.
 */
void drv8214_set_current_mirror_gain(Drv8214 *driver, Drv8214CsGainSel csGainSel);

/**
 * @brief Set the ripple counter threshold as close as possible to the target threshold.
 * The threshold value is given with 16 bits, but stored only with 10 bits of precision.
 * Therefore target threshold is rounded up or down to the closest possible value before being set.
 * @param driver Driver handle.
 * @param target Target threshold.
 * @param scale Optionnal return value, can be set to NULL. Division which have been applied to target threshold.
 * @return Actual threshold
 */
uint16_t drv8214_set_ripple_counter_threshold(Drv8214 *driver, uint16_t target, Drv8214RippleCounterThresholdScale *scale);

/**
 * @brief Set the inverse motor resistance (scaled).
 * @param driver Driver handle.
 * @param inverseResistance Raw 8-bit code (must not be 0).
 * @param scale Scale applied to the inverse resistance.
 */
void drv8214_set_motor_resistance(Drv8214 *driver, uint8_t inverseResistance, Drv8214InverseMotorResistanceScale scale);

/**
 * @brief Set the motor constant (Kmc) and scale.
 * @param driver Driver handle.
 * @param motorConstant Raw 8-bit code.
 * @param scale Kmc scale selection.
 */
void drv8214_set_motor_constant(Drv8214 *driver, uint8_t motorConstant, Drv8214MotorConstantScale scale);

/**
 * @brief Set band-pass inverse quality factor (Q⁻¹).
 * @param driver Driver handle.
 * @param quality 4-bit code [0..15].
 */
void drv8214_set_ripple_counter_bandpass_quality(Drv8214 *driver, uint8_t quality);

/**
 * @brief Enable or disable error correction pulses.
 * @param driver Driver handle.
 * @param state true to disable pulses, false to enable.
 */
void drv8214_set_error_correction_pulse_disabled(Drv8214 *driver, bool state);

/**
 * @brief Set ripple counter output filter low-pass cutoff frequency (T_MECH).
 * @param driver Driver handle.
 * @param value 3-bit cutoff selector [0..7].
 */
void drv8214_set_ripple_counter_out_lowpass_cutoff(Drv8214 *driver, uint8_t value);

/**
 * @brief Set error-correction false-ripple window.
 * @param driver Driver handle.
 * @param window Window percentage selector.
 */
void drv8214_set_error_correction_false_window(Drv8214 *driver, Drv8214ErrorCorrectorWindow window);

/**
 * @brief Set error-correction miss-ripple window.
 * @param driver Driver handle.
 * @param window Window percentage selector.
 */
void drv8214_set_error_correction_miss_window(Drv8214 *driver, Drv8214ErrorCorrectorWindow window);

/**
 * @brief Set PI regulator coefficients.
 * @param driver Driver handle.
 * @param kpNum Kp numerator (5-bit).
 * @param kpDen Kp denominator (K_DIV).
 * @param kiNum Ki numerator (5-bit).
 * @param kiDen Ki denominator (K_DIV).
 */
void drv8214_set_pi_coefficients(Drv8214 *driver, uint8_t kpNum, Drv8214KDiv kpDen, uint8_t kiNum, Drv8214KDiv kiDen);

#ifdef __cplusplus
}
#endif

#endif // DRV8214_CONTROL_H
