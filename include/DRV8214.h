/**
 * @file drv8214.h
 * @author Killian Baillifard, Theo Heng.
 * @date 10.10.2025
 * @brief DRV8214 library.
 */

#ifndef DRV8214_STATUS_H
#define DRV8214_STATUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "drv8214_status.h"
#include "drv8214_config.h"
#include "drv8214_control.h"

#define DRV8214_I2C_ADDR_00  0x30  // Address = 0x60 / 0x61 in 8-bit | A1 = 0, A0 = 0 
#define DRV8214_I2C_ADDR_0Z  0x31  // Address = 0x62 / 0x63 in 8-bit | A1 = 0, A0 = Z
#define DRV8214_I2C_ADDR_01  0x32  // Address = 0x64 / 0x65 in 8-bit | A1 = 0, A0 = 1
#define DRV8214_I2C_ADDR_Z0  0x33  // Address = 0x66 / 0x67 in 8-bit | A1 = Z, A0 = 0
#define DRV8214_I2C_ADDR_ZZ  0x34  // Address = 0x68 / 0x69 in 8-bit | A1 = Z, A0 = Z
#define DRV8214_I2C_ADDR_Z1  0x35  // Address = 0x6A / 0x6B in 8-bit | A1 = Z, A0 = 1
#define DRV8214_I2C_ADDR_10  0x36  // Address = 0x6C / 0x6D in 8-bit | A1 = 1, A0 = 0
#define DRV8214_I2C_ADDR_1Z  0x37  // Address = 0x6E / 0x6F in 8-bit | A1 = 1, A0 = Z
#define DRV8214_I2C_ADDR_11  0x38  // Address = 0x70 / 0x71 in 8-bit | A1 = 1, A0 = 1

#ifdef __cplusplus
}
#endif

#endif // DRV8214_STATUS_H