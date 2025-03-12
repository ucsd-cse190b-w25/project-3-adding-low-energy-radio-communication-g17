/*
 * lssm6dsl.c
 *
 *  Created on: Feb 2, 2025
 *      Author: Daniel Lov
 */

/* Include memory map of our MCU */
#include <stm32l475xx.h>
#include <i2c.h>

#define LSM6DSL_SADD    0x6A
#define CTRL1_XL   0x10
#define INT1_CTRL  0x0D
#define MD1_CFG 0x5E
#define TAP_CFG 0x58
#define WAKE_UP_THS 0x5B
#define WAKE_UP_DUR 0X5C
#define LSB_GRAV 16384 // LSB per G, found in datasheet

void lsm6dsl_init() {
    uint8_t ctrl1_xl_data[2];
    ctrl1_xl_data[0] = CTRL1_XL;  // Register address for CTRL1_XL
    ctrl1_xl_data[1] = 0x60;      // Value to set (416 Hz, High-Performance mode)

    uint8_t int1_ctrl_data[2];
    int1_ctrl_data[0] = INT1_CTRL;  // Register address for INT1_CTRL
    int1_ctrl_data[1] = 0x01;       // Value to set (enable data-ready interrupt)

    uint8_t wake_up_dur_data[2];
    wake_up_dur_data[0] = WAKE_UP_DUR;
    wake_up_dur_data[1] = 0x00;

    uint8_t wake_up_ths_data[2];
    wake_up_ths_data[0] = WAKE_UP_THS;
    wake_up_ths_data[1] = 0x02;

    uint8_t tap_cfg_data[2];
    tap_cfg_data[0] = TAP_CFG;
    tap_cfg_data[1] = 0x90;

    uint8_t md1_cfg_data[2];
    md1_cfg_data[0] = MD1_CFG;
    md1_cfg_data[1] = 0x20;

    // Write to CTRL1_XL register to configure the accelerometer
    i2c_transaction(LSM6DSL_SADD, 0, ctrl1_xl_data, 2);

    // Write to INT1_CTRL register to enable the interrupt
    i2c_transaction(LSM6DSL_SADD, 0, int1_ctrl_data, 2);

    // Write to WAKE_UP_DUR register to set duration to 0
    i2c_transaction(LSM6DSL_SADD, 0, wake_up_dur_data, 2);

    // Write to WAKE_UP register to set wakeup threshold
    i2c_transaction(LSM6DSL_SADD, 0, wake_up_ths_data, 2);

    // Write to INT1_CTRL register to enable the interrupt
    i2c_transaction(LSM6DSL_SADD, 0, tap_cfg_data, 2);

    // Write to MD1_CFG register to set wake-up interrupt to int1 pin.
    i2c_transaction(LSM6DSL_SADD, 0, md1_cfg_data, 2);
}

void lsm6dsl_read_xyz(int16_t* x, int16_t* y, int16_t* z) {
    uint8_t data[6];
    uint8_t reg = 0x28;
    /*
     * OUTX_L_XL    0x28
     * OUTX_H_XL    0x29
     * OUTY_L_XL    0x2A
     * OUTY_H_XL    0x2B
     * OUTZ_L_XL    0x2C
     * OUTZ_H_XL    0x2D
     */

    if (i2c_transaction(LSM6DSL_SADD, 0, &reg, 1)) {
    	return;
    }

    if (i2c_transaction(LSM6DSL_SADD, 1, data, 6)) {
    	return;
    }

    *x = (int16_t) (data[1] << 8 | data[0]);
    *y = (int16_t) (data[3] << 8 | data[2]);
    *z = (int16_t) (data[5] << 8 | data[4]);
}
