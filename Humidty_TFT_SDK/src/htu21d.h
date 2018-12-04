/*
 * htu21d.h
 *
 *  Created on: Jul 30, 2014
 *      Author: JMBrinkhus
 */

#include <stdio.h>
#include "xiic_l.h"


#ifndef HTU21D_H_
#define HTU21D_H_

// HTU21D Constants
#define HTU21D_I2C_ADDR				(0x40)	// 1000_000x
#define HTU21D_14B_CONV_DELAY_MS	(50)	// Datasheet claims max conversion time of 50ms for 14 bit resolution
#define HTU21D_13B_CONV_DELAY_MS	(25)	// Datasheet claims max conversion time of 25ms for 13 bit resolution
#define HTU21D_12B_CONV_DELAY_MS	(13)	// Datasheet claims max conversion time of 13ms for 12 bit resolution
#define HTU21D_11B_CONV_DELAY_MS	(7)		// Datasheet claims max conversion time of  7ms for 11 bit resolution
#define CRC_POLY					(0x131)	// CRC Polynomial: X^8 + X^5 + X^4 + 1

// HTU21D BIT MASKS
#define HTU21D_RESOLUTION_BIT7_MASK				(0x80)
#define HTU21D_RESOLUTION_BIT0_MASK				(0x01)
#define HTU21D_BATTERY_STATUS_MASK				(0x40)
#define HTU21D_HEATER_STATUS_MASK				(0x04)

// HTU21D I2C Commands
#define HTU21D_I2C_CMD_RESET					(0xFE)
#define HTU21D_I2C_CMD_MEAS_TEMP_WITHOUT_HOLD	(0xF3)
#define HTU21D_I2C_CMD_MEAS_HUM_WITHOUT_HOLD	(0xF5)
#define HTU21D_I2C_CMD_WRITE_USER_REG			(0xE6)
#define HTU21D_I2C_CMD_READ_USER_REG			(0xE7)

// Typedef'ed Enumerations
typedef enum htu21d_status{
	htu21d_status_ok = 0,
	htu21d_status_i2c_transfer_error = 1,
	htu21d_status_crc_error = 2
}htu21d_status;
typedef enum htu21d_resolution{
	htu21d_resolution_t_14b_rh_12b = 0,
	htu21d_resolution_t_12b_rh_8b  = 1,
	htu21d_resolution_t_13b_rh_10b = 2,
	htu21d_resolution_t_11b_rh_11b = 3
}htu21d_resolution;
typedef enum htu21d_battery_status{
	htu21d_battery_ok = 0,
	htu21d_battery_low = 1
}htu21d_battery_status;
typedef enum htu21d_heater_status{
	htu21d_heater_off = 0,
	htu21d_heater_on = 1
}htu21d_heater_status;

#endif /* HTU21D_H_ */

// External variables
extern 	u32 				htu21d_axi_address;
extern	htu21d_resolution	htu21d_res;

// Function Declarations
void			htu21d_init(u32 axi_address);
htu21d_status	htu21d_reset(void);
htu21d_status	htu21d_set_resolution(htu21d_resolution);
htu21d_status	htu21d_read_temperature_and_relative_humidity(float* t, float* rh);
htu21d_status	htu21d_get_battery_status(htu21d_battery_status*);
htu21d_status	htu21d_get_heater_status(htu21d_heater_status*);
htu21d_status	htu21d_enable_heater(void);
htu21d_status	htu21d_disable_heater(void);
float			htu21d_compute_dew_point(float Tamb, float RHamb);
int CRC8(char*);
int CRC16(char*);
