/*
 * htu21d.c
 *
 *  Created on: Aug 12, 2014
 *      Author: JMBrinkhus
 */

#include "htu21d.h"
#include "xiic_l.h"
#include "sleep.h"
#include "math.h"
#include "xparameters.h"

// HTU21D Global Variables
u32 				htu21d_axi_address;
htu21d_resolution	htu21d_res;

/*********************************************************************
 *
 * Function:    htu21d_init
 *
 * Description:	Initialize the axi address of the i2c core and the
 * 				internal resolution variable to t_14b_rh_12b to reflect
 * 				the sensor's initial resolution value on reset
 *
 * Parameters:	u32 axi_address -
 * 					Address of Xilinx AXI IIC Peripheral
 *
 * Returns:		void
 *
 *********************************************************************/
void htu21d_init(u32 axi_address){

	htu21d_axi_address = axi_address;
	htu21d_res = htu21d_resolution_t_14b_rh_12b;

	return;
}


/*********************************************************************
 *
 * Function:    htu21d_reset
 *
 * Description:	Send I2C Reset command to HTU21D and wait 10ms
 *
 * Parameters:	none
 *
 * Returns:		Enumerated type htu21d_status with possible values
 * 				htu21d_status_ok or htu21d_status_i2c_transfer_error
 *
 *********************************************************************/
htu21d_status htu21d_reset(void){

	char tx_buf[1];
	int byte_count=0;

	// Send I2C reset command
	tx_buf[0] = HTU21D_I2C_CMD_RESET;
	byte_count = XIic_Send(htu21d_axi_address, HTU21D_I2C_ADDR, (u8*)tx_buf, 1, XIIC_STOP);
	if(byte_count!=1){
		return htu21d_status_i2c_transfer_error;
	}

	// Wait 10ms
	usleep(10000);

	return htu21d_status_ok;
}

/*********************************************************************
 *
 * Function:    htu21d_set_resolution
 *
 * Description:	Read the user register from the device, modify its
 * 				contents to reflect the resolution that is passed in
 * 				to this function, and then write the updated user
 * 				register value to the device
 *
 * Parameters:	htu21d_resolution res
 * 					Resolution to set the device to
 *
 * Returns:		Enumerated type htu21d_status with possible values
 * 				htu21d_status_ok or htu21d_status_i2c_transfer_error
 *
 *********************************************************************/
htu21d_status	htu21d_set_resolution(htu21d_resolution res){

	htu21d_res = res;
	char tx_buf[2];
	char rx_buf[1];
	int byte_count=0;

	// Read user register
	tx_buf[0] = HTU21D_I2C_CMD_READ_USER_REG;
	byte_count = XIic_Send(htu21d_axi_address, HTU21D_I2C_ADDR, (u8*)tx_buf, 1, XIIC_STOP);
	if(byte_count!=1){
		return htu21d_status_i2c_transfer_error;
	}
	byte_count = XIic_Recv(htu21d_axi_address, HTU21D_I2C_ADDR, (u8*)rx_buf, 1, XIIC_STOP);
	if(byte_count!=1){
		return htu21d_status_i2c_transfer_error;
	}

	// Modify user register to reflect resolution change
	tx_buf[1] = rx_buf[0] & ~(HTU21D_RESOLUTION_BIT7_MASK | HTU21D_RESOLUTION_BIT0_MASK);	// Zero out bits 7 and 0
	if(res==htu21d_resolution_t_13b_rh_10b || res==htu21d_resolution_t_11b_rh_11b)
		tx_buf[1] |= HTU21D_RESOLUTION_BIT7_MASK;	// Set bit 7
	if(res==htu21d_resolution_t_12b_rh_8b || res==htu21d_resolution_t_11b_rh_11b)
		tx_buf[1] |= HTU21D_RESOLUTION_BIT0_MASK;	// Set bit 0

	// Write user register
	tx_buf[0] = HTU21D_I2C_CMD_WRITE_USER_REG;
	byte_count = XIic_Send(htu21d_axi_address, HTU21D_I2C_ADDR, (u8*)tx_buf, 2, XIIC_STOP);
	if(byte_count!=2){
		return htu21d_status_i2c_transfer_error;
	}

	return htu21d_status_ok;
}

/*********************************************************************
 *
 * Function:    htu21d_read_temperature_and_relative_humidity
 *
 * Description:	Send I2C commands to start a temperature conversion,
 * 				wait for completion, read the temperature value, start
 * 				a relative humidity conversion, wait for completion,
 * 				and read the relative humidity value
 *
 * Parameters:	float* temperature -
 * 					pointer to temperature variable
 * 				float* relative_humidity -
 * 					pointer to relative humidity variable
 *
 * Returns:		Enumerated type htu21d_status with possible values
 * 				htu21d_status_ok or htu21d_status_i2c_transfer_error
 * 				or htu21d_status_crc_error
 *
 *********************************************************************/
htu21d_status	htu21d_read_temperature_and_relative_humidity(float* temperature, float* relative_humidity){

	char tx_buf[1];
	char rx_buf[3];
	u16 adc16;
	int byte_count=0;
	float humidity=0;

	// Start temperature ADC conversion
	tx_buf[0] = HTU21D_I2C_CMD_MEAS_TEMP_WITHOUT_HOLD;
	byte_count = XIic_Send(htu21d_axi_address, HTU21D_I2C_ADDR, (u8*)tx_buf, 1, XIIC_STOP);
	if(byte_count!=1){
		return htu21d_status_i2c_transfer_error;
	}

	// Wait only as long as is needed for the resolution that is currently set
	if(htu21d_res == htu21d_resolution_t_11b_rh_11b){
		usleep(HTU21D_11B_CONV_DELAY_MS * 1000);
	}else if(htu21d_res == htu21d_resolution_t_12b_rh_8b){
		usleep(HTU21D_12B_CONV_DELAY_MS * 1000);
	}else if(htu21d_res == htu21d_resolution_t_13b_rh_10b){
		usleep(HTU21D_13B_CONV_DELAY_MS * 1000);
	}else{
		usleep(HTU21D_14B_CONV_DELAY_MS * 1000);
	}

	// Read temperature ADC result
	byte_count = XIic_Recv(htu21d_axi_address, HTU21D_I2C_ADDR, (u8*)rx_buf, 3, XIIC_STOP);
	if(byte_count!=3){
		return htu21d_status_i2c_transfer_error;
	}

	// CRC Temperature Data
	if(CRC16(rx_buf) == TRUE){
		// Concatenate the received bytes into the 16 bit result
		adc16 = 256*rx_buf[0] + (rx_buf[1]&0xFC);	//Maximum of 14 bits of resolution
		// Use formula to convert ADC result to degrees Celcius
		*temperature = (float)adc16 * pow(2, -16) * 175.72 - 46.85;
	}else{
		*temperature = 0;
		return htu21d_status_crc_error;
	}

	// Start relative humidity ADC conversion
	tx_buf[0] = HTU21D_I2C_CMD_MEAS_HUM_WITHOUT_HOLD;
	byte_count = XIic_Send(htu21d_axi_address, HTU21D_I2C_ADDR, (u8*)tx_buf, 1, XIIC_STOP);
	if(byte_count!=1){
		return htu21d_status_i2c_transfer_error;
	}

	// Wait only as long as is needed for the resolution that is currently set
	if(htu21d_res == htu21d_resolution_t_11b_rh_11b){
		usleep(HTU21D_11B_CONV_DELAY_MS * 1000);
	}else if(htu21d_res == htu21d_resolution_t_12b_rh_8b){
		usleep(HTU21D_12B_CONV_DELAY_MS * 1000);
	}else if(htu21d_res == htu21d_resolution_t_13b_rh_10b){
		usleep(HTU21D_13B_CONV_DELAY_MS * 1000);
	}else{
		usleep(HTU21D_14B_CONV_DELAY_MS * 1000);
	}

	// Read relative humidity ADC result
	byte_count = XIic_Recv(htu21d_axi_address, HTU21D_I2C_ADDR, (u8*)rx_buf, 3, XIIC_STOP);
	if(byte_count!=3){
		return htu21d_status_i2c_transfer_error;
	}

	// CRC relative humidity data
	if(CRC16(rx_buf) == TRUE){
		//Concatenate the received bytes into the 16 bit result
		adc16 = 256*rx_buf[0] + (rx_buf[1]&0xF0);
		// Use formula to convert ADC result to relative humidity as a percentage
		humidity = -6.0 + 125.0 * (float)adc16 / 65536.0;
		// Bound humidity from 0% to 100%
		if(humidity<0){
			humidity = 0;
		}else if(humidity>100){
			humidity = 100;
		}
	}else{
		return htu21d_status_crc_error;
	}

	// Assign relative humidity
	*relative_humidity = humidity;

	return htu21d_status_ok;
}

/*********************************************************************
 *
 * Function:    htu21d_get_battery_status
 *
 * Description:	Send I2C command to read battery status
 *
 * Parameters:	htu21d_battery_status* batt_stat -
 * 					Pointer to battery status variable
 *
 * Returns:		Enumerated type htu21d_status with possible values
 * 				htu21d_status_ok or htu21d_status_i2c_transfer_error
 *
 *********************************************************************/
htu21d_status htu21d_get_battery_status(htu21d_battery_status* batt_stat){

	char tx_buf[1];
	char rx_buf[1];
	int byte_count=0;

	// Read user register
	tx_buf[0] = HTU21D_I2C_CMD_READ_USER_REG;
	byte_count = XIic_Send(htu21d_axi_address, HTU21D_I2C_ADDR, (u8*)tx_buf, 1, XIIC_STOP);
	if(byte_count!=1){
		return htu21d_status_i2c_transfer_error;
	}
	byte_count = XIic_Recv(htu21d_axi_address, HTU21D_I2C_ADDR, (u8*)rx_buf, 1, XIIC_STOP);
	if(byte_count!=1){
		return htu21d_status_i2c_transfer_error;
	}

	// Interpret result
	if( rx_buf[0] & HTU21D_BATTERY_STATUS_MASK ){
		*batt_stat = htu21d_battery_low;
	}else{
		*batt_stat = htu21d_battery_ok;
	}

	return htu21d_status_ok;
}

/*********************************************************************
 *
 * Function:    htu21d_get_heater_status
 *
 * Description:	Send I2C command to read heater status
 *
 * Parameters:	htu21d_heater_status* heat_stat -
 * 					Pointer to heater status variable that will be
 * 					assigned after status is read
 *
 * Returns:		Enumerated type htu21d_status with possible values
 * 				htu21d_status_ok or htu21d_status_i2c_transfer_error
 *
 *********************************************************************/
htu21d_status htu21d_get_heater_status(htu21d_heater_status* heat_stat){

	char tx_buf[1];
	char rx_buf[1];
	int byte_count=0;

	tx_buf[0] = HTU21D_I2C_CMD_READ_USER_REG;
	byte_count = XIic_Send(htu21d_axi_address, HTU21D_I2C_ADDR, (u8*)tx_buf, 1, XIIC_STOP);
	if(byte_count!=1){
		return htu21d_status_i2c_transfer_error;
	}

	byte_count = XIic_Recv(htu21d_axi_address, HTU21D_I2C_ADDR, (u8*)rx_buf, 1, XIIC_STOP);
	if(byte_count!=1){
		return htu21d_status_i2c_transfer_error;
	}

	if(rx_buf[0] & HTU21D_HEATER_STATUS_MASK){
		*heat_stat = htu21d_heater_on;
	}else{
		*heat_stat = htu21d_heater_off;
	}

	return htu21d_status_ok;
}

/*********************************************************************
 *
 * Function:    htu21d_enable_heater
 *
 * Description:	Send I2C command to read user register, modify the
 * 				user register to enable the heater
 *
 * Parameters:	N/A
 *
 * Returns:		Enumerated type htu21d_status with possible values
 * 				htu21d_status_ok or htu21d_status_i2c_transfer_error
 *
 *********************************************************************/
htu21d_status htu21d_enable_heater(void){

	char tx_buf[2];
	char rx_buf[1];
	int byte_count=0;

	tx_buf[0] = HTU21D_I2C_CMD_READ_USER_REG;
	byte_count = XIic_Send(htu21d_axi_address, HTU21D_I2C_ADDR, (u8*)tx_buf, 1, XIIC_STOP);
	if(byte_count!=1){
		return htu21d_status_i2c_transfer_error;
	}

	byte_count = XIic_Recv(htu21d_axi_address, HTU21D_I2C_ADDR, (u8*)rx_buf, 1, XIIC_STOP);
	if(byte_count!=1){
		return htu21d_status_i2c_transfer_error;
	}

	tx_buf[0] = HTU21D_I2C_CMD_WRITE_USER_REG;
	tx_buf[1] = rx_buf[0] | HTU21D_HEATER_STATUS_MASK;
	byte_count = XIic_Send(htu21d_axi_address, HTU21D_I2C_ADDR, (u8*)tx_buf, 2, XIIC_STOP);
	if(byte_count!=2){
		return htu21d_status_i2c_transfer_error;
	}

	return htu21d_status_ok;
}

/*********************************************************************
 *
 * Function:    htu21d_disable_heater
 *
 * Description:	Send I2C command to read user register, modify the
 * 				user register to disable the heater
 *
 * Parameters:	N/A
 *
 * Returns:		Enumerated type htu21d_status with possible values
 * 				htu21d_status_ok or htu21d_status_i2c_transfer_error
 *
 *********************************************************************/
htu21d_status htu21d_disable_heater(void){

	char tx_buf[2];
	char rx_buf[1];
	int byte_count=0;

	tx_buf[0] = HTU21D_I2C_CMD_READ_USER_REG;
	byte_count = XIic_Send(htu21d_axi_address, HTU21D_I2C_ADDR, (u8*)tx_buf, 1, XIIC_STOP);
	if(byte_count!=1){
		return htu21d_status_i2c_transfer_error;
	}

	byte_count = XIic_Recv(htu21d_axi_address, HTU21D_I2C_ADDR, (u8*)rx_buf, 1, XIIC_STOP);
	if(byte_count!=1){
		return htu21d_status_i2c_transfer_error;
	}

	tx_buf[0] = HTU21D_I2C_CMD_WRITE_USER_REG;
	tx_buf[1] = rx_buf[0] & ~HTU21D_HEATER_STATUS_MASK;
	byte_count = XIic_Send(htu21d_axi_address, HTU21D_I2C_ADDR, (u8*)tx_buf, 2, XIIC_STOP);
	if(byte_count!=2){
		return htu21d_status_i2c_transfer_error;
	}

	return htu21d_status_ok;
}

/*********************************************************************
 *
 * Function:    htu21d_compute_dew_point
 *
 * Description:	Compute dew point temperature in degrees C
 *
 * Parameters:	Tamb -
 * 					float containing ambient temperature
 * 				RHamb -
 * 					float containing ambient relative humidity
 *
 * Returns:		Returns dew point temperature in degrees C
 *
 *********************************************************************/
float htu21d_compute_dew_point(float Tamb, float RHamb){

	float A = 8.1332;
	float B = 1762.39;
	float C = 235.66;
	float PP_Tamb = pow(10,A-B/(Tamb+C));
	float Td = -(B/(log10(RHamb*PP_Tamb/100)-A)+C);

	return Td;
}






/*********************************************************************
 *
 * Function:    CRC8
 *
 * Description:	Check 1 byte of data with 8 bits of CRC information.
 * 				Use polynomial X^8 + X^5 + X^4 + 1
 * 				For use with TSYS02D serial number read
 *
 * Parameters:	char* data -
 * 					2 byte character array. data[0] is the data byte
 * 					and data[1] contains the CRC information
 *
 * Returns:		TRUE if operation succeeded else FALSE
 *
 *********************************************************************/
int CRC8(char* data){
	u32 div, poly;
	int i;
	div = 256*data[0]+data[1];
	poly = CRC_POLY;
	//printf("Divisor: 0x%X\n",(unsigned int)div);
	for(i=0;i<8;i++){
		if( (1<<(15-i))&div ){
			//printf("         0x%X\n         0x%X\n",(unsigned int)(poly<<(7-i)),(unsigned int)div);
			div ^= (poly<<(7-i));
		}
	}
	//printf("Result: 0x%X",(unsigned int)div);
	if( (div&(0xFF))==0x00 ){
		return TRUE;
	}else{
		return FALSE;
	}
}

/*********************************************************************
 *
 * Function:    CRC16
 *
 * Description:	Check 2 bytes of data with 8 bits of CRC information.
 * 				Use polynomial X^8 + X^5 + X^4 + 1
 * 				For use with TSYS02D serial number read and checking
 * 				ADC results for several sensors
 *
 * Parameters:	char* data -
 * 					3 byte character array. data[0] is the most sig-
 * 					nificant byte, data[1] is the least significant
 * 					byte, and data[2] contains the CRC information
 *
 * Returns:		TRUE if operation succeeded else FALSE
 *
 *********************************************************************/
int CRC16(char* data){
	u32 div, poly;
	int i;
	div = 256*256*data[0]+256*data[1]+data[2];
	poly = CRC_POLY;
	//printf("Divisor: 0x%X\n",(unsigned int)div);
	for(i=0;i<16;i++){
		if( (1<<(23-i))&div ){
			//printf("         0x%X\n         0x%X\n",(unsigned int)(poly<<(15-i)),(unsigned int)div);
			div ^= (poly<<(15-i));
		}
	}
	//printf("Result: 0x%X",(unsigned int)div);
	if( (div&(0xFF))==0x00 ){
		return TRUE;
	}else{
		return FALSE;
	}
}
