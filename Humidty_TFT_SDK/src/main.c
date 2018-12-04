//Copyright dhq (August 9 2018)
//License GPLV3
#include <stdio.h>
#include <sleep.h>
#include <time.h>
#include <unistd.h>

#include "platform.h"
#include "xil_printf.h"
#include "xparameters.h"
#include "xgpio.h"
#include "xstatus.h"
#include "Delay.h"
#include "SPI.h"
#include "LCD_Driver.h"
#include "LCD_GUI.h"
#include "htu21d.h"


extern XGpio gpio0;
extern XSpi  SpiInstance;	 /* The instance of the SPI device */

extern const unsigned char font[] ;

#define BACKGROUND  WHITE
#define FOREGROUND BLUE
#define DELAY 1000

#include "htu21d.h"

void htu21d_main_menu(void);

int main()
{
	int Status;
    char key_input;
    int i;
    htu21d_status stat;
    float temperature;
    float relative_humidity;
    float dew_point;
    //htu21d_battery_status	batt_stat;
    //htu21d_heater_status	heat_stat;

    //Initialize the UART
    init_platform();
    xil_printf("MiniZed humidity monitoring. \r\n");

	/* Initialize the GPIO 0 driver */
	Status = XGpio_Initialize(&gpio0, XPAR_AXI_GPIO_0_DEVICE_ID);
	if (Status != XST_SUCCESS) {
		xil_printf("Gpio 0 Initialization Failed\r\n");
		return XST_FAILURE;
	}

	// Set up the AXI SPI Controller
	Status = XSpi_Init(&SpiInstance,SPI_DEVICE_ID);
	if (Status != XST_SUCCESS) {
		xil_printf("SPI Mode Failed\r\n");
		return XST_FAILURE;
	}

    printf("Hello World\n");

    // Set the AXI address of the IIC core
    htu21d_init(XPAR_AXI_IIC_0_BASEADDR);

    // Display the main menu
    htu21d_main_menu();


	xil_printf("TFT initialized \r\n");

	xil_printf("**********Init LCD**********\r\n");
	LCD_SCAN_DIR LCD_ScanDir = SCAN_DIR_DFT;//SCAN_DIR_DFT = D2U_L2R
	LCD_Init(LCD_ScanDir );

	xil_printf("LCD Show \r\n");
	GUI_Show();
	delay_ms(1000);
	// Set resolution to 12-bit RH and 14-bit Temp
	stat = htu21d_set_resolution(htu21d_resolution_t_14b_rh_12b);
	printf("\nSetting HTU21D Resolution to 14-bit Temperature and 12-bit Relative Humidity\n");

	char tempbuf[16] = {};
	char humibuf[16] = {};


    while(1){

		// Read Temperature and Relative Humidity once
		 printf("\n");
		 printf("Reading Temperature and Relative Humidity...\n");
		 stat = htu21d_read_temperature_and_relative_humidity(&temperature, &relative_humidity);

		 // Display the status returned from the read_temperature and relative humidity
		 // operation and display the temperature and relative humidity if successful
		 printf("Temperature and Relative Humidity Read Complete with status: ");
		 if(stat==htu21d_status_ok){
			 printf("Ok.\n");
			 printf("Temperature : %5.2f%cC, \tRelative Humidity : %4.1f%%",temperature,248,relative_humidity);
		 }else if(stat==htu21d_status_i2c_transfer_error){
			 printf("Transfer Error.");
		 }else if(stat==htu21d_status_crc_error){
			printf("CRC Error.");
		 }

		LCD_Clear(GUI_BACKGROUND);
		GUI_DrawRectangle(0,0,159,127,BLUE,DRAW_EMPTY,DOT_PIXEL_2X2);

		GUI_DisString_EN(40,10,"Temp",&Font20,GUI_BACKGROUND,CYAN);
		GUI_DisString_EN(15,70,"Humidity",&Font20,GUI_BACKGROUND,CYAN);

		sprintf(tempbuf, "%2.2f C ", temperature);
		sprintf(humibuf, "%2.2f %%", relative_humidity);

		GUI_DisString_EN(10,40,tempbuf,&Font24,GUI_BACKGROUND,YELLOW);
		GUI_DisString_EN(10,100,humibuf,&Font24,GUI_BACKGROUND,YELLOW);


		delay_ms(5000);



    }

    return 0;

}

void htu21d_main_menu(void){

    //Clear the screen
    printf("\033[2J");

    //Display the main menu
    printf("*******************************************\n");
    printf("****      Measurement Specialties      ****\n");
    printf("*******************************************\n");

    printf("\n");
    printf("   HTU21D - Humidity/Temperature Sensor   \n");
    printf("------------------------------------------\n");

    printf("\n");
    printf("Select a task:\n");
    printf("  (1)   - Reset\n");
    printf("  (2)   - Set Resolution\n");
    printf("  (3)   - Read Temperature and Relative Humidity Once\n");
    printf("  (4)   - Read Temperature and Relative Humidity 20 Times\n");
    printf("  (5)   - Compute Dew Point\n");
    printf("  (6)   - Get Battery Status\n");
    printf("  (7)   - Get Heater Status\n");
    printf("  (8)   - Enable Heater\n");
    printf("  (9)   - Disable Heater\n");
    printf(" (ESC)  - Quit\n");
    printf("\n");

    return;
}

