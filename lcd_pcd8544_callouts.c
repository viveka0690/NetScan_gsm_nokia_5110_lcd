/********************************************************************************
 * lcd_pcd8544_cbk.h
 *
 * Description:
 *   This file defines all function prototypes from lcd_pcd8854_cbk.h
 *
 *   This file can be modified to implement the SPI communication on any
 *   other microcontroller or processor.
 *
 * Circuit Description:
 *
 * NOKIA 5110 LCD
 * ========================================
 * Pin 1 (VCC) 			--> 	3.3V	(DO NOT USE 5V !!!)
 * Pin 2 (GND) 			--> 	GND
 * Pin 3 (SCE) 			--> 	CS Pin - chip select, enable data transfer
 * Pin 4 (RST) 			--> 	RESET pin
 * Pin 5 (D/C) 			--> 	MODE pin - Command/Data
 * Pin 6 (MOSI) 		--> 	DATA pin
 * Pin 6 (SCLK) 		--> 	CLOCK pin
 * Pin 8 (LED) 			--> 	Resistor 180 Ohms  --> 3.3V  (DO NOT CONNECT DIRECTLY TO 3.3V !!!)
 *
 ******************************************************************************** */

/*-------------------------------------------------------------------
 * Includes
 * ----------------------------------------------------------------*/

#include "bcm2835.h"
#include "lcd_pcd8554_cbk.h"

/*-------------------------------------------------------------------
 * Defines
 * ----------------------------------------------------------------*/

/// This means pin HIGH, true, 3.3volts on a pin.
#define NOKIA_5110_HIGH 0x1
/// This means pin LOW, false, 0volts on a pin.
#define NOKIA_5110_LOW  0x0

#define NOKIA_5110_CS  		 RPI_GPIO_P1_24
#define NOKIA_5110_CLK         RPI_GPIO_P1_23
#define NOKIA_5110_DATA        RPI_GPIO_P1_19
#define NOKIA_5110_MODE        RPI_GPIO_P1_11
#define NOKIA_5110_RST         RPI_GPIO_P1_15 

/*#define NOKIA_5110_CS 		RPI_GPIO_P1_11
#define NOKIA_5110_CLK 		RPI_GPIO_P1_15
#define NOKIA_5110_DATA		RPI_GPIO_P1_16
#define NOKIA_5110_MODE		RPI_GPIO_P1_18
#define NOKIA_5110_RST		RPI_GPIO_P1_22*/ 

/*-------------------------------------------------------------------
 * Function Definitions
 * ----------------------------------------------------------------*/

void Nokia_5110_Initialize_SPI(void)
{
    if (!bcm2835_init())
    {
        return;
    }

	// set the direction of the pins
	bcm2835_gpio_fsel(NOKIA_5110_CS  , BCM2835_GPIO_FSEL_OUTP); // CE
	bcm2835_gpio_fsel(NOKIA_5110_CLK , BCM2835_GPIO_FSEL_OUTP); // SCLK
	bcm2835_gpio_fsel(NOKIA_5110_DATA, BCM2835_GPIO_FSEL_OUTP); // MOSI
	bcm2835_gpio_fsel(NOKIA_5110_MODE, BCM2835_GPIO_FSEL_OUTP); // MODE
	bcm2835_gpio_fsel(NOKIA_5110_RST , BCM2835_GPIO_FSEL_OUTP); // RST

	// set the default state for the pins
	bcm2835_gpio_write(NOKIA_5110_CS,   HIGH);
	bcm2835_gpio_write(NOKIA_5110_CLK,  HIGH);
	bcm2835_gpio_write(NOKIA_5110_DATA, HIGH);
	bcm2835_gpio_write(NOKIA_5110_MODE, HIGH);

	// reset the display
	bcm2835_gpio_write(NOKIA_5110_RST, LOW);
	bcm2835_delay(5);
	bcm2835_gpio_write(NOKIA_5110_RST, HIGH);
}

void Nokia_5110_Write_Byte(uint8_t data, uint8_t lcd_mode)
{
	uint8_t i;

	// enable the lcd
	bcm2835_gpio_write(NOKIA_5110_CS,   LOW);

	if (NOKIA_5110_MODE_CMD == lcd_mode)
		bcm2835_gpio_write(NOKIA_5110_MODE,   LOW);
	else
		bcm2835_gpio_write(NOKIA_5110_MODE,   HIGH);

	for (i = 0; i < 8; i++)
	{
		bcm2835_gpio_write(NOKIA_5110_DATA, data & 0x80);
		data = data << 1;
		// toggle the clock pin
		bcm2835_gpio_write(NOKIA_5110_CLK,  LOW);
		bcm2835_gpio_write(NOKIA_5110_CLK,  HIGH);
	}

	// disable the lcd
	bcm2835_gpio_write(NOKIA_5110_CS,   HIGH);
}

void Nokia_5110_Stop_SPI(void)
{
	bcm2835_close();
}

/* ***************************************************************************
 * Revision logs
 *
 * - 31-Aug-2013 Santiago Villafuerte v1
 *   + Callback file for multi platform usage
 *
* ************************************************************************* */
