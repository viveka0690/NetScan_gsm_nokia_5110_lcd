
#ifndef LCD_PCD8544_CBK_H
#define LCD_PCD8544_CBK_H

/********************************************************************************
 * lcd_pcd8544_cbk.h
 *
 * Description:
 *   This file declares function prototypes to write data to the outside.
 *
 *   Implement SPI by using bit bang or HW SPI directly on your hardware.
 *
 *   Create an "lcd_pcd8544_callout.c" file to define the functions.
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

#include "lcd_pcd8544.h"
#include "typedefs.h"

/*-------------------------------------------------------------------
 * Function Prototypes
 * ----------------------------------------------------------------*/

void Nokia_5110_Initialize_SPI(void);
void Nokia_5110_Write_Byte(uint8_t data, uint8_t lcd_mode);
void Nokia_5110_Stop_SPI(void);

/* ***************************************************************************
 * Revision logs
 *
 * - 31-Aug-2013 Santiago Villafuerte v1
 *   + Callback file for multi platform usage
 *
* ************************************************************************* */

#endif /* LCD_PCD8544_CBK_H */
