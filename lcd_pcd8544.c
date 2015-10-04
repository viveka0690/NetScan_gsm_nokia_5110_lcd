/********************************************************************************
 * lcd_pcd8544.c
 *
 * Description:
 *   This is a library to control an 84x48 pixel LCD display based on controller
 *   PCD8455 such as the Nokia 5110 displays. A simple 6x8 font is included which
 *   allows a 14x6 characters display.
 *
 ************************************************************************* */

/*-------------------------------------------------------------------
 * Includes
 * ----------------------------------------------------------------*/

#include "lcd_pcd8544.h"
#include "font_6x8.h"
//#include<stdio.h>
/*-------------------------------------------------------------------
 * Local function prototypes
 * ----------------------------------------------------------------*/

static void Nokia_5110_Write_Command(uint8_t data);
static void Nokia_5110_Write_Data(uint8_t data);

/*-------------------------------------------------------------------
 * Function definitions
 * ----------------------------------------------------------------*/
 
void Nokia_5110_Init(void)
{
	Nokia_5110_Initialize_SPI();
 
	// write commands to initialize
	Nokia_5110_Write_Command(0x21);
	Nokia_5110_Write_Command(0xc6);
	Nokia_5110_Write_Command(0x06);
	Nokia_5110_Write_Command(0x13);
	Nokia_5110_Write_Command(0x20);
	Nokia_5110_Clear_All();
	Nokia_5110_Write_Command(0x0c);
}
 
void Nokia_5110_Set_Cursor(uint8_t x, uint8_t y)
{
	Nokia_5110_Write_Command(0x40 | y); //Y increments in rows of 8 pixels
	Nokia_5110_Write_Command(0x80 | x * 6); // X increments in pixels
}
 
void Nokia_5110_Putchar(char c, uint8_t print_mode)
{
	uint8_t line;
	unsigned char ch = 0;

	for (line = 0; line < 6; line++)
	{
		ch = font6x8[c - 32][line];
		Nokia_5110_Write_Data((print_mode == NOKIA_5110_PRINT_NORMAL) ? ch : (ch ^ 0xff));
	}
}
 
void Nokia_5110_Putstring(const char *string)
{
	while (*string != '\0')
	{
		Nokia_5110_Putchar(*string, NOKIA_5110_PRINT_NORMAL);
		string++;
	}
}

void Nokia_5110_Putstring_Inverse(const char *string)
{
	while (*string != '\0')
	{
		Nokia_5110_Putchar(*string, NOKIA_5110_PRINT_INVERSE);
		string++;
	}
}

static void Nokia_5110_Write_Command(uint8_t data)
{
	Nokia_5110_Write_Byte(data, NOKIA_5110_MODE_CMD);
}

static void Nokia_5110_Write_Data(uint8_t data)
{
	Nokia_5110_Write_Byte(data, NOKIA_5110_MODE_DATA);
}

void Nokia_5110_Clear_All(void)
{
	uint8_t i, j;

	for (i = 0; i < 6; i++)
	{
		for (j = 0; j < 84; j++)
		{
			Nokia_5110_Write_Data(0);
		}
	}
}

/* ***************************************************************************
* ************************************************************************* */
