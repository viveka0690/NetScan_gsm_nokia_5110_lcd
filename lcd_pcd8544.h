#ifndef _LCD_PCD8544_H
#define _LCD_PCD8544_H

/********************************************************************************
 * lcd_pcd8544.h
 *
 * Description:
 *   This is a library to control an 84x48 pixel LCD display based on controller
 *   PCD8455 such as the Nokia 5110 displays. A simple 6x8 font is included which
 *   allows a 14x6 characters display.
 *
 ******************************************************************************** */

/*-------------------------------------------------------------------
 * Includes
 * ----------------------------------------------------------------*/

#include "lcd_pcd8554_cbk.h"
#include "typedefs.h"

/*-------------------------------------------------------------------
 * Defines
 * ----------------------------------------------------------------*/

#define NOKIA_5110_WIDTH 84
#define NOKIA_5110_HEIGHT 48

#define NOKIA_5110_MODE_CMD 0
#define NOKIA_5110_MODE_DATA 1

#define NOKIA_5110_PRINT_NORMAL 0
#define NOKIA_5110_PRINT_INVERSE 1

/*-------------------------------------------------------------------
 * Function Prototypes
 * ----------------------------------------------------------------*/

void Nokia_5110_Init(void);
void Nokia_5110_Set_Cursor(uint8_t x, uint8_t y);
void Nokia_5110_Putchar(char c, uint8_t print_mode);
void Nokia_5110_Putstring(const char *string);
void Nokia_5110_Putstring_Inverse(const char *string);
void Nokia_5110_Clear_All(void);

/* ***************************************************************************
 * Revision logs
 *
 * - 31-Aug-2013 Santiago Villafuerte v2
 *   + Callback file for multi platform usage
 *
 * - 12-Jan-2012 Mike Pose v1
 *   + Original file
 *
* ************************************************************************* */

#endif
