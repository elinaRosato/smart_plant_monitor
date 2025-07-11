/***************************************************************************
* LCD Header File
*
* Created: 2022-04-16
* Updated: 2022-04-16
* Author: GBO
* updated by: nlantau
**************************************************************************/
/***************************************************************************
Connection Diagram:
----------- ----------
|ATmega328p | | LCD |
| | | |
| PB0|---------------->|D7 |
| PB1|---------------->|D6 |
| PB2|---------------->|D5 |
| PB3|---------------->|D4 |
| | |D3 |
| | |D2 |
| | |D1 |
| | |D0 |
| | | |
| PB4|---------------->|E |
| | GND --->|RW |
| PB5|---------------->|RS |
----------- ----------
**************************************************************************/
/***** Include guard ******************************************************/
#ifndef LCD_N_H_
#define LCD_N_H_
#ifndef F_CPU
#define F_CPU 16000000UL
#endif /* F_CPU */
/***** Include section ****************************************************/
#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
/***** Macro Definitions **************************************************/
#define LCD_D7_PORT PORTB // lcd D7 connection
#define LCD_D7_BIT PB0
#define LCD_D7_DDR DDRB
#define LCD_D6_PORT PORTB // lcd D6 connection
#define LCD_D6_BIT PB1
#define LCD_D6_DDR DDRB
#define LCD_D5_PORT PORTB // lcd D5 connection
#define LCD_D5_BIT PB2
#define LCD_D5_DDR DDRB
#define LCD_D4_PORT PORTB // lcd D4 connection
#define LCD_D4_BIT PB3
#define LCD_D4_DDR DDRB
#define LCD_E_PORT PORTB // lcd Enable pin
#define LCD_E_BIT PB4
#define LCD_E_DDR DDRB
#define LCD_RS_PORT PORTB // lcd Register Select pin
#define LCD_RS_BIT PB5
#define LCD_RS_DDR DDRB
// LCD module information
#define FIRST_ROW 0x00 // start of line 1
#define SECOND_ROW 0xC0 // start of line 2
// LCD instructions
#define LCD_CLEAR 0x01 // replace all characters with ASCII 'space'
#define LCD_HOME 0x02 // return cursor to first position on first line
#define LCD_ENTRY_MODE 0x06 // shift cursor from left to right on read/write
#define LCD_DISPLAY_OFF 0x08 // turn display off
#define LCD_DISPLAY_ON 0x0C // display on, cursor off, don't blink character
#define LCD_FUNCTION_RESET 0x30 // reset the LCD
#define LCD_FUNCTION_SET_4BIT 0x28 // 4-bit data, 2-line display, 5 x 7 font
#define LCD_SET_CURSOR 0x80 // set cursor position
/***** Function prototypes ************************************************/
void lcd_write(uint8_t);
void lcd_putcmd(uint8_t);
void lcd_putc(uint8_t);
void lcd_puts(uint8_t*);
void lcd_init(void);
void lcd_init_pinout(void);
void lcd_clear_row(uint8_t row);
void lcd_clear_and_home(void);
void lcd_print_rows(uint8_t* first_row, uint8_t* second_row);
#endif /* LCD_N_H_ */
