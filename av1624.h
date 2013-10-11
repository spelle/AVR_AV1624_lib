/*
 * av1624.h
 *
 *  Created on: Mar 29, 2013
 *      Author: khumbu
 */

#ifndef AV1624_H_
#define AV1624_H_



#include <avr/io.h>

#include <stdio.h>

#define NBLINE         2
#define NBCHAR_PER_LINE 16



// In 4-bit Mode, in order to reduce I/O pins

#define AV1624_DATA_PORT     PORTD  // Data Port
#define AV1624_DATA_DDR      DDRD   // Data Port Direction Register
#define AV1624_DATA_DDR_MASK 0xF0   // 4-bit mode
#define AV1624_DATA_MASK     0xF0   // 4-bit mode

#define AV1624_CTL_PORT      PORTB  // Control Port
#define AV1624_CTL_DDR       DDRB   // Control Port Direction Register
#define AV1624_CTL_DDR_MASK  0x07	// Control Port Direction Register Mask

#define AV1624_CTL_RS        PORTB0 // Control Signal RS
#define AV1624_CTL_RW        PORTB1 // Control Signal RW
#define AV1624_CTL_E         PORTB2 // Control Signal E


// Enumeration for HIGH / LOW State
typedef enum
{
	low_v = 0x00,
	high_v = 0x01
} bool_t ;



void lcd_4bit_Send( bool_t rs_v, bool_t rw_v, uint8_t v ) ;

void lcd_4bit_SendCmd( uint8_t v ) ;

void lcd_4bit_WriteData( uint8_t v ) ;

void lcd_4bit_Init() ;

int lcd_4bit_putchar_printf(char var, FILE *stream) ;
//



#endif /* AV1624_H_ */

