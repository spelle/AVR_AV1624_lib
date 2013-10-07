/*
 * av1624.h
 *
 *  Created on: Mar 29, 2013
 *      Author: khumbu
 */

#ifndef AV1624_H_
#define AV1624_H_



#include <avr/io.h>
#include <util/delay.h>



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



void lcd_4bit_Send( bool_t rs_v, bool_t rw_v, byte v )
{
	AV1624_CTL_PORT = (AV1624_CTL_PORT & ~(1 << AV1624_CTL_RS)) | (rs_v << AV1624_CTL_RS) ;

	AV1624_CTL_PORT = (AV1624_CTL_PORT & ~(1 << AV1624_CTL_RW)) | (rw_v << AV1624_CTL_RW) ;

	AV1624_DATA_PORT = (AV1624_DATA_PORT & ~(AV1624_DATA_MASK)) | (v & AV1624_DATA_MASK) ;

	AV1624_CTL_PORT |= ( 1 << AV1624_CTL_E ) ;
	_delay_ms(10) ;
	AV1624_CTL_PORT &= ~(1 << AV1624_CTL_E) ;
	_delay_ms(10) ;

	AV1624_DATA_PORT = (AV1624_DATA_PORT & ~(AV1624_DATA_MASK)) | ((v<<4) & AV1624_DATA_MASK) ;

	AV1624_CTL_PORT |= ( 1 << AV1624_CTL_E ) ;
	_delay_ms(10) ;
	AV1624_CTL_PORT &= ~(1 << AV1624_CTL_E) ;
	_delay_ms(10) ;
}

void lcd_4bit_SendCmd( byte v )
{
	lcd_4bit_Send( low_v, low_v, v ) ;
}

void lcd_4bit_WriteData( byte v )
{
	static uint8_t nbOutCharLine = 0 ;
	static bool_t  currentLine = high_v ;

	if( '\n' != v )
	{
//		nbOutCharLine ++ ;
		lcd_4bit_Send( high_v, low_v, v ) ;
	}
	else
	{
		for( ; nbOutCharLine < NBCHAR_PER_LINE ; nbOutCharLine++ )
		{
			lcd_4bit_Send( high_v, low_v, ' ' ) ;
		}
	}

	if( (NBCHAR_PER_LINE <= nbOutCharLine) || ('\n' == v) )
	{
		lcd_4bit_SendCmd( (currentLine == high_v ? 0xA8 : 0x80) ) ;
		nbOutCharLine = 0 ;
		currentLine = ( currentLine == high_v ? low_v : high_v ) ;
	}
}

void lcd_4bit_Init()
{
	AV1624_DATA_DDR |= AV1624_DATA_DDR_MASK ;
	AV1624_CTL_DDR  |= AV1624_CTL_DDR_MASK ;

	nbOutCharLine = 0 ;
	currentLine = high_v ;

	// Init LCD
	lcd_4bit_SendCmd( 0x33 ) ;
	lcd_4bit_SendCmd( 0x32 ) ;

	// Function Set
	lcd_4bit_SendCmd( 0x28 ) ;

	// Display Off
	lcd_4bit_SendCmd( 0x08 ) ;

	// Entry Mode Set
	lcd_4bit_SendCmd( 0x04 ) ;

	// Display Clear
	lcd_4bit_SendCmd( 0x01 ) ;

	// Home
	lcd_4bit_SendCmd( 0x02 ) ;

	// Display On, Cursor On, Blink
	lcd_4bit_SendCmd( 0x0E ) ;

}



int usart_putchar_printf(char var, FILE *stream)
{
	lcd_4bit_WriteData(var);
//
	return 0;
}
//
static FILE mystdout = FDEV_SETUP_STREAM(usart_putchar_printf, NULL, _FDEV_SETUP_WRITE);



#endif /* AV1624_H_ */

