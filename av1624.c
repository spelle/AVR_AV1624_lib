/*
 * av1624.c
 *
 *  Created on: Oct 2, 2013
 *      Author: lhotse
 */



#include "av1624.h"



uint8_t nbOutCharLine = 0 ;
bool_t  currentLine = high_v ;

void lcd_4bit_Send( bool_t rs_v, bool_t rw_v, byte v )
{
	AV1624_CTL_PORT = (AV1624_CTL_PORT & ~(1 << AV1624_CTL_RS)) | (rs_v << AV1624_CTL_RS) ;

	AV1624_CTL_PORT = (AV1624_CTL_PORT & ~(1 << AV1624_CTL_RW)) | (rw_v << AV1624_CTL_RW) ;

	AV1624_DATA_PORT = (AV1624_DATA_PORT & ~(AV1624_DATA_MASK)) | (v & AV1624_DATA_MASK) ;

	AV1624_CTL_PORT |= ( 1 << AV1624_CTL_E ) ;
	delay(10) ;
	AV1624_CTL_PORT &= ~(1 << AV1624_CTL_E) ;
	delay(10) ;

	AV1624_DATA_PORT = (AV1624_DATA_PORT & ~(AV1624_DATA_MASK)) | ((v<<4) & AV1624_DATA_MASK) ;

	AV1624_CTL_PORT |= ( 1 << AV1624_CTL_E ) ;
	delay(10) ;
	AV1624_CTL_PORT &= ~(1 << AV1624_CTL_E) ;
	delay(10) ;
}

void lcd_4bit_SendCmd( byte v )
{
	lcd_4bit_Send( low_v, low_v, v ) ;
}

void lcd_4bit_WriteData( byte v )
{
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

	return 0;
}



#endif /* AV1624_H_ */
