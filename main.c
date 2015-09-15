
// How long we want to blink the LED for
#define LED_DURATION 1000

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>        // Supplied Watch Dog Timer Macros 
#include <util/delay.h>

#define BAUD            9600

#define STX_PORT        PORTB
#define STX_DDR         DDRB
#define STX_BIT         0


void sputchar( uint8_t c )
{
	c = ~c;
	STX_PORT &= ~(1<<STX_BIT);            // start bit

	// 10 bits
	uint8_t i = 10;
	for( ; i > 0; i-- )
	{    

		_delay_us( 1e6 / BAUD );            // bit duration
		if( c & 1 )
		{
			STX_PORT &= ~(1<<STX_BIT);        // data bit 0
		}
		else
		{			
			STX_PORT |= 1<<STX_BIT;           // data bit 1 or stop bit
		}
		c >>= 1;
	}
} 

void sputs( void *s )
{
	uint8_t *s1 = s;
	while( *s1 ) sputchar( *s1++ );
}

uint16_t temp_meas()
{
	ADMUX = 1<<REFS1 | 0<<REFS0 | 0<<ADLAR | 0<<REFS2 | 1<<MUX3 | 1<<MUX2 | 1<<MUX1 | 1<<MUX0;
	        // REF = 1.1V, 10Bit, Temp
	ADCSRA = 1<<ADEN | 1<<ADSC | 0<<ADATE | 0<<ADIF | 0<<ADIE | 1<<ADPS2 | 1<<ADPS1 | 0<<ADPS0;
	        // 8MHz / 64 = 125kHz, Start Conversion
	while( ADCSRA & 1<<ADSC );            // until conversion done
	return  ADC; 
}

/** 
 * This will enable the analog to digital converter which allows
 * us to read an anlog input value on a pin.
 */
void enable_adc()
{
	// Configure ADMUX register
	ADMUX |=
		(1 << MUX2) |  
		(1 << MUX3) | // Use Band Gap as Vref
		(0 << REFS0)| 
		(0 << REFS1); // Set refs0 and 1 to 0 to use Vcc as Vref

	// Configure ADCSRA register
	ADCSRA |=
		(1 << ADEN)| // Set ADEN bit to 1 to enable the ADC
		(0 << ADSC); // Set ADSC to 0 to make sure no conversions are happening
}

void disable_adc()
{
	ADCSRA &= ~(1 << ADEN);
}

long read_vcc()
{
	// Enable the ADC register
	enable_adc();
	
	// Wait for Vref to settle
  	_delay_ms(250);

	// Start conversion
	ADCSRA |= (1 << ADSC);

	// Measure
	while (((ADCSRA >> ADSC) & 1)){}

	uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
	uint8_t high = ADCH; // unlocks both

	long result = (high<<8) | low;

	result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  	return result; // Vcc in millivolts
}

/*
Port B Data Direction Register (controls the mode of all pins within port B)
DDRB is 8 bits: [unused:unused:DDB5:DDB4:DDB3:DDB2:DDB1:DDB0]
*/
void init_pins()
{
	// LED PIN
	DDRB = (1 << PB4);
}

void blink()
{
	PORTB ^= (1 << PB4);
	_delay_ms(LED_DURATION);
}

int main ()
{
	char s[20];

	STX_PORT |= 1<<STX_BIT;
	STX_DDR |= 1<<STX_BIT;

	enable_adc();
	sputs( "Hello!\n\r" );

	for (;;)
	{
		uint8_t i = 12;
		for( ; i > 0; i-- )
		{
			_delay_ms(2000);
			utoa(read_vcc(), s, 10 );
			sputs( s );
			sputs( " " );
		}
		sputs( "\n\r" );
		blink();
	}
}