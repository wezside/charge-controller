
// LED 
#define LED_PIN PB4
#define LED_DURATION 1000

// Bluetooth switch on/off
#define BLE_PIN PB2

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>        // Supplied Watch Dog Timer Macros 
#include <util/delay.h>

#define BAUD            9600

#define STX_PORT        PORTB
#define STX_DDR         DDRB
#define STX_BIT         4


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

void disable_adc()
{
	ADCSRA &= ~(1 << ADEN);
}

long read_vcc()
{
	// Clear previous
	ADMUX &= ~0x0F;

	// Configure ADMUX register
	ADMUX |=
		(0 << ADLAR)|
		(0 << MUX0) | 
		(0 << MUX1) | 
		(1 << MUX2) |  
		(1 << MUX3) | // Use Band Gap as Vref
		(0 << REFS0)| 
		(0 << REFS1); // Set refs0 and 1 to 0 to use Vcc as Vref

	// Configure ADCSRA register
	ADCSRA |=
		(1 << ADEN)| // Set ADEN bit to 1 to enable the ADC
		(0 << ADSC); // Set ADSC to 0 to make sure no conversions are happening

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

void wdt_reset_safety()
{
	cli(); 
	// The MCU Status Register provides information on which reset source caused an MCU Reset.
	// Test to see if it was the Watchdog timer - if it was disable it to avoid an infinite reset 
	// loop.
	if (MCUSR & (1 << WDRF))
	{            
		// If a reset was caused by the Watchdog Timer...
		MCUSR &= ~(1 << WDRF);                  // Clear the WDT reset flag
		WDTCR |= ((1 << WDCE) | (1 << WDE));    // Enable the WD Change Bit
		WDTCR  = 0x00;                      	// Disable the WDT
	}
	sei();
}

void wdt_reset_safety()
{
	cli(); 
	// The MCU Status Register provides information on which reset source caused an MCU Reset.
	// Test to see if it was the Watchdog timer - if it was disable it to avoid an infinite reset 
	// loop.
	if (MCUSR & (1 << WDRF))
	{            
		// If a reset was caused by the Watchdog Timer...
		MCUSR &= ~(1 << WDRF);                  // Clear the WDT reset flag
		WDTCR |= ((1 << WDCE) | (1 << WDE));    // Enable the WD Change Bit
		WDTCR  = 0x00;                      	// Disable the WDT
	}
	sei();
}

void init_wdt()
{
	// Set up Watch Dog Timer for Inactivity
	WDTCR |= ((1 << WDCE) | (1 << WDE));    // Enable the WD Change Bit
	WDTCR  = (1 << WDIE) |               	// Enable WDT Interrupt
           	 (1 << WDP2) | (1 << WDP1);     // Set Timeout to ~1 seconds
}

void sleep_avr()
{
	// Disable analog to digital converter
	disable_adc();

	set_sleep_mode(SLEEP_MODE_PWR_DOWN);    // replaces above statement
	cli();
	sleep_enable();                         // Sets the Sleep Enable bit in the MCUCR Register (SE BIT)
	sei();                                  // Enable interrupts
	sleep_cpu();                            // sleep
	sleep_disable();                        // Disable interrupts

	// Enable global interrupts
	sei(); 
}

/*
Port B Data Direction Register (controls the mode of all pins within port B)
DDRB is 8 bits: [unused:unused:DDB5:DDB4:DDB3:DDB2:DDB1:DDB0]
*/
void init_pins()
{
	// LED PIN
	DDRB = (1 << LED_PIN);
	DDRB = (1 << BLE_PIN);
}

void blink()
{
	PORTB ^= (1 << LED_PIN);
	_delay_ms(LED_DURATION);
}

ISR(WDT_vect)
{
}

int main ()
{
	// Disable the ADC
	disable_adc();

	// Watchdog timer reset safety check
	wdt_reset_safety();
	
	// Initialise watchdog timer
	init_wdt();

	// Identify input and output pins
	init_pins();

	// Buffer for text
	char s[20];

	// Setup direction and port for debug logging
	STX_PORT |= 1<<STX_BIT;
	STX_DDR |= 1<<STX_BIT;

	for (;;)
	{
		// Go to sleep
		sleep_avr();
		
		// Enable BLE module
		// PORTB |= (1 << BLE_PIN);

		// Read VCC and convert to base 10 number
		utoa(read_vcc(), s, 10);

		// Output VCC to soft serial PIN STX_BIT
		sputs(s);
		sputs("\n\r");

		blink();
		blink();
		_delay_ms(1000);
	}
}