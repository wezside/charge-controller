/*
All my ATtiny85 chips have their 8MHz fuse set
by default they run at 1MHz, so adjust accordingly
this constant is used by delay.h, so make sure it stays above the include
*/
#define F_CPU 8000000

// How long we want to blink the LED for
#define LED_DURATION 1000

#include <avr/io.h>
/*
delay.h provides _delay_ms and _delay_us functions
*/
#include <util/delay.h>


/*
Port B Data Direction Register (controls the mode of all pins within port B)
DDRB is 8 bits: [unused:unused:DDB5:DDB4:DDB3:DDB2:DDB1:DDB0]
*/
void init_pins()
{
	// LED PIN
	DDRB = (1 << PB4);
}

/**
 * Blink LED connected to pin PB4
 */
void blink()
{
	PORTB ^= (1 << PB4);
	_delay_ms(LED_DURATION);
}

int main ()
{
	init_pins();
	
	for (;;)
	{
		blink();
	}
}