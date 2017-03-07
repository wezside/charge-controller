#define STX_PORT PORTB
#define STX_DDR DDRB
#define STX_BIT 4
#define BAUD 9600

void util_init()
{
	// Setup direction and port for debug logging
	STX_PORT |= 1<<STX_BIT;
	STX_DDR |= 1<<STX_BIT;
}

void sputchar(uint8_t c)
{
	c = ~c;
	STX_PORT &= ~(1 << STX_BIT);            // start bit

	// 10 bits
	uint8_t i = 10;
	for(; i > 0; i--)
	{    
		_delay_us(1e6 / BAUD);	              // bit duration
		if( c & 1 )
		{
			STX_PORT &= ~(1 << STX_BIT);        // data bit 0
		}
		else
		{			
			STX_PORT |= 1 << STX_BIT;           // data bit 1 or stop bit
		}
		c >>= 1;
	}
} 

void sputs(void *s)
{
	uint8_t *s1 = s;
	while(*s1) sputchar(*s1++);
}

inline long map(long x, long in_min, long in_max, long out_min, long out_max, int cap)
{
	long tmp = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	if (cap > 0 && tmp > out_max) tmp = out_max;
	if (cap > 0 && tmp < out_min) tmp = out_min;
	return tmp;
}

void delay_ms(uint16_t count) 
{
	while(count--) _delay_ms(1);
}

void delay_us(uint16_t count) 
{
	while(count--) _delay_us(1);
}

void send(long val)
{
	char s[20];	
	utoa(val, s, 10);
	sputs(s);
	sputs("\n\r");
}
