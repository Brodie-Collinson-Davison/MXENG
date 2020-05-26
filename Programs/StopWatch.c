#include "Controller.h"

static char serial_string[200] = {0};

static const uint8_t debounceDelay = 100;
static uint32_t timed_milliseconds = 0;

//timerFunctions
void timer_start()
{
	TCCR1B |= (1<<CS10);// 00000101 sets the prescaler
	TCCR1B |= (1<<3); //sets the timer to clear on compare = true
}

void timer_stop()
{
	TCCR1B = 0;
}

//Start/ stop button
ISR(INT0_vect)
{
	static uint32_t lastB1PressedTime = 0;
	static bool countTime = false;
	
	if (milliseconds > lastB1PressedTime + debounceDelay)
	{
		countTime = !countTime;
	}
	
	if (countTime)
	{
		timer_start();
	} else 
	{
		timer_stop();
	}
	
	lastB1PressedTime = milliseconds;
}

//button 2
ISR (INT1_vect)
{
	static uint32_t lastB2PressedTime = 0;
	
	if (milliseconds > lastB2PressedTime + debounceDelay)
	{
		timed_milliseconds = 0;
	} 
	
	lastB2PressedTime = milliseconds;
}

ISR(TIMER1_COMPA_vect)
{
	timed_milliseconds ++;
}

int main(void)
{
	//initialisation section, runs once
	serial0_init(); 
	milliseconds_init(); 
	_delay_ms(20);
	
	//button interrupts
	DDRD &= ~(1<<PD0) | ~(1<<PD1);		// INT0  is also PD0 and we set the DDR to input
	PORTD |= (1<<PD0) | (1<<PD1);		// enable pullupresistor on PD0
	EICRA |= (1<<ISC11) | (1<<ISC21);	// these two bits set
	EICRA &= ~(1<<ISC00);				// INT0 to trigger on a FALLING edge
	EIMSK |= (1<<INT0) | (1<<INT1);		// enable INT0 & INT1
	sei();								// globally enable interrupts
	
	//timer setup
	TCCR1A = 0;//disable hardware output
	timer_start();
	TIMSK1 = (1<<OCIE1A);// 00000010 enables the timer      
	OCR1A = 15999;
	
	while(1)
	{
		if (true)
		{
			sprintf(serial_string, "ElapsedTime:%lu.%lu s\r", timed_milliseconds/1000, timed_milliseconds%1000);
			serial0_print_string(serial_string);
		}
	}
	return(1);
} 
