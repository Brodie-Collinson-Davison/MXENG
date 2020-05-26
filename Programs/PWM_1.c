#include "Controller.h"

static char serial_string[200] = {0};

int map(int a, int b, int c, int d)
{
	return (((double)a/(double)b) * (d-c)) + c;
}

int main(void)
{
	//initialisation section, runs once
	serial0_init();
	adc_init();
	milliseconds_init();
	_delay_ms(20);
	
	//Set OC1A aS OUTPUT
	DDRB |= (1<<5);
	DDRF &= ~(1<<0);
	
	//Set on down, clear on down count
	TCCR1A |= (1<<COM1A1);
	
	//SET PRESCALER to 8
	TCCR1B |= (1<<CS11);
	
	//Select PWM Mode (Phase & frequency Correct)
	TCCR1B |= (1<<WGM13);
	
	//Set TOP to 20000
	ICR1 = 20000;
	
	//COMPARE VALUE
	OCR1A = 2000;
	
	uint16_t hor = 0;
	
	while(1)
	{
		hor = adc_read(PF0);
		
		int duty = 0;
		duty = map(hor, 1023, 0, 20000);
		sprintf(serial_string, "%4u\r", duty);
		serial0_print_string(serial_string);
		
		OCR1A = duty;
		
	}
	
	return(1);
}
