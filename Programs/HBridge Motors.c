#include "Controller.h"
#include <math.h>

static char serial_string[200] = {0};

//////////////////////////////////////////////////////////////////////////
//METHODS
//////////////////////////////////////////////////////////////////////////

int map(int a, int b, int c, int d)
{
	return (((double)a/(double)b) * (d-c)) + c;
}

int main(void)
{
	uint16_t joystickX = 0;
	uint16_t joystickY = 0;
	
	adc_init();
	_delay_ms(20);
	
	//Set Analogue pins 0,1 as INPUTS
	DDRF &= ~(1<<0)|~(1<<1);
	
	//		TIMER 1		//
	
	DDRB |= (1<<5)|(1<<6);				//Set OC1A and OC1B as OUTPUT
	TCCR1A |= (1<<COM1A1)|(1<<COM1B1);	//Set on down, clear on up (OC1A & OC1B)
	TCCR1B |= (1<<CS10);				//SET PRESCALER to 1
	TCCR1B |= (1<<WGM13);				//Select PWM Mode (Phase & frequency Correct)
	ICR1 = 800;							//Set top to 800
	
	//		TIMER 3		//
	
	DDRE |= (1<<3)|(1<<4);				//Set OC3A and OC3B as Output
	TCCR3A |= (1<<COM3A1)|(1<<COM3B1);	//Set on down, clear on up (OC3A & OC3B)
	TCCR3B |= (1<<CS10);				//Set PRE to 1
	TCCR3B |= (1<<WGM13);				//Select PWM Mode (Phase and frequency correct) [MODE 8]
	ICR3 = 800;							//Set top to 800
	
	while(1)
	{
		//get joystick values
		joystickX = adc_read(PINF0);
		joystickY = adc_read(PINF1);
		
		//Set compare registers
		OCR3A = map(joystickX, 1023, 0, 800);
		OCR3B = map(joystickX, 1023, 800, 0);
		OCR1A = map (joystickY, 1023, 0, 800);
		OCR1B = map (joystickY, 1023, 800, 0);
	}
	
	return(1);
}
