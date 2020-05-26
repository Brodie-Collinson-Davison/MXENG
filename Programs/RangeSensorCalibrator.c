#include "Controller.h"
#include <string.h>
#include <math.h>

static char serial_string[200] = {0};
static bool sampleToggle = true;
static bool clearBuffers = false;

ISR(INT0_vect)
{
	static uint32_t lastPressTime = 0;
	static uint16_t debounceDelay = 200;
	
	if (milliseconds > lastPressTime + debounceDelay)
	{
		if (sampleToggle)
		clearBuffers = true;
		
		sampleToggle = !sampleToggle;
	}
}

void ftos (float fVal, char* outStr)
{
	int dVal, dec, i;

	fVal += 0.005;   // added after a comment from Matt McNabb, see below.

	dVal = fVal;
	dec = (int)(fVal * 100) % 100;

	memset(outStr, 0, 100);
	outStr[0] = (dec % 10) + '0';
	outStr[1] = (dec / 10) + '0';
	outStr[2] = '.';

	i = 3;
	while (dVal > 0)
	{
		outStr[i] = (dVal % 10) + '0';
		dVal /= 10;
		i++;
	}
}

int main(void)
{
	serial0_init();
	adc_init();
	milliseconds_init();
	_delay_ms(20);
	
	DDRF &= ~(1<<0)|~(1<<1);
	
	DDRD &= ~(1<<PD0);
	PORTD |= (1<<PD0);
	EICRA |= (1<<ISC01)|(1<<ISC11);
	EICRA &= ~(1<<ISC00);
	EIMSK |= (1<<INT0);
	sei();
	
	uint16_t buffer1 [1000]= {0};
	uint16_t buffer2 [1000]= {0};
	
	uint16_t bufferSize = sizeof(buffer1) / sizeof (uint16_t);
	uint16_t iteration = 0;
	
	while(1)
	{
		if (sampleToggle && iteration < bufferSize) {
			
			buffer1 [iteration] = adc_read (PF0);
			buffer2 [iteration] = adc_read (PF1); 
			
			iteration++;
			
		} else if (iteration >= bufferSize)
		{
			uint32_t counter1 = 0;
			uint32_t counter2 = 0;
			
			for (int i = 0; i < bufferSize; i ++)
			{
				counter1 += buffer1[i];
				counter2 += buffer2[i];
			}
			
			float avg1 = (counter1 / bufferSize);
			float avg2 = (counter2 / bufferSize);
			
			float dist1 = 3554.9/avg1 - 3.6994;
			float dist2 = 2986.6/avg2 - 2.2201;
			
			char avg1s[100];
			char avg2s[100];
			char dist1s[100];
			char dist2s[100];
			
			ftos(avg1, avg1s);
			ftos(avg2, avg2s);
			ftos(dist1, dist1s);
			ftos(dist2, dist2s);
			
			sprintf(serial_string, "ADC VALUES: %s  |  %s || DIST: %d cm | %d cm\r", dist1s, dist1s, (int)round(dist1), (int)round(dist2));
			serial0_print_string(serial_string);
			clearBuffers = true;
		}
		
		if (clearBuffers || (iteration >= 500 && !sampleToggle))
		{
			iteration = 0;
			for (int i = 0; i < 500; i ++)
			{
				if (buffer1[i] = 0)
				break;
				
				buffer1[i] = 0;
				buffer2[i] = 0;
			}
			clearBuffers = false;
		}
	}
	return(1);
}
