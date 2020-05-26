#include "Controller.h"

//global variables
static char serial_string[200] = {0};

bool autonomyEnabled = false;
bool msgRecievedSuccessful = false;
uint8_t fsmComState = 0;
uint8_t recvDataBytes [4] = {0};

int main(void)
{
	uint32_t lastMsgSendTime = 0;
	
	uint16_t distSensors [3];
	uint16_t joystick1 [2];
	uint16_t joystick2;
	
	//initialisation section, runs once
	serial0_init ();
	serial2_init ();
	milliseconds_init ();
	adc_init ();
	lcd_init ();
	_delay_ms (20);
	
	
	//button interrupts
	DDRD &= ~(1<<PD0);		// INT0  is also PD0 and we set the DDR to input
	PORTD |= (1<<PD0);		// enable pullupresistor on PD0
	EICRA &= ~(1<<ISC00);	// INT0 to trigger on a FALLING edge
	EIMSK |= (1<<INT0);		// enable INT0 & INT1
	sei();					// globally enable interrupts
	
	UCSR2B |= (1 << RXCIE2); // Enable the USART Receive Complete interrupt (USART_RXC)
	
	//joystick inputs
	DDRF &= ~(1<<PF0) | ~(1<<PF1) | ~(1<<PF2);
	
	while(1)
	{
		//Read joystick values
		joystick1 [1] = adc_read (PF0);
		joystick1 [0] = adc_read (PF1);
		joystick2 = adc_read (PF2);
		
		//////////////////////////////////////////////////////////////////////////
		//		COMS
		//////////////////////////////////////////////////////////////////////////
		
		//SEND MSG
		if (milliseconds - lastMsgSendTime >= 1000)
		{
			//prep data within a range of 0 - 253
			uint8_t databytes [4] = {0};
			databytes [0] = map (joystick2, 1023, 0, 253);
			databytes [1] = map (joystick1 [0], 1023, 0, 253);
			databytes [2] = map (joystick1 [1], 1023, 0, 253);
			databytes [3] = map (joystick2, 1023, 0, 253);
			
			for (int i = 0; i < 4; i ++)
			{
				sprintf(serial_string, "%d [%3d]\n", i, databytes[i]);
				serial0_print_string(serial_string);
			}
			
			serial2_write_byte (0xFF);	//Send start byte
			
			//write databytes to serial2
			for (int i = 0; i < 4; i ++)
			{
				serial2_write_byte (databytes [i]);
			}
			
			serial2_write_byte (0xFE);	//Send stop byte
			
			lastMsgSendTime = milliseconds;
		}
		
		//RECIEVE MSG
		if (msgRecievedSuccessful)
		{
			//read distance values
			for (int i = 0; i < 3; i ++)
			{
				distSensors [i] = map (recvDataBytes [i], 253, 0, 1023);
			}
			
			msgRecievedSuccessful = false;
		}
		
	}
	
	return(1);
}

//convert range of a number from a-b to d-c
int map(int a, int b, int c, int d)
{
	return (((double)a/(double)b) * (d-c)) + c;
}

//receive message interrupt
ISR (USART2_RX_vect)
{
	uint8_t inDataByte = UDR2;
	
	switch (fsmComState)
	{
		case 0:
			//IDLE LOOP WAITING FOR MSG START BYTE
		break;
		
		case 1:
			recvDataBytes [0] = inDataByte;
			fsmComState ++;
		break;
		
		case 2:
			recvDataBytes [1] = inDataByte;
			fsmComState ++;
		break;
		
		case 3:
			recvDataBytes [2] = inDataByte;
			fsmComState ++;
		break;
		
		case 4:
			if (inDataByte == 0xFE)
			{
				msgRecievedSuccessful = true;
			}
		
			fsmComState = 0;
		break;
	}
	
	if (inDataByte == 0xFF)
	{
		fsmComState = 1;
	}
}

//Start / stop autonomy button
ISR (INT0_vect)
{
	static const uint8_t debounceDelay = 100;
	static uint32_t lastPressedTime = 0;
	
	if (milliseconds > lastPressedTime + debounceDelay)
	{
		autonomyEnabled = !autonomyEnabled;
		lastPressedTime = milliseconds;
	}
}
