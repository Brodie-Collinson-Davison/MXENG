#include "Controller.h"
#include <math.h>

//////////////////////////////////////////////////////////////////////////
//			GLOBAL VARS
//////////////////////////////////////////////////////////////////////////

static char serial_string[200] = {0};

uint8_t lastMSGSendTime = 0;
uint8_t recvDataBytes [4] = {0};
uint8_t fsmComState = 0;
bool msgRecieveSuccessful = false;

float mapf (float a, float b, float c, float d);
int mapi (int a, int b, int c, int d);

int main(void)
{
	//////////////////////////////////////////////////////////////////////////
	//			Declarations
	//////////////////////////////////////////////////////////////////////////
	
	
	
	//////////////////////////////////////////////////////////////////////////
	//		Initialisation
	//////////////////////////////////////////////////////////////////////////
	
	adc_init();
	milliseconds_init();
	serial2_init();
	serial0_init();
	_delay_ms(20);
	
	//Set Analogue pins 0,1 as INPUTS
	DDRF &= ~(1<<0)|~(1<<1);
	
	UCSR2B |= (1 << RXCIE2); // Enable the USART Receive Complete interrupt (USART_RXC)
	
	//		TIMER 1		//
	
	DDRB |= (1<<5)|(1<<6);				//Set OC1A and OC1B as OUTPUT
	TCCR1A |= (1<<COM1A1)|(1<<COM1B1);	//Set on down, clear on up (OC1A & OC1B)
	TCCR1B |= (1<<CS10);				//SET PRESCALER to 1
	TCCR1B |= (1<<WGM13);				//Select PWM Mode (Phase & frequency Correct)
	ICR1 = 800;							//Set top to 80069
	
	//		TIMER 3		//
	
	DDRE |= (1<<3)|(1<<4);				//Set OC3A and OC3B as Output
	TCCR3A |= (1<<COM3A1)|(1<<COM3B1);	//Set on down, clear on up (OC3A & OC3B)
	TCCR3B |= (1<<CS10);				//Set PRE to 1
	TCCR3B |= (1<<WGM13);				//Select PWM Mode (Phase and frequency correct) [MODE 8]
	ICR3 = 800;							//Set top to 800
	
	//////////////////////////////////////////////////////////////////////////
	//			Program Loop
	//////////////////////////////////////////////////////////////////////////
	
	while(1)
	{
		if (msgRecieveSuccessful)
		{
			//get joystick values between -1 and 1
			float joyPos [2] = {mapf ((float)recvDataBytes [1], 253.0f, -1.0f, 1.0f), mapf ((float)recvDataBytes [2], 253.0f, -1.0f, 1.0f)};
			
			//motor direction mapping
			//fwd =  1, 1	//bck = -1, -1
			//lft = -1, 1	//rght = 1, -1
			float motorMapping [2] =
			{
				joyPos[0] + joyPos[1],
				joyPos[1] - joyPos[0]
			};
			
			//normalization
			if (motorMapping[0] > 1.0f)
			motorMapping[0] = 1.0f;
			if (motorMapping[0] < -1.0f)
			motorMapping[0] = -1.0f;
			if (motorMapping[1] > 1.0f)
			motorMapping[1] = 1.0f;
			if (motorMapping[1] < -1.0f)
			motorMapping[1] = -1.0f;
			
			//set compare values for pwm generation
			//400 = midpoint as OCRnA / OCRnB ranges from 0 - 800
			OCR1A = 400 + (int)(motorMapping[0] * 400);
			OCR1B = 400 - (int)(motorMapping[0] * 400);
			OCR3A = 400 - (int)(motorMapping[1] * 400);
			OCR3B = 400 + (int)(motorMapping[1] * 400);
			
			sprintf (serial_string, "DATA: x: %3d y: %3d || ", recvDataBytes [1], recvDataBytes [2]);
			serial0_print_string (serial_string);
			sprintf (serial_string, "M1: %3d , %3d | M2: %3d , %3d\n", OCR1A, OCR1B, OCR3A, OCR3B);
			serial0_print_string (serial_string);
			
			msgRecieveSuccessful = false;
		}
	}
	
	return(1);
}

//////////////////////////////////////////////////////////////////////////
//			METHODS
//////////////////////////////////////////////////////////////////////////

float mapf (float a, float b, float c, float d)
{
	return ((a / b) * (d - c)) + c;
}

int mapi (int a, int b, int c, int d)
{
	return (((double)a/(double)b) * (d-c)) + c;
}

ISR (USART2_RX_vect)
{
	uint8_t inDataByte = UDR2;
	
	switch (fsmComState)
	{
		case 0:
		//WAIT FOR MSG START BYTE
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
		recvDataBytes [3] = inDataByte;
		fsmComState ++;
		break;
		
		case 5:
		
		if (inDataByte == 0xFE)
		{
			msgRecieveSuccessful = true;
		}
		
		fsmComState = 0;
		
		break;
	}
	
	if (inDataByte == 0xFF)
	{
		fsmComState = 1;
	}
}
