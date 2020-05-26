#include "Controller.h"

//global variables
static char serial_string[200] = {0};
static char lcd_string [50] = {0};
static const uint16_t NO_MSG_RECEIVED_SHUTDOWN_TIME_MS = 500;
static const uint16_t LED_NOTIFICATION_FLASH_TIME_MS = 5;

//LED coms pins
#define COMS_LED_SUCCESS_PIN PA1;
#define COMS_LED_FAILED_PIN PA0;
#define AUTONOMY_ENABLED_LED_PIN PA2;

bool autonomyEnabled = false;
bool msgRecievedSuccessful = false;
uint8_t fsmComState = 0;
uint8_t recvDataBytes [4] = {0};

uint32_t lastReceiveMSGLedFlashTime = 0;
uint32_t lastFailedReceiveMSGLedFlashTime = 0;

int map (int a, int b, int c, int d);
void setNotificationLEDS(uint32_t currentTime, uint32_t lastSuccessLEDTime, uint32_t lastFailedLEDTime, bool autonomyEnabled);

int main(void)
{
	uint32_t lastMsgSendTime = 0;
	uint32_t currentTime = 0;
	
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
	//Sets the led output pins
	DDRA |= (1<<COMS_LED_SUCCESS_PIN) | (1<<COMS_LED_FAILED_PIN) | (1<<AUTONOMY_ENABLED_LED_PIN);
	
	while(1)
	{
		//get current time
		currentTime = milliseconds;
		
		//Read joystick values
		joystick1 [1] = adc_read (PF0);
		joystick1 [0] = adc_read (PF1);
		joystick2 = adc_read (PF2);
		
		//////////////////////////////////////////////////////////////////////////
		//		COMS
		//////////////////////////////////////////////////////////////////////////
		
		//SEND MSG
		if (currentTime - lastMsgSendTime >= 100)
		{
			//prep data within a range of 0 - 253
			uint8_t databytes [4] = {0};
			databytes [0] = autonomyEnabled;
			databytes [1] = map (joystick1 [0], 1023, 0, 253);
			databytes [2] = map (joystick1 [1], 1023, 0, 253);
			databytes [3] = map (joystick2, 1023, 0, 253);
			
			lastMsgSendTime = currentTime;
			
			serial2_write_byte (0xFF);	//Send start byte
			
			for (int i = 0; i < 4; i ++)
			{
				serial2_write_byte (databytes [i]);
			}
			
			serial2_write_byte (0xFE);	//Send stop byte
		}
		
		//RECIEVE MSG
		if (msgRecievedSuccessful)
		{
			autonomyModeEnabled = recvDataBytes [0] == 1;
			lastMSGReceiveTime = currentTime; 
			lastReceiveMSGLedFlashTime = currentTime; 

			//read distance values
			for (int i = 0; i < 3; i ++)
			{
				distSensors [i] = map (recvDataBytes [i], 253, 0, 1023);
			}
			
			sprintf (serial_string, "1: %3d 2: %3d 3: %3d\r", distSensors [0], distSensors [1], distSensors [2]);
			serial0_print_string(serial_string);
			
			lcd_goto (0x00);
			sprintf (lcd_string, "1: %3d || 2: %3d", distSensors [0], distSensors [1]);
			lcd_puts (lcd_string);
			
			lcd_goto (0x40);
			sprintf (lcd_string, "3: %3d", distSensors [2]);
			lcd_puts (lcd_string);
			
			msgRecievedSuccessful = false;
		}

		//LED comms
		setNotificationLEDS(currentTime, lastReceiveMSGLedFlashTime, lastFailedReceiveMSGLedFlashTime, autonomyModeEnabled)	
	}
	
	return(1);
}

//convert range of a number from a-b to d-c
int map(int a, int b, int c, int d)
{
	return (((double)a/(double)b) * (d-c)) + c;
}

//LED Notification function
void setNotificationLEDS(uint32_t currentTime, uint32_t lastSuccessLEDTime, uint32_t lastFailedLEDTime, bool autonomyEnabled)
{
	if(currentTime > lastSuccessLEDTime + LED_NOTIFICATION_FLASH_TIME_MS)
	{
		PORTA &= ~(1<<COMS_LED_SUCCESS_PIN);
	}
	else
	{
		PORTA |= (1<<COMS_LED_SUCCESS_PIN);
	}

	if(currentTime > lastFailedLEDTime + LED_NOTIFICATION_FLASH_TIME_MS)
	{
		PORTA &= ~(1<<COMS_LED_FAILED_PIN);
	}
	else
	{
		PORTA |= (1<<COMS_LED_FAILED_PIN);
	}

	if(autonomyEnabled)
	{
		PORTA &= ~(1<<AUTONOMY_ENABLED_LED_PIN)
	}
	else
	{
		PORTA |= (1<<AUTONOMY_ENABLED_LED_PIN);
	}
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
