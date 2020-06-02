/*
AUTHORS: Brodie Collinson Davison, Sadman Sakib
*/
#include "Controller.h"
#include <math.h>

//////////////////////////////////////////////////////////////////////////
//			GLOBAL VARS
//////////////////////////////////////////////////////////////////////////

static char serial_string[200] = {0};

//Distance measurement buffer size declaration
static const uint16_t DISTANCE_MEASUREMENT_BUFFER_SIZE = 200;

//COMS settings
static const uint16_t LED_NOTIFICATION_FLASH_TIME_MS = 10;
static const uint16_t NO_MSG_RECEIVED_SHUTDOWN_TIME_MS = 500;
static const uint16_t MSG_SEND_DELAY_MS = 100;

//TIMER PWM SETTINGS
static const uint16_t MOTOR_TIMER_COMPARE_REGISTER_TOP = 1300;
static const uint16_t SERVO_TIMER_COMPARE_REGISTER_TOP = 20000;

static const uint16_t SERVO_PWM_MIN_SIGNAL = 1750;
static const uint16_t SERVO_PWM_MAX_SIGNAL = 2000;
static const uint16_t SERVO_PWM_STOP_SIGNAL = 1870;

//COMS led pins
#define COMS_LED_SUCCESS_PIN PA0
#define COMS_LED_FAILED_PIN PA2
#define AUTONOMY_MODE_ENABLE_LED_PIN PA1

//Motor PWM pins
#define M1_PWM_PIN_1 PB5
#define M1_PWM_PIN_2 PB6
#define M2_PWM_PIN_1 PE3
#define M2_PWM_PIN_2 PE4
#define SERVO_PWM_PIN PH3


uint8_t recvDataBytes [4] = {0};
uint8_t fsmComState = 0;
bool msgRecieveSuccessful = false;

uint32_t lastReceiveMSGLedFlashTime = 0;
uint32_t lastFailedReceiveMSGLedFlashTime = 0;

///			FUNCTION DECLARATIONS			///

float mapf (float a, float b, float c, float d);
int mapi (int a, int b, int c, int d);
void setMotorSpeed (float joyPos[]);
void setNotificationLEDS (uint32_t currentTime, uint32_t lastSuccessLEDTime, uint32_t lastFailLEDTime, bool autonomyEnabled);
void initialiseTimers ();
uint16_t convertDistanceADCToCM (uint16_t adcDist);
uint16_t convertLRDistanceADCToCM (uint16_t adcDist);

int main(void)
{
	///		DECLARATIONS		///
	
	bool operating = false;
	bool autonomyModeEnabled = false;
	
	//coms
	uint32_t lastMSGSendTime = 0;
	uint32_t lastMSGReceiveTime = 0;
	uint32_t currentTime = 0;
	
	//distance sampling
	uint16_t distanceValues [3];
	uint16_t avgDistADCValue [3];
	uint16_t distADCValueBuffer [DISTANCE_MEASUREMENT_BUFFER_SIZE][3];
	uint16_t distADCBufferIndex = 0;
	
	//////////////////////////////////////////////////////////////////////////
	//		Initialisation
	//////////////////////////////////////////////////////////////////////////
	
	adc_init();
	milliseconds_init();
	serial2_init();
	serial0_init();
	_delay_ms(20);
	
	//Set distance sensor input pins
	DDRF &= ~(1<<PF0)|~(1<<PF1)|~(1<<PF2);
	//Set coms led output pins
	DDRA |= (1<<COMS_LED_SUCCESS_PIN) | (1<<COMS_LED_FAILED_PIN) | (1<<AUTONOMY_MODE_ENABLE_LED_PIN);
	
	//Sets timers up
	initialiseTimers ();
	
	//Globally enable interrupts
	sei ();
	
	UCSR2B |= (1 << RXCIE2); // Enable the USART Receive Complete interrupt (USART_RXC)
	
	//////////////////////////////////////////////////////////////////////////
	//			Program Loop
	//////////////////////////////////////////////////////////////////////////
	
	while(1)
	{
		///			COMS			///
		
		//get current time
		currentTime = milliseconds;
		
		///			RECEIVE MESSAGE			///
		if (msgRecieveSuccessful)
		{
			//incoming msg debug
			//sprintf (serial_string, "1: %3d || 2: %3d || 3; %3d || 4: %3d\r", recvDataBytes [0], recvDataBytes [1], recvDataBytes [2], recvDataBytes [3]);
			//serial0_print_string(serial_string);
			
			lastMSGReceiveTime = currentTime;
			lastReceiveMSGLedFlashTime = currentTime;
			msgRecieveSuccessful = false;
			
			///			OPERATIONS			///
			
			if (operating)
			{
				//set autonomy mode
				autonomyModeEnabled = recvDataBytes [0] == 1;
				
				//get joystick values between -1 and 1
				float joyPos [2] = {mapf ((float)recvDataBytes [1], 253.0f, -1.0f, 1.0f), mapf ((float)recvDataBytes [2], 253.0f, -1.0f, 1.0f)};
				setMotorSpeed (joyPos);
				
				//servo
				OCR4A = mapi (recvDataBytes [3], 253, SERVO_PWM_MIN_SIGNAL, SERVO_PWM_MAX_SIGNAL);
			}
		}
		
		///			SEND MESSAGE			///
		if (currentTime - lastMSGSendTime >= MSG_SEND_DELAY_MS)
		{
			lastMSGSendTime = currentTime;
			
			serial2_write_byte(0xFF);	//Send start byte
			
			//send data
			for (int i = 0; i < 3; i ++)
			{
				serial2_write_byte (distanceValues [i]);	
			}
						
			serial2_write_byte(0xFE);	//Send stop byte
		}
		
		//Check if has recieved a message recently
		if (currentTime - lastMSGReceiveTime >= NO_MSG_RECEIVED_SHUTDOWN_TIME_MS)
		operating = false;
		else
		operating = true;
		
		if (!operating)
		{
			//set to 0,0 to stop motors
			float joyPos [2] = {0.0, 0.0};
			setMotorSpeed (joyPos);
			
			//servo stop signal
			OCR4A = SERVO_PWM_STOP_SIGNAL;
			
			//autonomy mode disable
			autonomyModeEnabled = false;
		}
		
		//		DISTANCE SAMPLING		//
		if (distADCBufferIndex < (DISTANCE_MEASUREMENT_BUFFER_SIZE - 1))
		{
			distADCValueBuffer [distADCBufferIndex][0] = adc_read (PF0);
			distADCValueBuffer [distADCBufferIndex][1] = adc_read (PF1);
			distADCValueBuffer [distADCBufferIndex][2] = adc_read (PF2);
			
			distADCBufferIndex ++;
			
			//sample debug
			//sprintf (serial_string, "1: %3d 2: %3d 3: %3d\r", distADCValueBuffer [distADCBufferIndex][0], distADCValueBuffer [distADCBufferIndex][1], distADCValueBuffer [distADCBufferIndex][2]);
			//serial0_print_string (serial_string);
		}
		
		//calculate average (buffer is full)
		if (distADCBufferIndex == DISTANCE_MEASUREMENT_BUFFER_SIZE - 1)
		{
			uint64_t count [3] = {0};
			
			//sum each distance sensor value
			for (int i = 0; i < DISTANCE_MEASUREMENT_BUFFER_SIZE; i ++)
			{
				count [0] += distADCValueBuffer [i][0];
				count [1] += distADCValueBuffer [i][1];
				count [2] += distADCValueBuffer [i][2];
			}
			
			//calculate averages
			for (int i = 0; i < 3; i ++)
			{
				avgDistADCValue [i] = count [i] / DISTANCE_MEASUREMENT_BUFFER_SIZE;
			}
			
			distanceValues [0] = convertDistanceADCToCM (avgDistADCValue [0]);
			distanceValues [1] = convertDistanceADCToCM (avgDistADCValue [1]);
			distanceValues [2] = convertLRDistanceADCToCM (avgDistADCValue [2]);
			
			sprintf (serial_string, "avgs: 1: %4d || 2: %4d || 3: %4d\r", avgDistADCValue [0], avgDistADCValue [1], avgDistADCValue [2]);
			serial0_print_string (serial_string);
			
			distADCBufferIndex = 0;
		}
		
		
		//		Notification LEDS		//
		setNotificationLEDS (currentTime, lastReceiveMSGLedFlashTime, lastFailedReceiveMSGLedFlashTime, autonomyModeEnabled);
	}
	
	return(1);
}

//////////////////////////////////////////////////////////////////////////
//			METHODS
//////////////////////////////////////////////////////////////////////////

uint16_t convertDistanceADCToCM (uint16_t adcDist)
{
	return (uint16_t)((2986.6f / adcDist) - 2.2201f);
}

uint16_t convertLRDistanceADCToCM (uint16_t adcDist)
{
	return (uint16_t)((8337.6f / adcDist) - 12.976f);
}

void setNotificationLEDS (uint32_t currentTime, uint32_t lastSuccessLEDTime, uint32_t lastFailLEDTime, bool autonomyEnabled)
{
	if (currentTime > lastSuccessLEDTime + LED_NOTIFICATION_FLASH_TIME_MS)
	{
		PORTA &= ~(1<<COMS_LED_SUCCESS_PIN);
		} else {
		
		PORTA |= (1<<COMS_LED_SUCCESS_PIN);
	}
	
	if (currentTime > lastFailLEDTime + LED_NOTIFICATION_FLASH_TIME_MS)
	{
		PORTA &= ~(1<<COMS_LED_FAILED_PIN);
		} else {
		
		PORTA |= (1<<COMS_LED_FAILED_PIN);
	}
	
	if (autonomyEnabled)
	{
		PORTA |= (1<<AUTONOMY_MODE_ENABLE_LED_PIN);
		} else {
		
		PORTA &= ~(1<<AUTONOMY_MODE_ENABLE_LED_PIN);
	}
}

void setMotorSpeed (float joyPos[])
{
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
	OCR1A = (MOTOR_TIMER_COMPARE_REGISTER_TOP / 2) - (int)(motorMapping[0] * (MOTOR_TIMER_COMPARE_REGISTER_TOP / 2));
	OCR1B = (MOTOR_TIMER_COMPARE_REGISTER_TOP / 2) + (int)(motorMapping[0] * (MOTOR_TIMER_COMPARE_REGISTER_TOP / 2));
	OCR3A = (MOTOR_TIMER_COMPARE_REGISTER_TOP / 2) + (int)(motorMapping[1] * (MOTOR_TIMER_COMPARE_REGISTER_TOP / 2));
	OCR3B = (MOTOR_TIMER_COMPARE_REGISTER_TOP / 2) - (int)(motorMapping[1] * (MOTOR_TIMER_COMPARE_REGISTER_TOP / 2));
	
	//sprintf (serial_string, "x: %3d y:%3d || M1: %3d %3d M2: %3d %3d\r", (int)(joyPos [0] * 100), (int)(joyPos [1] * 100), OCR1A, OCR1B, OCR3A, OCR3B);
	//serial0_print_string (serial_string);
}

void initialiseTimers ()
{
	//		TIMER 1	(M1)	//
	DDRB |= (1<<M1_PWM_PIN_1)|(1<<M1_PWM_PIN_2);	//Set OC1A and OC1B as OUTPUT
	TCCR1A |= (1<<COM1A1)|(1<<COM1B1);				//Set on down, clear on up (OC1A & OC1B)
	TCCR1B |= (1<<CS10);							//SET PRESCALER to 1
	TCCR1B |= (1<<WGM13);							//Select PWM Mode (Phase & frequency Correct)
	ICR1 = MOTOR_TIMER_COMPARE_REGISTER_TOP;		//Set compare register top
	
	//		TIMER 3	 (M2)	//
	DDRE |= (1<<M2_PWM_PIN_1)|(1<<M2_PWM_PIN_2);	//Set OC3A and OC3B as Output
	TCCR3A |= (1<<COM3A1)|(1<<COM3B1);				//Set on down, clear on up (OC3A & OC3B)
	TCCR3B |= (1<<CS10);							//Set PRESCALER to 1
	TCCR3B |= (1<<WGM13);							//Select PWM Mode (Phase and frequency correct) [MODE 8]
	ICR3 = MOTOR_TIMER_COMPARE_REGISTER_TOP;		//Set compare register top
	
	//		TIMER 4  (SERVO)	//
	DDRH |= (1<<SERVO_PWM_PIN);					//Set OC4A as Output
	TCCR4A |= (1<<COM4A1);						//Set on down, clear on up (OC4A)
	TCCR4B |= (1<<CS11);						//Set PRE to 8
	TCCR4B |= (1<<WGM13);						//Select PWM Mode (Phase and frequency correct) [MODE 8]
	ICR4 = SERVO_TIMER_COMPARE_REGISTER_TOP;	//Set compare register top
}

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
			} else {
			
			lastFailedReceiveMSGLedFlashTime = milliseconds;
		}
		
		fsmComState = 0;
		
		break;
	}
	
	if (inDataByte == 0xFF)
	{
		fsmComState = 1;
	}
}
