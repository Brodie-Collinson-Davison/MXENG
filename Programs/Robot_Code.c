#include "Controller.h"
#include <math.h>

//////////////////////////////////////////////////////////////////////////
//			GLOBAL VARS
//////////////////////////////////////////////////////////////////////////

static const uint16_t DISTANCE_MEASUREMENT_BUFFER_SIZE = 200;
static const uint16_t NO_MSG_RECEIVED_SHUTDOWN_TIME_MS = 500;
static const uint16_t LED_NOTIFICATION_FLASH_TIME_MS = 5;

//coms
#define COMS_LED_SUCCESS_PIN PA1
#define COMS_LED_FAILED_PIN PA0
#define AUTONOMY_MODE_ENABLE_LED_PIN PA2

//Sensor pins
#define LONG_RANGE_SENSOR PF0
#define SHORT_RANGE_SENSOR_LEFT PF1
#define SHORT_RANGE_SENSOR__RIGHT PF2

//pwm pins
#define M1_PWM_PIN_1 PB5
#define M1_PWM_PIN_2 PB6
#define M2_PWM_PIN_1 PE3
#define M2_PWM_PIN_2 PE4
#define SERVO_PWM_PIN PH3

static char serial_string[200] = {0};

uint8_t recvDataBytes [4] = {0};
uint8_t fsmComState = 0;
bool msgRecieveSuccessful = false;

uint32_t lastReceiveMSGLedFlashTime = 0;
uint32_t lastFailedReceiveMSGLedFlashTime = 0;

//function declarations

float mapf (float a, float b, float c, float d);
int mapi (int a, int b, int c, int d);
void setMotorSpeed (float joyPos[]);
void setNotificationLEDS (uint32_t currentTime, uint32_t lastSuccessLEDTime, uint32_t lastFailLEDTime, bool autonomyEnabled);

int main(void)
{
	//////////////////////////////////////////////////////////////////////////
	//			Declarations
	//////////////////////////////////////////////////////////////////////////
	
	bool operating = false;
	bool autonomyModeEnabled = false;
	
	//coms
	uint32_t lastMSGSendTime = 0;
	uint32_t lastMSGReceiveTime = 0;
	uint32_t currentTime = 0;
	
	//distance sampling
	uint16_t avgDistADCValue [3] = {0};
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
	DDRF &= ~(1<<LONG_RANGE_SENSOR)|~(1<<SHORT_RANGE_SENSOR_LEFT)|~(1<<SHORT_RANGE_SENSOR__RIGHT);
	
	//Set coms led output pins
	DDRA |= (1<<COMS_LED_SUCCESS_PIN) | (1<<COMS_LED_FAILED_PIN) | (1<<AUTONOMY_MODE_ENABLE_LED_PIN);
	
	//		TIMER 1		//
	
	DDRB |= (1<<M1_PWM_PIN_1)|(1<<M1_PWM_PIN_2);				//Set OC1A and OC1B as OUTPUT
	TCCR1A |= (1<<COM1A1)|(1<<COM1B1);	//Set on down, clear on up (OC1A & OC1B)
	TCCR1B |= (1<<CS10);				//SET PRESCALER to 1
	TCCR1B |= (1<<WGM13);				//Select PWM Mode (Phase & frequency Correct)
	ICR1 = 800;							//Set top to 800
	
	//		TIMER 3		//
	
	DDRE |= (1<<M2_PWM_PIN_1)|(1<<M2_PWM_PIN_2);				//Set OC3A and OC3B as Output
	TCCR3A |= (1<<COM3A1)|(1<<COM3B1);	//Set on down, clear on up (OC3A & OC3B)
	TCCR3B |= (1<<CS10);				//Set PRE to 1
	TCCR3B |= (1<<WGM13);				//Select PWM Mode (Phase and frequency correct) [MODE 8]
	ICR3 = 800;							//Set top to 800
	
	//		TIMER 4		//
	DDRH |= (1<<SERVO_PWM_PIN);						//Set OC4A as Output
	TCCR4A |= (1<<COM4A1);				//Set on down, clear on up (OC4A)
	TCCR4B |= (1<<CS11);				//Set PRE to 8
	TCCR4B |= (1<<WGM13);				//Select PWM Mode (Phase and frequency correct) [MODE 8]
	ICR4 = 20000;						//Set top to 20000
	
	//Globally enable interrupts
	sei ();
	
	//Enable coms ISR
	UCSR2B |= (1 << RXCIE2); // Enable the USART Receive Complete interrupt (USART_RXC)
	
	//////////////////////////////////////////////////////////////////////////
	//			Program Loop
	//////////////////////////////////////////////////////////////////////////
	
	while(1)
	{
		//		COMS
		//////////////////////////////////////////////////////////////////////////
		
		//get current time
		currentTime = milliseconds;
		
		//Recieve Message
		if (msgRecieveSuccessful)
		{
			//incoming msg debug
			//sprintf (serial_string, "1: %3d || 2: %3d || 3; %3d || 4: %3d\r", recvDataBytes [0], recvDataBytes [1], recvDataBytes [2], recvDataBytes [3]);
			//serial0_print_string(serial_string);
			
			autonomyModeEnabled = recvDataBytes [0] == 1;
			lastMSGReceiveTime = currentTime;
			lastReceiveMSGLedFlashTime = currentTime;
			msgRecieveSuccessful = false;
			
			//		OPERATIONS
			//////////////////////////////////////////////////////////////////////////
			
			//Check if has recieved a message recently
			if (currentTime - lastMSGReceiveTime >= NO_MSG_RECEIVED_SHUTDOWN_TIME_MS)
			{
				operating = false;
				} else {
				
				operating = true;
				}
			
			if (operating)
			{
				//get joystick values between -1 and 1
				float joyPos [2] = {mapf ((float)recvDataBytes [1], 253.0f, -1.0f, 1.0f), mapf ((float)recvDataBytes [2], 253.0f, -1.0f, 1.0f)};
				setMotorSpeed (joyPos);
				
				//servo
				OCR4A = mapi (recvDataBytes [3], 253, 1750, 2000);
				
				} else {
				
				//set to 0,0 to stop motors
				float joyPos [2] = {0.0, 0.0};
				setMotorSpeed (joyPos);
				
				//servo stop signal
				OCR4A = 1870;
			}
		}
		
	
		
		//Send Message
		if (currentTime - lastMSGSendTime >= 100)
		{
			lastMSGSendTime = currentTime;
			
			serial2_write_byte(0xFF);	//Send start byte
			
			//send data
			serial2_write_byte (mapi (avgDistADCValue [0], 1023, 0, 253));
			serial2_write_byte (mapi (avgDistADCValue [1], 1023, 0, 253));
			serial2_write_byte (mapi (avgDistADCValue [2], 1023, 0, 253));
			
			serial2_write_byte(0xFE);	//Send stop byte
		}
		
		
		//		DISTANCE SAMPLING		//
		//////////////////////////////////////////////////////////////////////////
		
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
			
			//sprintf (serial_string, "avgs: 1: %4d || 2: %4d || 3: %4d\r", avgDistADCValue [0], avgDistADCValue [1], avgDistADCValue [2]);
			//serial0_print_string (serial_string);
			
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

//		SCLAING FUNCTIONS		//

float mapf (float a, float b, float c, float d)
{
	return ((a / b) * (d - c)) + c;
}

int mapi (int a, int b, int c, int d)
{
	return (((double)a/(double)b) * (d-c)) + c;
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
	//400 = midpoint as OCRnA / OCRnB ranges from 0 - 800
	OCR1A = 400 + (int)(motorMapping[0] * 400);
	OCR1B = 400 - (int)(motorMapping[0] * 400);
	OCR3A = 400 - (int)(motorMapping[1] * 400);
	OCR3B = 400 + (int)(motorMapping[1] * 400);
	
	sprintf (serial_string, "x: %3d y:%3d || M1: %3d %3d M2: %3d %4d\r", (int)(joyPos [0] * 100), (int)(joyPos [1] * 100), OCR1A, OCR1B, OCR3A, OCR3B);
	serial0_print_string (serial_string);
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
