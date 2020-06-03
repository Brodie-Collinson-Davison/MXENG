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

//Servo PWM signals
static const uint16_t SERVO_PWM_MIN_SIGNAL = 1750;
static const uint16_t SERVO_PWM_MAX_SIGNAL = 2000;
static const uint16_t SERVO_PWM_STOP_SIGNAL = 1870;

//Autonomy stuff
enum Directions {forward, back, turnLeft, turnRight};

uint16_t stallDuration = 0;
uint32_t stallStartTime = 0;
bool stalling = false;

//turning variables
static const uint16_t COMPLETE_ROTATION_TIME_MS = 8450;
uint16_t turnDuration = 0;
uint32_t turnStartTime = 0;
bool isTurning = false;



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

//Distance sensors
#define FWD_DISTANCE_SENSOR_PIN PF2
#define LFT_DISTANCE_SENSOR_PIN PF0
#define RGT_DISTANCE_SENSOR_PIN PF1

#define FWD_DISTANCE_SENSOR_INDEX 2
#define LFT_DISTANCE_SENSOR_INDEX 0
#define RGT_DISTANCE_SENSOR_INDEX 1

//COMS variables
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
void setTravelDirection (enum Directions dir, float speed);
void stopTraveling ();
void turnSetAngle (float angle);

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
	uint16_t distanceValues [3] = {0};
	uint16_t distSensorAvgADCValues [3] = {0};
	uint16_t distSensorADCValueBuffer [DISTANCE_MEASUREMENT_BUFFER_SIZE][3];
	uint16_t distADCBufferIndex = 0;
	
	//autonomy

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
				
				if (autonomyModeEnabled)
				{
					///			AUTONOMY CONTROLS			///
					
					if (stalling)	// Waits for operation to complete
					{
						if (isTurning)
						{
							if (currentTime - turnStartTime >= turnDuration)
							{
								isTurning = false;
								stopTraveling ();
							}
						}
						
						//end stalling check
						if (currentTime - stallStartTime >= stallDuration)
							stalling = false;
						
					} else {

						//go forward naturally
						if (distanceValues [FWD_DISTANCE_SENSOR_INDEX] <= 15)
						{
							stopTraveling ();
							
							if (distanceValues [LFT_DISTANCE_SENSOR_INDEX] > distanceValues [RGT_DISTANCE_SENSOR_INDEX])
							{
								turnSetAngle (-60);
							} else if (distanceValues [RGT_DISTANCE_SENSOR_INDEX] > distanceValues [LFT_DISTANCE_SENSOR_INDEX]) {
								turnSetAngle (60);	
							} else if (distanceValues [LFT_DISTANCE_SENSOR_INDEX] == distanceValues [RGT_DISTANCE_SENSOR_INDEX])
							{
								//sample both 90* left and 90* right distance with long range sensor and figure out which way to go
								
								//for now turn right
								turnSetAngle (90);	
							}
								
						} else if (distanceValues [LFT_DISTANCE_SENSOR_INDEX] >= 5 && distanceValues [RGT_DISTANCE_SENSOR_INDEX] >= 5) {
							
							setTravelDirection (forward, 1);	
						}
						
						//turn away from side obstacles
						if (distanceValues [LFT_DISTANCE_SENSOR_INDEX] < 5)
						{
							turnSetAngle (30);
						} else if (distanceValues [RGT_DISTANCE_SENSOR_INDEX] < 5)
						{
							turnSetAngle (-30);
						}
						//stop if fwd distance too close
							//figure out what way to turn
							//if no way is prefered go right
						
					}
					
				} else {
					
					//disable autonomy variables
					isTurning = false;
					stalling = false;
					
					///			MAUNAL CONTROLS			///
					
					//motors
					float joyPos [2] = {mapf ((float)recvDataBytes [1], 253.0f, -1.0f, 1.0f), mapf ((float)recvDataBytes [2], 253.0f, 1.0f, -1.0f)};
					setMotorSpeed (joyPos);
					
					//servo
					OCR4A = mapi (recvDataBytes [3], 253, SERVO_PWM_MIN_SIGNAL, SERVO_PWM_MAX_SIGNAL);
				}
			}
		}
		
		///			SEND MESSAGE			///
		if (currentTime - lastMSGSendTime >= MSG_SEND_DELAY_MS)
		{
			lastMSGSendTime = currentTime;
			
			serial2_write_byte(0xFF);	//Send start byte
			
			//send distance values
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
			stopTraveling ();
				
			//servo stop signal
			OCR4A = SERVO_PWM_STOP_SIGNAL;
			
			//autonomy mode disable
			autonomyModeEnabled = false;
		}
		
		//		DISTANCE SAMPLING		//
		
		//fill buffer
		if (distADCBufferIndex < DISTANCE_MEASUREMENT_BUFFER_SIZE - 1)
		{
			distSensorADCValueBuffer [distADCBufferIndex][LFT_DISTANCE_SENSOR_INDEX] = adc_read (LFT_DISTANCE_SENSOR_PIN);
			distSensorADCValueBuffer [distADCBufferIndex][RGT_DISTANCE_SENSOR_INDEX] = adc_read (RGT_DISTANCE_SENSOR_PIN);
			distSensorADCValueBuffer [distADCBufferIndex][FWD_DISTANCE_SENSOR_INDEX] = adc_read (FWD_DISTANCE_SENSOR_PIN);
			
			distADCBufferIndex ++;
		}
		
		//buffer full, calculate avg adc values and distances
		if (distADCBufferIndex == DISTANCE_MEASUREMENT_BUFFER_SIZE - 1)
		{
			//sum buffers
			uint64_t counter [3] = {0};
			
			for (int i = 0; i < DISTANCE_MEASUREMENT_BUFFER_SIZE; i ++)
			{
				counter [0] += distSensorADCValueBuffer [i][LFT_DISTANCE_SENSOR_INDEX];
				counter [1] += distSensorADCValueBuffer [i][RGT_DISTANCE_SENSOR_INDEX];
				counter [2] += distSensorADCValueBuffer [i][FWD_DISTANCE_SENSOR_INDEX];
			}
			
			//avg values
			distSensorAvgADCValues [LFT_DISTANCE_SENSOR_INDEX] = counter [LFT_DISTANCE_SENSOR_INDEX] / DISTANCE_MEASUREMENT_BUFFER_SIZE;
			distSensorAvgADCValues [RGT_DISTANCE_SENSOR_INDEX] = counter [RGT_DISTANCE_SENSOR_INDEX] / DISTANCE_MEASUREMENT_BUFFER_SIZE;
			distSensorAvgADCValues [FWD_DISTANCE_SENSOR_INDEX] = counter [FWD_DISTANCE_SENSOR_INDEX] / DISTANCE_MEASUREMENT_BUFFER_SIZE;
			
			//distance calculations
			distanceValues [0] = convertDistanceADCToCM (distSensorAvgADCValues [LFT_DISTANCE_SENSOR_INDEX]);
			distanceValues [1] = convertDistanceADCToCM (distSensorAvgADCValues [RGT_DISTANCE_SENSOR_INDEX]);
			distanceValues [2] = convertLRDistanceADCToCM (distSensorAvgADCValues [FWD_DISTANCE_SENSOR_INDEX]);
			
			//reset buffer index
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
	uint16_t dist = (uint16_t)((2986.6f / adcDist) - 2.2201f);
	
	if (dist > 30)
		dist = 30;
	if (dist < 3)
		dist = 3;
		
	return dist;
}

uint16_t convertLRDistanceADCToCM (uint16_t adcDist)
{
	uint16_t dist = (uint16_t)((8337.6f / adcDist) - 12.976f);
	
	if (dist > 80)
		dist = 80;
	if (dist < 10)
		dist = 10;
	
	return dist;
}

//positive angles are right turns
//negative angles are left turns
void turnSetAngle (float angle)
{
	if (angle > 0) {
		// turn right
		setTravelDirection (turnRight, 1);
	} else {
		// turn left
		setTravelDirection (turnLeft, 1);
	}
	
	float revolutionPercent = abs (angle) / 360.0f;
	turnDuration = revolutionPercent * COMPLETE_ROTATION_TIME_MS;
	turnStartTime = milliseconds;
	isTurning = true;
	
	//stall the autnomy mode while turning
	stallDuration = turnDuration + 25;
	stallStartTime = turnStartTime;
	stalling = true;
}

//Motors need 70% throttle to start moving and thus the input speed needs to be remapped
void setTravelDirection (enum Directions dir, float speed)
{
	float mappedSpeed = mapf (speed, 1.0f, 0.65f, 1.0f);
	
	//ensures the motors are at full idle if no speed is applied
	if (speed == 0)
		mappedSpeed = 0;
		
	if (dir == forward)
	{
		float motorSpeed [2] = {-1.0f * mappedSpeed, 0};
		setMotorSpeed (motorSpeed);
	}
	
	if (dir == back)
	{
		float motorSpeed [2] = {mappedSpeed, 0};
		setMotorSpeed (motorSpeed);
	}
	
	if (dir == turnLeft)
	{
		float motorSpeed [2] = {0, mappedSpeed};
		setMotorSpeed (motorSpeed);
	}
	
	if (dir == turnRight)
	{
		float motorSpeed [2] = {0, -1.0f * mappedSpeed};
		setMotorSpeed (motorSpeed);
	}
}

void stopTraveling ()
{
	float motorSpeed [2] = {0, 0};
	setMotorSpeed (motorSpeed);
}

void setMotorSpeed (float joyPos[])
{
	/*
		JOY POS TO MOTOR DIR
		
		FWD = -1, 0
		BCK = 1, 0
		LFT = 0, 1
		RGT = 0, -1	
	
	*/
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
