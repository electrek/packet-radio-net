// RX with motor controller (RX1)

// -*- mode: C++ -*-
// Example sketch showing how to create a simple addressed, reliable messaging client
// with the RH_RF69 class. RH_RF69 class does not provide for addressing or
// reliability, so you should only use RH_RF69  if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example rf69_server.
// Demonstrates the use of AES encryption, setting the frequency and modem 
// configuration

#include <Arduino.h>
#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

void Blink(byte PIN, byte DELAY_MS, byte loops);
void secondTick();
void runActuatorCheck();
void sendMsg();
void checkMsg();

// Actions
#define IDLE 0
#define CLOSING 1
#define OPENING 2
#define STOPPING 3
// States
#define UNKNOWN 0
#define CLOSED 1
#define OPEN 2

uint8_t actuatorAction = IDLE;
uint8_t actuatorCount = 0;
uint8_t actuatorState = UNKNOWN;

const char* actuatorActionStr[8] = {"IDLE", "CLOSING", "OPENING", "STOPPING"};
const char* actuatorStateStr[8] = {"UNKNOWN", "CLOSED", "OPEN"};

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

// Select which 'port' M1, M2, M3 or M4. In this case, M1
 Adafruit_DCMotor *myMotor = AFMS.getMotor(1);
// You can also make another motor on port M2
//Adafruit_DCMotor *myOtherMotor = AFMS.getMotor(2);

/************ Radio Setup ***************/
// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 915.0

// who am i? (server address)
#define MY_ADDRESS     3

#if defined(ARDUINO_SAMD_FEATHER_M0) // Feather M0 w/Radio
  #define RFM69_CS      8
  #define RFM69_INT     3
  #define RFM69_RST     4
  #define LED           13
#endif

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram rf69_manager(rf69, MY_ADDRESS);

int16_t packetnum = 0;  // packet counter, we increment per xmission
#define VBATPIN A7
float measuredvbat = analogRead(VBATPIN)*2.0*3.3/1024;

void setup() 
{
	Serial.begin(115200);
	//while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

	AFMS.begin();  // create with the default frequency 1.6KHz
	//AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  
  // Set the speed to start, from 0 (off) to 255 (max speed)
	myMotor->setSpeed(255);
	
//	myMotor->run(BACKWARD);  //open drawer
//	Serial.println("Open drawer");
//	delay(8000);
//	myMotor->run(RELEASE);
//	myMotor->run(FORWARD);	//close drawer
//	Serial.println("Close drawer");
//	delay(8000);
//	myMotor->run(RELEASE);

  
	pinMode(LED, OUTPUT);     
	pinMode(RFM69_RST, OUTPUT);
	digitalWrite(RFM69_RST, LOW);

	Serial.println("Feather Addressed RFM69 RX Test!");
	Serial.println();

  // manual reset
	digitalWrite(RFM69_RST, HIGH);
	delay(10);
	digitalWrite(RFM69_RST, LOW);
	delay(10);
  
	if (!rf69_manager.init()) {
		Serial.println("RFM69 radio init failed");
		while (1);
	}
	Serial.println("RFM69 radio init OK!");
	// Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
	// No encryption
	if (!rf69.setFrequency(RF69_FREQ)) {
		Serial.println("setFrequency failed");
	}

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0xAB, 0xCD, 0xEF, 0x01, 0x23, 0x45, 0x67, 0x89,
                    0xFE, 0xDC, 0xBA, 0x98, 0x76, 0x54, 0x32, 0x10};
  rf69.setEncryptionKey(key);
  
  pinMode(LED, OUTPUT);

  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");
}

unsigned long ul_1secIntervalMillis = 1000UL;  //1 sec interval
unsigned long ul_PreviousMillis = millis();

// Dont put this on the stack:
uint8_t data[] = "And hello back to you";
// Dont put this on the stack:
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];

void loop()
{
	secondTick();
	
	if (rf69_manager.available())
  	{
		// Wait for a message addressed to us from the client
    	uint8_t len = sizeof(buf);
    	uint8_t from;
		if (rf69_manager.recvfromAck(buf, &len, &from))
		{
			buf[len] = 0; // zero out remaining string
      
			Serial.print("Got packet from #"); Serial.print(from);
			Serial.print(" [RSSI : ");
			Serial.print(rf69.lastRssi());
			Serial.print("] : ");
			Serial.println(buf[0]);
			Blink(LED, 40, 3); //blink LED 3 times, 40ms between blinks

		// Check to see what command was sent
			switch(buf[0])
			{
				case 'I' :
					Serial.println("case I, Open drawer");
					//do something for command "I"
					myMotor->run(BACKWARD);  //open drawer
					actuatorAction = OPENING;
					actuatorState = UNKNOWN;
					actuatorCount = 10;
					break;
				case 'J' :
					Serial.println("case J, Close drawer");
					//do something for command "J"
					myMotor->run(FORWARD);   //close drawer
					actuatorAction = CLOSING;
					actuatorState = UNKNOWN;
					actuatorCount = 10;
					break;
				case 'K' :
					Serial.println("case K, STOP");
					//do something for command "K"
					myMotor->run(RELEASE);   //STOP
					actuatorAction = STOPPING;
					actuatorState = UNKNOWN;
					actuatorCount = 0;
					//do something for command "K"
					break;
				default:
					Serial.println("default");
					// do the default
					break;
			}
			len = strlen(actuatorActionStr[actuatorAction]);
			Serial.print("len = ");
			Serial.println(len);
			memcpy(data, actuatorActionStr[actuatorAction], len);
			data[len] = 0;
			// Send a reply back to the originator client
			if (!rf69_manager.sendtoWait(data, len, from))
				Serial.println("Sending failed (no ack)");
		}
	}
}


void Blink(byte PIN, byte DELAY_MS, byte loops)
{
	for (byte i=0; i<loops; i++)
  	{
		digitalWrite(PIN,HIGH);
		delay(DELAY_MS);
		digitalWrite(PIN,LOW);
		delay(DELAY_MS);
  	}
}

void secondTick()
{
	unsigned long ul_CurrentMillis = millis();
	if ((ul_CurrentMillis - ul_PreviousMillis >= ul_1secIntervalMillis))
	{
		runActuatorCheck();
		measuredvbat = analogRead(VBATPIN)*2.0*3.3/1024;
		Serial.print("VBat: " );
		Serial.println(measuredvbat);
		ul_PreviousMillis = ul_CurrentMillis;
	}
}

void runActuatorCheck()
{
	Serial.print("Actuator State is ");
	Serial.println(actuatorStateStr[actuatorState]);
	Serial.print("Actuator Action is ");
	Serial.println(actuatorActionStr[actuatorAction]);
	if (actuatorAction != IDLE)  // actuator is either OPENING or CLOSING
	{
		actuatorCount--;
		if (actuatorCount==0)
		{
			if (actuatorAction==CLOSING)  // if closing completed, actuator state is closed
				{
					actuatorState = CLOSED;
				}
				if (actuatorAction==OPENING)  // if closing completed, actuator state is closed
				{
					actuatorState = OPEN;
				}
			actuatorAction = IDLE;
			myMotor->run(RELEASE);
		}
	}
}

void sendMsg()
{

}

void checkMsg()
{
	
}