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

void Blink(byte PIN, byte DELAY_MS, byte loops);
void secondTick();
void sendStatus();
void checkMsg();
void sendMsg(char* radiopacket, uint8_t address);

// Actions
#define IDLE 0
#define SEND_SIGNAL 1
// States
#define QUEEN_NOMATCH_KING_NOMATCH 0
#define QUEEN_MATCH_KING_NOMATCH 1
#define QUEEN_NOMATCH_KING_MATCH 2
#define QUEEN_MATCH_KING_MATCH 3

uint8_t action = IDLE;
uint8_t state = QUEEN_NOMATCH_KING_NOMATCH;

const char* actionStr[8] = {"IDLE", "SEND_SIGNAL"};
const char* stateStr[8] = {"QUEEN_NOMATCH_KING_NOMATCH", "QUEEN_MATCH_KING_NOMATCH", "QUEEN_NOMATCH_KING_MATCH", "QUEEN_MATCH_KING_MATCH"};

/************ Radio Setup ***************/
// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 915.0

// who am i? (server address)
#define MY_ADDRESS     2

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
int16_t lastRSSI = 0;
#define VBATPIN A7
#define HALLEFFECT1 A0
#define HALLEFFECT2 A1
float measuredvbat = analogRead(VBATPIN)*2.0*3.3/1024;
float measuredhall1 = analogRead(HALLEFFECT1)*5.0/3.0*3.3/1024;
float measuredhall2 = analogRead(HALLEFFECT2)*5.0/3.0*3.3/1024;

void setup() 
{
	Serial.begin(115200);
	//while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

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
char radiopacket[20] = "Hello World #";

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
				case 'A' :
					Serial.println("case A, Send status");
					//do something for command "A"
					sendStatus();
					break;
				case 'B' :
					Serial.println("case B, Send signal");
					//do something for command "B"
					 radiopacket[0] = 'C';
					 sendMsg(radiopacket,3);  //"Open" command is 'C', desk drawer address is 3
					break;
				case 'C' :
					Serial.println("case C, RESET");
					//do something for command "C"
					break;
				default:
					Serial.println("default");
					// do the default
					break;
			}
			len = strlen(actionStr[action]);
			Serial.print("len = ");
			Serial.println(len);
			memcpy(data, actionStr[action], len);
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
		measuredvbat = analogRead(VBATPIN)*2.0*3.3/1024;
		Serial.print("VBat: " );
		Serial.println(measuredvbat);
		measuredhall1 = analogRead(HALLEFFECT1)*5.0/3.0*3.3/1024;
		Serial.print("VHall1: " );
		Serial.println(measuredhall1);
		measuredhall2 = analogRead(HALLEFFECT2)*5.0/3.0*3.3/1024;
		Serial.print("VHall2: " );
		Serial.println(measuredhall2);
		ul_PreviousMillis = ul_CurrentMillis;
	}
}


void sendStatus()
{
	
}

void checkMsg()
{
	
}

void sendMsg(char* radiopacket, uint8_t address)
{
  // Send a message to the DESTINATION!
  if (rf69_manager.sendtoWait((uint8_t *)radiopacket, strlen(radiopacket), address))
  {
    // Now wait for a reply from the server
    uint8_t len = sizeof(buf);
    uint8_t from;   
    if (rf69_manager.recvfromAckTimeout(buf, &len, 2000, &from))
    {
      buf[len] = 0; // zero out remaining string
    
      Serial.print("Got reply from #"); Serial.println(from);
      Serial.print("Reply:"); Serial.println((char*)buf);
      Serial.print(" [RSSI :");
      lastRSSI = rf69.lastRssi();
      Serial.println(lastRSSI);
      
    // Serial.print("] : ");
    // Serial.println((char*)buf);     
      Blink(LED, 40, 3); //blink LED 3 times, 40ms between blinks
    }
    else
    {
      Serial.println("No reply, is anyone listening?");
    }
  }
  else
  {
    Serial.println("Sending failed (no ack)");
    lastRSSI = 0;
  }
}