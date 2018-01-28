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
#include <SD.h>
#include <Adafruit_VS1053.h>

void Blink(byte PIN, byte DELAY_MS, byte loops);
void checkbatt();
void printDirectory(File dir, int numTabs);
void checkRX(uint8_t* buf);
void doCommand(uint8_t* buf);

// VS1053 setup (audio player)
#define VS1053_RESET   -1     // VS1053 reset pin (not used!)


  #define VS1053_CS       6     // VS1053 chip select pin (output)
  #define VS1053_DCS     10     // VS1053 Data/command select pin (output)
  #define CARDCS          5     // Card chip select pin
  // DREQ should be an Int pin *if possible* (not possible on 32u4)
  #define VS1053_DREQ     11     // VS1053 Data request, ideally an Interrupt pin


Adafruit_VS1053_FilePlayer musicPlayer = Adafruit_VS1053_FilePlayer(VS1053_RESET, VS1053_CS, VS1053_DCS, VS1053_DREQ, CARDCS);

/************ Radio Setup ***************/
// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 915.0

// who am i? (server address)
#define MY_ADDRESS     1


  #define RFM69_CS      8
  #define RFM69_INT     3
  #define RFM69_RST     4
  #define LED           13

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram rf69_manager(rf69, MY_ADDRESS);

int16_t packetnum = 0;  // packet counter, we increment per xmission
#define VBATPIN A7
float measuredvbat = analogRead(VBATPIN)*2.0*3.3/1024;

uint8_t track = 0;
const char* trackStr[] = {"track001.mp3", "track002.mp3", "track003.mp3", "track004.mp3", "track005.mp3", "track006.mp3", "track007.mp3", "track008.mp3"};
void setup() 
{
    pinMode(RFM69_CS, INPUT_PULLUP);  // disable radio for now...
	Serial.begin(115200);
	//while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

    if (! musicPlayer.begin()) // initialise the music player
	{ 
     Serial.println(F("Couldn't find VS1053, do you have the right pins defined?"));
     while (1);
  	}
	Serial.println(F("VS1053 found"));
	musicPlayer.sineTest(0x44, 500);    // Make a tone to indicate VS1053 is working
  
  if (!SD.begin(CARDCS)) {
    Serial.println(F("SD failed, or not present"));
   // while (1);  // don't do anything more
  }
  Serial.println("SD OK!");
  
  // list files
  printDirectory(SD.open("/"), 0);
  
  // Set volume for left, right channels. lower numbers == louder volume!
  musicPlayer.setVolume(0,0);
  

  // If DREQ is on an interrupt pin we can do background
  // audio playing
  musicPlayer.useInterrupt(VS1053_FILEPLAYER_PIN_INT);  // DREQ int
  
  // Play a file in the background, REQUIRES interrupts!
  Serial.println(F("Playing full track 001"));
  musicPlayer.startPlayingFile("track001.mp3");

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
	checkbatt();
	if (musicPlayer.playingMusic == false)
	{
		track = (track+1) % 8;
		Serial.print(F("Playing track "));
		Serial.println(track);
  		musicPlayer.startPlayingFile(trackStr[track]);
	}
	checkRX(buf);
			// Check to see what command was sent
			switch(buf[0])
			{
				case 'A' :
					Serial.println("case A");
					//do something for command "A"

					break;
				case 'B' :
					Serial.println("case B");
					//do something for command "B"

					break;
				case 'C' :
					//do something for command "C"
					break;
				default:
					Serial.println("default");
					// do the default
					break;
			}
}

void checkRX(uint8_t* buf)
{
	if (rf69_manager.available())
  	{
		// Wait for a message addressed to us from the client
    	uint8_t len = sizeof(buf);
    	uint8_t from;
		if (rf69_manager.recvfromAck(buf, &len, &from))
		{
			buf[len] = 0; // zero out remaining string
      
			Serial.print("Got packet from #"); Serial.print(from);
			Serial.print(" [RSSI :");
			Serial.print(rf69.lastRssi());
			Serial.print("] : ");
			Serial.println((char*)buf);
			doCommand(buf);
			Blink(LED, 40, 3); //blink LED 3 times, 40ms between blinks

			// Send a reply back to the originator client
			if (!rf69_manager.sendtoWait(data, sizeof(data), from))
				Serial.println("Sending failed (no ack)");
		}
	}
}

void doCommand(uint8_t* buf)
{
	// Check to see what command was sent
			switch(buf[0])
			{
				case 'A' :
					Serial.println("case A");
					//do something for command "A"

					break;
				case 'B' :
					Serial.println("case B");
					//do something for command "B"

					break;
				case 'C' :
					//do something for command "C"
					break;
				default:
					Serial.println("default");
					// do the default
					break;
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

void checkbatt()
{
	unsigned long ul_CurrentMillis = millis();
	if ((ul_CurrentMillis - ul_PreviousMillis > ul_1secIntervalMillis))
	{
		measuredvbat = analogRead(VBATPIN)*2.0*3.3/1024;
		Serial.print("VBat: " );
		Serial.println(measuredvbat);
	ul_PreviousMillis = ul_CurrentMillis;
	}
}

/// File listing helper
void printDirectory(File dir, int numTabs)
{
   while(true)
   {
     
     File entry =  dir.openNextFile();
     if (! entry) {
       // no more files
       //Serial.println("**nomorefiles**");
       break;
     }
     for (uint8_t i=0; i<numTabs; i++) {
       Serial.print('\t');
     }
     Serial.print(entry.name());
     if (entry.isDirectory()) {
       Serial.println("/");
       printDirectory(entry, numTabs+1);
     } else {
       // files have sizes, directories do not
       Serial.print("\t\t");
       Serial.println(entry.size(), DEC);
     }
     entry.close();
   }
}
