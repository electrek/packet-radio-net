// rf69 demo tx rx.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple addressed, reliable messaging client
// with the RH_RF69 class. RH_RF69 class does not provide for addressing or
// reliability, so you should only use RH_RF69  if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example rf69_server.
// Demonstrates the use of AES encryption, setting the frequency and modem 
// configuration

#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>
#include <SFE_MicroOLED.h>  // Include the SFE_MicroOLED library
#include <Encoder.h>

//////////////////////////
// MicroOLED Definition //
//////////////////////////
#define PIN_RESET 12  // Connect RST to pin 9
#define PIN_DC    11  // Connect DC to pin 8
#define PIN_CS    10 // Connect CS to pin 10
#define DC_JUMPER 0

//////////////////////////////////
// MicroOLED Object Declaration //
//////////////////////////////////
MicroOLED oled(PIN_RESET, PIN_DC, PIN_CS); // SPI declaration

// Encoder definition
#define ENCODERPIN0 5
#define ENCODERPIN1 6
Encoder knob(ENCODERPIN0, ENCODERPIN1);

/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 915.0

// change addresses for each client board, any number :)
#define MY_ADDRESS 0
#define RX1_ADDRESS 1
#define RX2_ADDRESS 2
#define NUM_RX 2

uint8_t rxAddresses[] = {RX1_ADDRESS,RX2_ADDRESS};


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
#define HALLEFFECT A0
float measuredvbat = analogRead(VBATPIN)*2.0*3.3/1024;
float measuredhall = analogRead(HALLEFFECT)*5.0/3.0*3.3/1024;

void setup() 
{
  Serial.begin(115200);
  //while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

  digitalPinToInterrupt(ENCODERPIN0); //on M0, Encoder library doesn't auto set these as interrupts
  digitalPinToInterrupt(ENCODERPIN1);
  
  oled.begin();    // Initialize the OLED
  oled.clear(ALL); // Clear the display's internal memory
  oled.display();  // Display what's in the buffer (splashscreen)
  delay(1000);     // Delay 1000 ms
  oled.clear(PAGE); // Clear the buffer.
  oled.display();
  
  
  pinMode(LED, OUTPUT);     
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  Serial.println("Feather Addressed RFM69 TX Test!");
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
  pinMode(VBATPIN, INPUT);

  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");
}


// Dont put this on the stack:
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
uint8_t data[] = "  OK";
long oldKnobPosition  = -999;
char radiopacket[20] = "Hello World #";

void loop() {
  //delay(1000);  // Wait 1 second between transmits, could also 'sleep' here!
 long newKnobPosition = knob.read();
  if (newKnobPosition != oldKnobPosition) {
    oldKnobPosition = newKnobPosition;
  //  Serial.println(newKnobPosition);
  }
  
  measuredvbat = analogRead(VBATPIN)*2.0*3.3/1024;
 // Serial.print("VBat: " );
	Serial.print(measuredvbat);
	Serial.print(" ");
  measuredhall = analogRead(HALLEFFECT)*5.0/3.0*3.3/1024;
//  Serial.print("HallEffect: " );
	Serial.print(measuredhall);
	Serial.print(" ");
  strcpy(radiopacket,"Hello World #");
  itoa(packetnum++, radiopacket+13, 10);
  
	if (measuredhall < 1.5)
	{
		strcpy(radiopacket,"A");
	}
	if (measuredhall > 3.5)
	{
		strcpy(radiopacket,"B");
	}

  //Serial.print("Sending "); Serial.println(radiopacket);
  
	for (int i=0; i<NUM_RX; i++)
	{
	  // Send a message to the DESTINATION!
		if (rf69_manager.sendtoWait((uint8_t *)radiopacket, strlen(radiopacket), rxAddresses[i]))
		{
			// Now wait for a reply from the server
			uint8_t len = sizeof(buf);
			uint8_t from;   
			if (rf69_manager.recvfromAckTimeout(buf, &len, 2000, &from))
			{
				buf[len] = 0; // zero out remaining string
		  
			 // Serial.print("Got reply from #"); Serial.print(from);
			 // Serial.print(" [RSSI :");
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
		update_page();
	}
}

void Blink(byte PIN, byte DELAY_MS, byte loops) {
  for (byte i=0; i<loops; i++)  {
    digitalWrite(PIN,HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN,LOW);
    delay(DELAY_MS);
  }
}

void update_page()
{
    oled.clear(PAGE);            // Clear the display
    oled.setCursor(0, 0);        // Set cursor to top-left
    oled.setFontType(0);         // Smallest font
    oled.print("VBAT: ");       // Print "VBATT:"
    oled.print(measuredvbat);    // Print VBATT reading
    oled.setCursor(0, 16);       // Set cursor to top-middle-left
    oled.setFontType(0);         // Repeat
    if (lastRSSI==0)
    {
        oled.print("no ack");
    }
    else
    {
        oled.print("RSSI: ");
        oled.print(lastRSSI);
    }
	oled.setCursor(0, 32);
	oled.print("HALL: ");       // Print "HALL:"
    oled.print(measuredhall);    // Print HallEffect reading
    oled.display();
}


