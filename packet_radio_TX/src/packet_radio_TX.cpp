// rf69 demo tx rx.pde
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
#include <Encoder.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Adafruit_Trellis.h"

void update_page();
void Blink(byte PIN, byte DELAY_MS, byte loops);
void sendMsg(char* radiopacket, uint8_t address);
void checkMsg();
void int0();
void int1();

/********* Encoder Setup ***************/
#define PIN_ENCODER_SWITCH 11

//digitalPinToInterrupt(10); //on M0, Encoder library doesn't auto set these as interrupts
//digitalPinToInterrupt(12);

//Encoder knob(10, 12);
uint8_t activeRow = 0;
long pos = -999;
long newpos;
int prevButtonState = HIGH;
bool needsRefresh = true;
bool advanced = false;
unsigned long startTime;

#define MENU_LENGTH 8

/********* Trellis Setup ***************/
#define MOMENTARY 0
#define LATCHING 1
#define MODE LATCHING //all Trellis buttons in latching mode
Adafruit_Trellis matrix0 = Adafruit_Trellis();
Adafruit_TrellisSet trellis =  Adafruit_TrellisSet(&matrix0);
#define NUMTRELLIS 1
#define numKeys (NUMTRELLIS * 16)
#define INTPIN A2

/************ OLED Setup ***************/
Adafruit_SSD1306 oled = Adafruit_SSD1306();
#define BUTTON_A 9
#define BUTTON_B 6
#define BUTTON_C 5
#define LED      13

/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 915.0

// change addresses for each client board, any number :)
#define MY_ADDRESS 1
#define Radio_ADDRESS 1
#define Chessboard_ADDRESS 2
#define DeskDrawer_ADDRESS 3
#define Lights_ADDRESS 4
#define RX5_ADDRESS 5
#define RX6_ADDRESS 6
#define RX7_ADDRESS 7
#define RX8_ADDRESS 8
#define NUM_RX 8

uint8_t rxAddresses[] = {Radio_ADDRESS,Chessboard_ADDRESS,DeskDrawer_ADDRESS,Lights_ADDRESS,RX5_ADDRESS,RX6_ADDRESS,RX7_ADDRESS,RX8_ADDRESS};

#if defined(ARDUINO_SAMD_FEATHER_M0) // Feather M0 w/Radio
  #define RFM69_CS      8
  #define RFM69_INT     3
  #define RFM69_RST     4
  #define LED           13
#endif

// Set up variables for rotary decoder
volatile unsigned long threshold = 00000;
// 'rotaryHalfSteps' is the counter of half-steps. The actual
// number of steps will be equal to rotaryHalfSteps / 2
//
volatile long rotaryHalfSteps = 0;

// Working variables for the interrupt routines
//
volatile unsigned long int0time = 0;
volatile unsigned long int1time = 0;
volatile boolean int0signal = false;
volatile boolean int1signal = false;
volatile boolean int0history = false;
volatile boolean int1history = false;


// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram rf69_manager(rf69, MY_ADDRESS);

int lastButton=17; //last button pressed for Trellis logic

int menuList[8]={1,2,3,4,5,6,7,8}; //for rotary encoder choices
int m = 0; //variable to increment through menu list
int lastTB[8] = {16, 16, 16, 16, 16, 16, 16, 16}; //array to store per-menu Trellis button
const char* menuListStr[8] = {"Radio", "Chessboard", "DeskDrawer", "Lights", "Other1", "Other2", "Other3", "Other4"};  // menu options align with different RX's (and RX addresses)
const char* menuSubListStr[8][16] =  {{"ON", "OFF", "FORWARD", "REVERSE", "PAUSE", "", "", "", "", "", "", "", "", "", "", ""},
                                      {"Queen Activate", "Knight Activate", "RESET", "", "", "", "", "", "", "", "", "", "", "", "", ""},
                                      {"OPEN", "CLOSE", "STOP", "", "", "", "", "", "", "", "", "", "", "", "", ""},
                                      {"ON", "OFF", "", "", "", "", "", "", "", "", "", "", "", "", "", ""},
                                      {"Action1", "Action2", "Action3", "", "", "", "", "", "", "", "", "", "", "", "", ""},
                                      {"Action1", "Action2", "Action3", "", "", "", "", "", "", "", "", "", "", "", "", ""},
                                      {"Action1", "Action2", "Action3", "", "", "", "", "", "", "", "", "", "", "", "", ""},
                                      {"Action1", "Action2", "Action3", "", "", "", "", "", "", "", "", "", "", "", "", ""}};
const char menuCmdStr[8][16] = {{'A', 'B', 'C', 'D', 'E', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z'},
                                {'F', 'G', 'H', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z'},
                                {'I', 'J', 'K', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z'},
                                {'K', 'L', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z'},
                                {'M', 'N', 'O', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z'},
                                {'P', 'Q', 'R', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z'},
                                {'S', 'T', 'U', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z'},
                                {'V', 'W', 'X', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z', 'z'}};
int16_t packetnum = 0;  // packet counter, we increment per xmission
int16_t lastRSSI = 0;

#define VBATPIN A7
#define HALLEFFECT A0
float measuredvbat = analogRead(VBATPIN)*2.0*3.3/1024;
// float measuredhall = analogRead(HALLEFFECT)*5.0/3.0*3.3/1024;

void setup() 
{
  Serial.begin(115200);
 // while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer
 
  // INT pin on Trellis requires a pullup
 pinMode(INTPIN, INPUT);
 digitalWrite(INTPIN, HIGH);
 trellis.begin(0x70);  

 pinMode(PIN_ENCODER_SWITCH, INPUT_PULLUP);//set encoder push switch pin to input pullup
 
  // Rotary decoder pin setup
  pinMode(10, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);

  attachInterrupt(10, int0, CHANGE);
  attachInterrupt(12, int1, CHANGE);

 // Initialize OLED display
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  oled.setTextWrap(false);
  oled.display();
  delay(500);
  oled.clearDisplay();
  oled.display();
  oled.setTextSize(2);
  oled.setTextColor(WHITE);
  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP); 
  pinMode(LED, OUTPUT);     
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

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

  oled.setCursor(0,0);
  oled.println("RFM69 @ ");
  oled.print((int)RF69_FREQ);
  oled.println(" MHz");
  oled.display();
  delay(1200); //pause to let freq message be read by a human

  oled.clearDisplay();
  oled.setCursor(0,0);
  oled.println("REMOTE FX");
  oled.setCursor(0,16);
  oled.println("TRIGGER");  
  oled.display();

    // light up all the LEDs in order
    for (uint8_t i=0; i<numKeys; i++) {
      trellis.setLED(i);
      trellis.writeDisplay();    
      delay(30);
    }
    // then turn them off
    for (uint8_t i=0; i<numKeys; i++) {
      trellis.clrLED(i);
      trellis.writeDisplay();    
      delay(30);
    }
}


// Dont put this on the stack:
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
uint8_t data[] = "  OK";
char radiopacket[20] = "Hello World #";

void loop()
{
  delay(30); // 30ms delay is required, dont remove me! (Trellis)
      /*************Rotary Encoder Menu***********/

    //check the encoder knob, set the current position as origin
    //long newPos = knob.read() / 4;//divide for encoder detents --> this is used with Encoder lib

    long newPos = rotaryHalfSteps / 2;
    #ifdef DEBUG
      Serial.print("pos=");
      Serial.print(pos);
      Serial.print(", newPos=");
      Serial.println(newPos);
    #endif

    if(newPos != pos)
    {
      int diff = newPos - pos;//check the different between old and new position
      if(diff>=1)
      {
        m++; 
        m = (m+MENU_LENGTH) % MENU_LENGTH;//modulo to roll over the m variable through the list size
      }

      if(diff==-1)
      { //rotating backwards
         m--;
         m = (m+MENU_LENGTH) % MENU_LENGTH;
      }
      #ifdef DEBUG
      Serial.print("Diff = ");
      Serial.print(diff);
      Serial.print("  pos= ");
      Serial.print(pos);
      Serial.print(", newPos=");
      Serial.println(newPos);
      Serial.println(menuList[m]);
      Serial.print("m is: ");
      Serial.println(m);
      #endif

      pos = newPos;

      //clear Trellis lights 
      for(int t=0;t<=16;t++)
      {
        trellis.clrLED(t);
        trellis.writeDisplay();
      }
      //light last saved light for current menu
        trellis.clrLED(lastTB[m]);
        trellis.setLED(lastTB[m]);
        trellis.writeDisplay();
      
      //write to the display
      oled.clearDisplay();
      oled.fillRect(0, (m)*4, 4, 4, WHITE);
      //oled.circleFill(3,(m+1)*5,2);
      oled.setCursor(6,2);
      oled.print(menuListStr[m]);
      oled.display();
    }

  measuredvbat = analogRead(VBATPIN)*2.0*3.3/1024;
//  Serial.print("VBat: " );
//	Serial.println(measuredvbat);
//	Serial.print(" ");
// measuredhall = analogRead(HALLEFFECT)*5.0/3.0*3.3/1024;
//  Serial.print("HallEffect: " );
//	Serial.print(measuredhall);
//	Serial.print(" ");

// remember that the switch is active low
  int buttonState = digitalRead(PIN_ENCODER_SWITCH);
  if (buttonState == LOW)
  {
      unsigned long now = millis();
      if (prevButtonState == HIGH)
      {
          prevButtonState = buttonState;
          startTime = now;
        // Serial.println("button pressed");
          trellis.clrLED(lastTB[m]);
          trellis.writeDisplay();    
          lastTB[m]=17;//set this above the physical range so 
          //next button press works properly
      } 
  } 
  else if (buttonState == HIGH && prevButtonState == LOW)
  {
    //Serial.println("button released!");
    prevButtonState = buttonState;
  }

/*************Trellis Button Presses***********/
  if (MODE == MOMENTARY)
  {
    if (trellis.readSwitches())
    { // If a button was just pressed or released...
      for (uint8_t ikeys=0; ikeys<numKeys; ikeys++)
      { // go through every button
        if (trellis.justPressed(ikeys)) 
        { // if it was pressed, turn it on
        //Serial.print("v"); Serial.println(i);
          trellis.setLED(ikeys);
        } 
        if (trellis.justReleased(ikeys))
        { // if it was released, turn it off
          //Serial.print("^"); Serial.println(i);
          trellis.clrLED(ikeys);
        }
      }
      trellis.writeDisplay(); // tell the trellis to set the LEDs we requested
    }
  }

  if (MODE == LATCHING)
  {
    if (trellis.readSwitches())
    { // If a button was just pressed or released...
      for (uint8_t ikeys=0; ikeys<numKeys; ikeys++)
      { // go through every button
        if (trellis.justPressed(ikeys))
        { // if it was pressed...
        //Serial.print("v"); Serial.println(ikeys);

        // Alternate the LED unless the same button is pressed again
        //if(ikeys!=lastButton){
          if(ikeys!=lastTB[m])
          { 
            if (trellis.isLED(ikeys))
            {
                trellis.clrLED(ikeys);
                lastTB[m]=ikeys; //set the stored value for menu changes
            }
            else
            {
              trellis.setLED(ikeys);
              //trellis.clrLED(lastButton);//turn off last one
              trellis.clrLED(lastTB[m]);
              lastTB[m]=ikeys; //set the stored value for menu changes
            }
            trellis.writeDisplay();
          }

         checkMsg();



              char radiopacket[20];
              

          //check the rotary encoder menu choice
      
          radiopacket[0] = menuCmdStr[m][ikeys];
          oled.clearDisplay();
          oled.setCursor(0,0);
          oled.print(menuListStr[m]);
          oled.setCursor(0,16);
          oled.print(menuSubListStr[m][ikeys]);
          oled.display();  

          Serial.print("Sending "); 
          Serial.println(radiopacket[0]);
          Serial.print("to ");
          Serial.println(rxAddresses[m]);

      //  rf69.send((uint8_t *)radiopacket, strlen(radiopacket));
      //  rf69.waitPacketSent(); 
          //reset packet so unassigned buttons don't send last command
      //    radiopacket[0]='z'; //also being used to turn off NeoPixels 
          //from any unused button

      sendMsg(radiopacket,rxAddresses[m]);

      //		update_page();
           
        }
    }
      // tell the trellis to set the LEDs we requested
      trellis.writeDisplay();
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

void update_page()
{
    oled.clearDisplay();            // Clear the display
    oled.setCursor(0, 0);        // Set cursor to top-left
    //oled.setFont(0);         // Smallest font
    oled.print("VBAT: ");       // Print "VBATT:"
    oled.print(measuredvbat);    // Print VBATT reading
    oled.setCursor(0, 16);       // Set cursor to top-middle-left
   // oled.setFont(0);         // Repeat
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
//	oled.print("HALL: ");       // Print "HALL:"
//    oled.print(measuredhall);    // Print HallEffect reading
    oled.display();
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

void checkMsg()
{
  if (rf69_manager.available())
  {
   // Wait for a message addressed to us from the client
   uint8_t len = sizeof(buf);
   uint8_t from;
   if (rf69_manager.recvfromAck(buf, &len, &from))
   {
     buf[len] = 0;
     Serial.print("got request from : 0x");
     Serial.print(from, HEX);
     Serial.print(": ");
     Serial.println((char*)buf);
     Serial.print("RSSI: "); Serial.println(rf69.lastRssi(), DEC);

     // echo last button       
     data[0] = 'z';
     // Send a reply back to the originator client
     if (!rf69_manager.sendtoWait(data, sizeof(data), from))
       Serial.println("sendtoWait failed");
   }        
 }
}

// Interrupt routines for rotary decoder
void int0()
{
  if ( micros() - int0time < threshold )
    return;
  int0history = int0signal;
  int0signal = REG_PORT_IN0 & PORT_PA18;
  if ( int0history == int0signal )
    return;
  int0time = micros();
  if ( int0signal == int1signal )
    rotaryHalfSteps++;
  else
    rotaryHalfSteps--;
}

void int1()
{
  if ( micros() - int1time < threshold )
    return;
  int1history = int1signal;
  int1signal = (REG_PORT_IN0 & PORT_PA19);
  if ( int1history == int1signal )
    return;
  int1time = micros();
}