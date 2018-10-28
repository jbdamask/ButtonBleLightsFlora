/*********************************************************************
/*********************************************************
Author: John B Damask & Adafruit folks (via their demos)
Created: July 27, 2018
Purpose: Using button or Bluefruit to control NeoPixels. 
Note: Using RGB neopixels, a button and a Feather Bluefruit 32u4.
      The Feather can both read and write from BLE. The idea is that colors can come in from
      a central device (e.g. a Pi) or you can touch the button and send a color code (state)
      to the device. Note that I've adopted the payload format from Adafruit for sending Colors 
      (as reconstructed from packetParser.cpp). Basically it's !C<red><green><blue><checksum>
Todo: Create classes for ble, touch and pixel functions.

*********************************************************************/
#include <Wire.h>

#include <string.h>
#include <Arduino.h>

/*   #include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif   */

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"
#include <Adafruit_NeoPixel.h>

/*=========================================================================*/
    #define FACTORYRESET_ENABLE     0  
    /* IO pins */
    #define PIXEL_PIN               6
    #define BUTTON_PIN              10
    /* Constants */
    #define NUMPIXELS               12
    #define BRIGHTNESS              30
    #define MIN                     1
    #define MAX                     255
    #define NUMTOUCH                12
    #define DEVICE_NAME             "AT+GAPDEVNAME=TouchLightsBle_upstairs"
/*=========================================================================*/


/* ==========================================================================
                NeoPixel Set to true if using GRBW neopixels                */
    bool neoPixelsWhite = false;  
/*=========================================================================*/

/* ========================================================================== 
                STATE TRACKING                                              */
  int buttonState;
  int lastButtonState = LOW;  
  /* We'll cycle through event states on button pushes */
  uint8_t lastState = 0;
  uint8_t currentState = 1;
  uint8_t minState = 0;
  uint8_t maxState = 11;
  bool isAnimationState = false;   // If true, each loop with call the same function. Useful for animations
/* ========================================================================== */


/* ========================================================================== 
                BUTTON DEBOUNCING                                             */
  unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
  unsigned long debounceDelay = 10;    // the debounce time; increase if the output flickers 
/* ========================================================================== */

uint8_t output = 0;
uint8_t len = 0;
bool printOnceBle = false; // BLE initialization 
Adafruit_NeoPixel pixel;

uint8_t state = 0;

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

/* ========================================================================== 
            BLE PACKET BUFFER AND PAYLOAD                                     */
  extern uint8_t packetbuffer[];
  // the defined length of a color payload
  //int colorLength = 6; // From days of sending RGB colors
  int colorLength = 4;
  uint16_t colLen = 3;
  // Payload stuff
  uint8_t xsum = 0;
  uint8_t PAYLOAD_START = "!";
  uint8_t COLOR_CODE = "C";
  uint8_t BUTTON_CODE = "B";
  uint8_t red;
  uint8_t green;
  uint8_t blue;
  // the ble payload, set to max buffer size
  uint8_t payload[21];
/* ========================================================================== */  

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup()
{
  //while (!Serial);  // Uncomment when connected to computer. Comment out otherwise
  delay(500);
  Serial.begin(115200);
  Serial.println("Setting up");
  // Set Flora Bluefruit LE to CMD mode
  digitalWrite(BLUEFRUIT_UART_MODE_PIN, HIGH);
  pinMode(BUTTON_PIN, INPUT_PULLUP); // Set button and use internal pullup resistor (so you don't need to add a physical one)
  
  if (neoPixelsWhite) {
    pixel = Adafruit_NeoPixel(NUMPIXELS, PIXEL_PIN, NEO_GRBW + NEO_KHZ800);  
  }else{
    pixel = Adafruit_NeoPixel(NUMPIXELS, PIXEL_PIN, NEO_GRB + NEO_KHZ800); 
  }

  // turn off neopixel
  pixel.begin(); // This initializes the NeoPixel library.
  for(uint8_t i=0; i<NUMPIXELS; i++) {
    pixel.setPixelColor(i, pixel.Color(0,0,0)); // off
  }
  pixel.show();

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));
  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  /* Customize Bluefruit name */
  ble.println(DEVICE_NAME);
  /* Disable command echo from Bluefruit */
  ble.echo(false);
  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();
  ble.verbose(false);  // debug info is a little annoying after this point!
  Serial.println("ready to rock");
}


/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  if(ble.isConnected()){
    if(!printOnceBle) {
      Serial.println("ble connected!");
      printOnceBle = true;
    }
    bleSetupOnConnect();
    // Check bluetooth input
    len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  }

  checkForEvent();
  if(isAnimationState){ setLights(); }    

/*
  int reading = digitalRead(BUTTON_PIN);
  if(reading != lastButtonState){
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

   // Check for bluetooth input
 if(len != 0) {
  Serial.println("Bluetooth event detected!");
  //delay(2000);
  bl();
 } else if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:
    // if the button state has changed:
    if(reading != buttonState){
      buttonState = reading;
      if(buttonState == LOW){
        if (currentState < maxState)
        {
          currentState = currentState + 1;
        }else{
          currentState = minState;
        }
        Serial.print("Button pressed! New state is: ");
        Serial.println(currentState);
        packAndSend(); 
      }
    }
  }
  lastButtonState = reading;
  if(isAnimationState){ setLights(); }
  */
  //delay(5);
}


void checkForEvent(){
  int reading = digitalRead(BUTTON_PIN);
  if(reading != lastButtonState){
    // reset the debouncing timer
    lastDebounceTime = millis();
  }
   // Check for bluetooth input
 if(len != 0) {
  Serial.println("Bluetooth event detected!");
  //delay(2000);
  bl();
 } else if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:
    // if the button state has changed:
    if(reading != buttonState){
      buttonState = reading;
      if(buttonState == LOW){
        if (currentState < maxState)
        {
          currentState = currentState + 1;
        }else{
          currentState = minState;
        }
        Serial.print("Button pressed! New state is: ");
        Serial.println(currentState);
        packAndSend(); 
      }
    }
  }
  lastButtonState = reading;
//  if(isAnimationState){ setLights(); }  
}

void setColors(uint8_t r, uint8_t g, uint8_t b){
  red = r; green = g; blue = b;
}

void wipe(){
  if(neoPixelsWhite){
    // My GRBW have twice the density of pixels and my GRB so they wipe faster
    colorWipe(pixel.Color(red, green, blue, 0),5);
  } else {
    colorWipe(pixel.Color(red, green, blue),10);
  }
}

void packAndSend()
{
  Serial.println("Sending data!");
  // Now package into a packetbuffer and write to Bluetooth
  payload[0] = 0x21;
  payload[1] = 0x42;
  payload[2] = currentState;

  xsum = 0;
  //for (uint8_t i=0; i<colLen; i++) {
  for (uint8_t i=0; i<colorLength-1; i++) {
    xsum += payload[i];
  }
  xsum = ~xsum;    
  payload[3] = xsum;  
  for(int i = 0; i < 4; i++){
    Serial.println(payload[i]);
  }
  ble.write(payload,colorLength);
  state = currentState;
  setLights();
}

void setLights(){

    switch (state){
      case 0:
        isAnimationState = false;      
        setColors(255, 192, 203); // Pink        
        wipe();
        break;        
      case 1:
        isAnimationState = false;      
        setColors(0, 128, 128); // Blue green
        wipe();
        break;      
      case 2:
        isAnimationState = false;      
        setColors(255, 0, 0);  // Red
        wipe();
        break;
      case 3:
        isAnimationState = false;      
        setColors(0, 255, 0);  // Green
        wipe();      
        break;
      case 4: 
        isAnimationState = false;      
        setColors(0, 255, 255); // Cyan
        wipe();
        break;
      case 5: 
        isAnimationState = false;      
        setColors(0, 0, 255); // Blue
        wipe();
        break;
      case 6:
        isAnimationState = false;      
        setColors(255, 165, 0); // Orange
        wipe();
        break;
      case 7:
        isAnimationState = false;      
        setColors(128, 0, 128); // Purple
        wipe();
        break;
      case 8:
        isAnimationState = false;      
        setColors(255, 255, 0); // Yellow
        wipe();
        break;
      case 9:
        isAnimationState = false;      
        setColors(192, 214, 228); // Grey / blue
        wipe();
        break;
      case 10:   
        isAnimationState = true; 
        rainbow(50);
        //delay(1000);
        break;
      case 11:   // OFF
        isAnimationState = false;
        setColors(0, 0, 0);
        wipe();
        break;
      default:
        isAnimationState = false;      
        setColors(0, 0, 0);
        wipe();
       break;  
    }

}

// Lights triggered by bluetooth
void bl(){
  /* Got a packet! */
//  printHex(packetbuffer, len);
  state = packetbuffer[2];
  setLights();
}


void bleSetupOnConnect(){
  if(ble.getMode() != 0) {
    Serial.println(F("***********************"));
    // Set Bluefruit to DATA mode
     Serial.println( F("Switching to DATA mode!") );
    ble.setMode(BLUEFRUIT_MODE_DATA);
    Serial.println(F("***********************"));
  }
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<pixel.numPixels(); i++) { 
    pixel.setPixelColor(i, c);
    pixel.setBrightness(BRIGHTNESS);
    pixel.show();
    delay(wait);
  }
}

void rainbow(uint8_t wait) {
  uint16_t i, j;
  Serial.println("Rainbow...");
  for(j=0; j<256; j++) {
    for(i=0; i<pixel.numPixels(); i++) {
      pixel.setPixelColor(i, Wheel((i+j) & 255));
      checkForEvent();
    }
    pixel.show();
    delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< pixel.numPixels(); i++) {
      pixel.setPixelColor(i, Wheel(((i * 256 / pixel.numPixels()) + j) & 255));
    }
    pixel.show();
    delay(wait);
  }
}

//Theatre-style crawling lights.
void theaterChase(uint32_t c, uint8_t wait) {
  for (int j=0; j<10; j++) {  //do 10 cycles of chasing
    for (int q=0; q < 3; q++) {
      for (uint16_t i=0; i < pixel.numPixels(); i=i+3) {
        pixel.setPixelColor(i+q, c);    //turn every third pixel on
      }
      pixel.show();

      delay(wait);

      for (uint16_t i=0; i < pixel.numPixels(); i=i+3) {
        pixel.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}

//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait) {
  for (int j=0; j < 256; j++) {     // cycle all 256 colors in the wheel
    for (int q=0; q < 3; q++) {
      for (uint16_t i=0; i < pixel.numPixels(); i=i+3) {
        pixel.setPixelColor(i+q, Wheel( (i+j) % 255));    //turn every third pixel on
      }
      pixel.show();

      delay(wait);

      for (uint16_t i=0; i < pixel.numPixels(); i=i+3) {
        pixel.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return pixel.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return pixel.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return pixel.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
