/*********************************************************************
 * This is an example for our nRF8001 Bluetooth Low Energy Breakout
 * 
 * Pick one up today in the adafruit shop!
 * ------> http://www.adafruit.com/products/1697
 * 
 * Adafruit invests time and resources providing this open source code, 
 * please support Adafruit and open-source hardware by purchasing 
 * products from Adafruit!
 * 
 * Written by Kevin Townsend/KTOWN  for Adafruit Industries.
 * MIT license, check LICENSE for more information
 * All text above, and the splash screen below must be included in any redistribution
 *********************************************************************/

// This version uses the internal data queing so you can treat it like Serial (kinda)!

#include <SPI.h>
#include "Adafruit_BLE_UART.h"
#include <CapacitiveSensor.h>
#include <stdlib.h>
#include <stdio.h>

// Connect CLK/MISO/MOSI to hardware SPI
// e.g. On UNO & compatible: CLK = 13, MISO = 12, MOSI = 11
#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 2     // This should be an interrupt pin, on Uno thats #2 or #3
#define ADAFRUITBLE_RST 9

Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);
/**************************************************************************/
/*!
 Configure the Arduino and start advertising with the radio
 */
/**************************************************************************/
CapacitiveSensor   cs_0_1 = CapacitiveSensor(0,1);        // 10M resistor between pins 0 & 2, pin 2 is sensor pin, add a wire and or foil if desired
CapacitiveSensor   cs_0_6 = CapacitiveSensor(0,6);        // 10M resistor between pins 0 & 2, pin 2 is sensor pin, add a wire and or foil if desired
//CapacitiveSensor   cs_1_0 = CapacitiveSensor(1,0);        // 10M resistor between pins 0 & 2, pin 2 is sensor pin, add a wire and or foil if desired

void setup(void)
{ 
  cs_0_1.set_CS_AutocaL_Millis(0xFFFFFFFF);     // turn off autocalibrate on channel 1 - just as an example

  Serial.begin(9600);
  while(!Serial); // Leonardo/Micro should wait for serial init
  Serial.println(F("Adafruit Bluefruit Low Energy nRF8001 Print echo demo"));

  BTLEserial.begin();
}

/**************************************************************************/
/*!
 Constantly checks for new events on the nRF8001
 */
/**************************************************************************/
aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;

void loop()
{
  long start = millis();
  long total1 =  cs_0_1.capacitiveSensor(30);
  long total2 =  cs_0_6.capacitiveSensor(30);

  // Tell the nRF8001 to do whatever it should be working on.
  BTLEserial.pollACI();

  // Ask what is our current status
  aci_evt_opcode_t status = BTLEserial.getState();
  // If the status changed....
  if (status != laststatus) {
    // print it out!
    if (status == ACI_EVT_DEVICE_STARTED) {
      Serial.println(F("* Advertising started"));
    }
    if (status == ACI_EVT_CONNECTED) {
      Serial.println(F("* Connected!"));
    }
    if (status == ACI_EVT_DISCONNECTED) {
      Serial.println(F("* Disconnected or advertising timed out"));
    }
    // OK set the last status change to this one
    laststatus = status;
  }

  if (status == ACI_EVT_CONNECTED) {

    Serial.print(total1);                // print sensor output
    Serial.print('\t');
    Serial.println(total2);
    //String tau1 = String(total1);
    //String tau2 = String(total2);

    // We need to convert the line to bytes, no more than 20 at this time
    //uint8_t sendbuffer1[20];
    //uint8_t sendbuffer2[20];
    //tau1.getBytes(sendbuffer1, 20);
    //tau2.getBytes(sendbuffer2, 20);
    //char sendbuffersize1 = min(20, tau1.length());
    //char sendbuffersize2 = min(20, tau2.length());

    // write the data
    //BTLEserial.write(sendbuffer1, sendbuffersize1);
    //BTLEserial.write(sendbuffer2, sendbuffersize2);
    
    if (total1>2000){
      int value = 1;
      String order = String(value);
      uint8_t sendbuffer[20];
      order.getBytes(sendbuffer, 20);
      char sendbuffersize = min(20, order.length());
      BTLEserial.write(sendbuffer, sendbuffersize);
    }
    
    if (total2>2000){
      int value = 2;
      String order = String(value);
      uint8_t sendbuffer[20];
      order.getBytes(sendbuffer, 20);
      char sendbuffersize = min(20, order.length());
      BTLEserial.write(sendbuffer, sendbuffersize);
    }
    

    // Lets see if there's any data for us!
    if (BTLEserial.available()) {
      Serial.print("* "); 
      Serial.print(BTLEserial.available()); 
      Serial.println(F(" bytes available from BTLE"));
    }
    // OK while we still have something to read, get a character and print it out
    while (BTLEserial.available()) {
      char c = BTLEserial.read();
      Serial.print(c);
    }

    // Next up, see if we have any data to get from the Serial console

    if (Serial.available()) {
      // Read a line from Serial
      Serial.setTimeout(100); // 100 millisecond timeout
      String s = Serial.readString();     

      // We need to convert the line to bytes, no more than 20 at this time
      uint8_t sendbuffer[20];
      s.getBytes(sendbuffer, 20);
      char sendbuffersize = min(20, s.length());

      Serial.print(F("\n* Sending -> \"")); 
      Serial.print((char *)sendbuffer); 
      Serial.println("\"");

      // write the data
      BTLEserial.write(sendbuffer, sendbuffersize);
    }
  }
}

