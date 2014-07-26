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
CapacitiveSensor   cs_0_3 = CapacitiveSensor(0,3);        // 10M resistor between pins 0 & 2, pin 2 is sensor pin, add a wire and or foil if desired
CapacitiveSensor   cs_0_4 = CapacitiveSensor(0,4);        // 10M resistor between pins 0 & 2, pin 2 is sensor pin, add a wire and or foil if desired
CapacitiveSensor   cs_0_5 = CapacitiveSensor(0,5);        // 10M resistor between pins 0 & 2, pin 2 is sensor pin, add a wire and or foil if desired

const int numReadings = 10;
const int numSensor = 4;

long readings[numSensor][numReadings];      // the tau readings from the analog
int index = 0;                  // the index of the current reading
long total[4];                  // the running total
long tau_filtered[4];                // the running average

long hist2 = 0;
long hist3 = 0;
boolean touch1 = false;
boolean handon2 = false;
boolean handon3 = false;
boolean handon4 = false;

void setup(void)
{ 
  cs_0_1.set_CS_AutocaL_Millis(0xFFFFFFFF);     // turn off autocalibrate on channel 1 - just as an example

  Serial.begin(9600);
  //while(!Serial); // Leonardo/Micro should wait for serial init
  Serial.println(F("Adafruit Bluefruit Low Energy nRF8001 Print echo demo"));
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    for (int sensor = 0; sensor < numSensor; sensor++)
       readings[sensor][thisReading] = 0;
  }
  for (int sensor = 0; sensor < numSensor; sensor++) {
    total[sensor] = 0;
    tau_filtered[sensor] = 0;
  }

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
  //long total1 =  cs_0_1.capacitiveSensor(30);
  //long total2 =  cs_0_3.capacitiveSensor(30);
  //long total3 =  cs_0_4.capacitiveSensor(30);
  //long total4 =  cs_0_5.capacitiveSensor(30);

  // Tell the nRF8001 to do whatever it should be working on.
  BTLEserial.pollACI();
  
  // running average filter; low-pass
  for (int sensor = 0; sensor < numSensor; sensor++)  
    total[sensor]= total[sensor] - readings[sensor][index];     // subtract the last reading 
    
    readings[0][index] = cs_0_1.capacitiveSensor(30);   // read from the sensor 
    readings[1][index] = cs_0_3.capacitiveSensor(30);   // read from the sensor
    readings[2][index] = cs_0_4.capacitiveSensor(30);   // read from the sensor
    readings[3][index] = cs_0_5.capacitiveSensor(30);   // read from the sensor
  for (int sensor = 0; sensor < numSensor; sensor++) {  
    total[sensor]= total[sensor] + readings[sensor][index];       // add the reading to the total
    tau_filtered[sensor] = total[sensor] / numReadings;           // calculate the average
  }

  index = index + 1;                  // advance to the next position in the array                      
  if (index >= numReadings)             // if we're at the end of the array... 
    index = 0;                          // ...wrap around to the beginning                     
   

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

    Serial.print(tau_filtered[0]);                // print sensor output
    Serial.print('\t');
    Serial.print(tau_filtered[1]);
    Serial.print('\t');
    Serial.print(tau_filtered[2]);
    Serial.print('\t');
    Serial.println(tau_filtered[3]);


    // We need to convert the line to bytes, no more than 20 at this time
    if (tau_filtered[0]>800)
      touch1 = true;
    if (touch1 == true && tau_filtered[0]<800){  
      int value = 1;
      String order = String(value);
      uint8_t sendbuffer[20];
      order.getBytes(sendbuffer, 20);
      char sendbuffersize = min(20, order.length());
      BTLEserial.write(sendbuffer, sendbuffersize);
      touch1 = false;
    }
    
    if (tau_filtered[1]>100){
      handon2 = true;
    }
    if (handon2 == true && tau_filtered[1] < 100){
        int value2 = 10;
        String order2 = String(value2);
        uint8_t sendbuffer2[20];
        order2.getBytes(sendbuffer2, 20);
        char sendbuffersize2 = min(20, order2.length());
        BTLEserial.write(sendbuffer2, sendbuffersize2);
        handon2 = false;
    }
 
    if (tau_filtered[2]>100){
      handon3 = true;
    }   
    if (handon3 == true && tau_filtered[2] < 100){
        int value3 = 11;
        String order3 = String(value3);
        uint8_t sendbuffer3[20];
        order3.getBytes(sendbuffer3, 20);
        char sendbuffersize3 = min(20, order3.length());
        BTLEserial.write(sendbuffer3, sendbuffersize3);
        handon3 = false;
    }
    
    if (tau_filtered[3]>100){
      handon4 = true;
    }   
    if (handon4 == true && tau_filtered[3] < 100){
        int value4 = 12;
        String order4 = String(value4);
        uint8_t sendbuffer4[20];
        order4.getBytes(sendbuffer4, 20);
        char sendbuffersize4 = min(20, order4.length());
        BTLEserial.write(sendbuffer4, sendbuffersize4);
        handon4 = false;
    }
    
    //hist2 = total2;
    //hist3 = total3;

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

