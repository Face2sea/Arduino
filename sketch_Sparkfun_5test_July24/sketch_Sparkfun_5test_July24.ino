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
#define ADAFRUITBLE_REQ A0
#define ADAFRUITBLE_RDY 0     // This should be an interrupt pin, on Uno thats #2 or #3
#define ADAFRUITBLE_RST A1

Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);
/**************************************************************************/
/*!
 Configure the Arduino and start advertising with the radio
 */
/**************************************************************************/
CapacitiveSensor   cs_5_4 = CapacitiveSensor(5,4);        // 10M resistor between pins 0 & 2, pin 2 is sensor pin, add a wire and or foil if desired
CapacitiveSensor   cs_5_6 = CapacitiveSensor(5,6);        // 10M resistor between pins 0 & 2, pin 2 is sensor pin, add a wire and or foil if desired
CapacitiveSensor   cs_5_7 = CapacitiveSensor(5,7);        // 10M resistor between pins 0 & 2, pin 2 is sensor pin, add a wire and or foil if desired
CapacitiveSensor   cs_5_8 = CapacitiveSensor(5,8);        // 10M resistor between pins 0 & 2, pin 2 is sensor pin, add a wire and or foil if desired
CapacitiveSensor   cs_5_9 = CapacitiveSensor(5,9);        // 10M resistor between pins 0 & 2, pin 2 is sensor pin, add a wire and or foil if desired

CapacitiveSensor   cs_3_2 = CapacitiveSensor(3,2);
//CapacitiveSensor   cs_5_11 = CapacitiveSensor(5,11);
//CapacitiveSensor   cs_4_3 = CapacitiveSensor(4,3);
//CapacitiveSensor   cs_5_20 = CapacitiveSensor(5,20);

const int numReadings = 10;
const int numSensor = 6;
const int SNR_touch = 10;
const int SNR_appro = 5;
const int offset_touch = 10;
const int offset_appro = 10;
const int thr1 = 80;
const int thr2 = 60;

long readings[numSensor][numReadings];      // the tau readings from the analog
int index = 0;                  // the index of the current reading
int count = 1;                  // count of long term reading
long total[5];                  // the running total of # readings
long tau_filtered[5];                // the running average signal -- similar with low-pass filtered signal 
float longterm_mean[5];          // long term mean for detrending
float longterm_sd[5];            // long term s.d. of tau_filtered.
long tau_anomaly[5];                    // final signal for thresholding to generate orders
int spacing = 0;

long lastvalue = 0;
long hist3 = 0;
boolean touch1 = false;
boolean touch5 = false;
boolean handon2 = false;
boolean handon3 = false;
boolean handon4 = false;

void setup(void)
{ 
  cs_5_4.set_CS_AutocaL_Millis(0xFFFFFFFF);     // turn off autocalibrate on channel 1 - just as an example

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
    longterm_mean[sensor] = 0;
    longterm_sd[sensor] = 0;
    tau_anomaly[sensor] = 0;
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
  //long total1 =  cs_4_9.capacitiveSensor(30);
  //long total2 =  cs_0_3.capacitiveSensor(30);
  //long total3 =  cs_0_4.capacitiveSensor(30);
  //long total4 =  cs_0_5.capacitiveSensor(30);

  // Tell the nRF8001 to do whatever it should be working on.
  BTLEserial.pollACI();
  
  // running average filter; low-pass
  for (int sensor = 0; sensor < numSensor; sensor++)  
    total[sensor]= total[sensor] - readings[sensor][index];     // subtract the last reading 
    
    readings[0][index] = cs_5_4.capacitiveSensor(30);   // read from the sensor 
    readings[1][index] = cs_5_6.capacitiveSensor(30);   // read from the sensor
    readings[2][index] = cs_5_7.capacitiveSensor(30);   // read from the sensor
    readings[3][index] = cs_5_8.capacitiveSensor(30);   // read from the sensor
    readings[4][index] = cs_5_9.capacitiveSensor(30);   // read from the sensor
    readings[5][index] = cs_3_2.capacitiveSensor(30);   // read from the sensor
    //readings[6][index] = cs_5_11.capacitiveSensor(30);   // read from the sensor
    //readings[7][index] = cs_5_20.capacitiveSensor(30);   // read from the sensor
    //readings[8][index] = cs_4_3.capacitiveSensor(30);   // read from the sensor
    
  for (int sensor = 0; sensor < numSensor; sensor++) {  
    total[sensor]= total[sensor] + readings[sensor][index];       // add the reading to the total
    tau_filtered[sensor] = total[sensor] / numReadings;           // calculate the average
    // Detrending
    longterm_mean[sensor] = (longterm_mean[sensor]*(count-1) + tau_filtered[sensor])/count;  //iteratively calc. mean     
    tau_anomaly[sensor] = tau_filtered[sensor] - long(longterm_mean[sensor]);
    longterm_sd[sensor] = sqrt((pow(longterm_sd[sensor],2)*(count - 1) + pow(tau_anomaly[sensor],2))/count); //iteratively calc. sd    
  }

  index = index + 1;                    // advance to the next position in the array                      
  if (index >= numReadings)             // if we're at the end of the array... 
    index = 0;                          // ...wrap around to the beginning                     
  count++;                      // count accumulates 
  if (count > 10) {
    count = 1;                  // avoid "count" overflow
    for (int sensor = 0; sensor < numSensor; sensor++) {
      longterm_mean[sensor] = 0;
      longterm_sd[sensor] = 0;
    }    
  }
  
    Serial.print(tau_filtered[0]);                // print sensor output
    Serial.print('\t');
    Serial.print(tau_filtered[1]);
    Serial.print('\t');
    Serial.print(tau_filtered[2]);
    Serial.print('\t');
    Serial.print(tau_filtered[3]);
    Serial.print('\t');
    Serial.print(tau_filtered[4]);
    Serial.print('\t');
    //Serial.println(total1);
    Serial.println(tau_filtered[5]);                // print sensor output
    //Serial.print('\t');
    //Serial.print(tau_filtered[6]);
    //Serial.print('\t');
    //Serial.println(tau_filtered[7]);
    //Serial.print('\t');
    //Serial.println(tau_filtered[7]);
  
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
   
    // We need to convert the line to bytes, no more than 20 at this time
    /*
    String tau = String(tau_filtered[0]);
    uint8_t sendbuffer_tau[20];
    tau.getBytes(sendbuffer_tau, 20);
    char sendbuffersize_tau = min(20, tau.length());
    BTLEserial.write(sendbuffer_tau, sendbuffersize_tau);
    */
      
    if ( tau_filtered[0] > 600){
      if (spacing == 10){
        int value = 20;
        BTLEserial.write(value);
        spacing = 0;
      }
      spacing++;    
    }
    if ( tau_filtered[0] > 80 && tau_filtered[0]<=300){
        int value = 21;
        BTLEserial.write(value);
      spacing++;    
    }
    
    //lastvalue = tau_filtered[0];
    
    /*
    int value = tau_filtered[0];
    //String order5 = String(value);
    //uint8_t sendbuffer5[20];
    //order5.getBytes(sendbuffer5, 20);
    //char sendbuffersize5 = min(20, order5.length());
    //BTLEserial.write(sendbuffer5, sendbuffersize5);
    BTLEserial.write(value);
    */
    if (tau_filtered[4] > 300)
      touch1 = true;
    if (touch1 == true && tau_filtered[4] < 200){  
      int value1 = 1;
      BTLEserial.write(value1);
      touch1 = false;
    }
    
    if (tau_filtered[1] > thr1){
      handon2 = true;
    }
    if (handon2 == true && tau_filtered[1] < thr2){
        int value2 = 10;
        BTLEserial.write(value2);
        handon2 = false;
    }
 
    if (tau_filtered[2] > thr1){
      handon3 = true;
    }   
    if (handon3 == true && tau_filtered[2] < thr2){
        int value3 = 11;
        BTLEserial.write(value3);
        handon3 = false;
    }
    
    if (tau_filtered[3] > thr1){
      handon4 = true;
    }   
    if (handon4 == true && tau_filtered[3] < thr2){
        int value4 = 12;
        BTLEserial.write(value4);
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

