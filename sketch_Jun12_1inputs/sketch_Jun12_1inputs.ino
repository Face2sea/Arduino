#include <CapacitiveSensor.h>
#include "Adafruit_FloraPixel.h"


/*
 * CapitiveSense Library Demo Sketch
 * Paul Badger 2008
 * Uses a high value resistor e.g. 10M between send pin and receive pin
 * Resistor effects sensitivity, experiment with values, 50K - 50M. Larger resistor values yield larger sensor values.
 * Receive pin is the sensor pin - try different amounts of foil/metal on this pin
 * Modified by Becky Stern 2013 to change the color of one RGB Neo Pixel based on touch input
 */


CapacitiveSensor   cs_9_10 = CapacitiveSensor(9,10);        // 10M resistor between pins 4 & 2, pin 2 is sensor pin, add a wire and or foil if desired
//CapacitiveSensor   cs_9_2 = CapacitiveSensor(9,2);        // 10M resistor between pins 4 & 6, pin 6 is sensor pin, add a wire and or foil
//CapacitiveSensor   cs_4_8 = CapacitiveSensor(4,8);        // 10M resistor between pins 4 & 8, pin 8 is sensor pin, add a wire and or foil
Adafruit_FloraPixel strip = Adafruit_FloraPixel(1);

void setup()                    
{
   cs_9_10.set_CS_AutocaL_Millis(0xFFFFFFFF);     // turn off autocalibrate on channel 1 - just as an example
   Serial.begin(9600);
    strip.begin();
    strip.show();

}

void loop()                    
{
    long start = millis();
    long total1 =  cs_9_10.capacitiveSensor(30);
    //long total2 =  cs_9_2.capacitiveSensor(30);
    //long total3 =  cs_4_8.capacitiveSensor(30);


if (total1 > 550){
  strip.setPixelColor(0, Color(255,0,0));  
  strip.show();
}
else if (total1 > 500 & total1 < 550){

  strip.setPixelColor(0, Color(0,255,0));  
  strip.show();
}
else if (total1 > 450 & total1 < 500){
  //digitalWrite(7, HIGH);
  strip.setPixelColor(0, Color(0,0,255));  
  strip.show();
}
else if (total1 > 400 & total1 < 450){
  //digitalWrite(7, HIGH);
  strip.setPixelColor(0, Color(255,255,0));  
  strip.show();
}
else if (total1 > 300 & total1 < 400){
  //digitalWrite(7, HIGH);
  strip.setPixelColor(0, Color(255,0,255));  
  strip.show();
}
else if (total1 > 200 & total1 < 300){
  //digitalWrite(7, HIGH);
  strip.setPixelColor(0, Color(0,255,255));  
  strip.show();
}
else if (total1 > 100 & total1 < 200){
  //digitalWrite(7, HIGH);
  strip.setPixelColor(0, Color(255,255,255));  
  strip.show();
}
else if (total1 > 100 & total1 < 200){
  //digitalWrite(7, HIGH);
  strip.setPixelColor(0, Color(0,150,0));  
  strip.show();
}
else if (total1 > 40 & total1 < 100){
  //digitalWrite(7, HIGH);
  strip.setPixelColor(0, Color(0,100,0));  
  strip.show();
}
else if (total1 > 20 & total1 < 40){
  //digitalWrite(7, HIGH);
  strip.setPixelColor(0, Color(0,50,0));  
  strip.show();
}
else {
  //digitalWrite(7, HIGH);
  strip.setPixelColor(0, Color(0,0,255));  
  strip.show();
}
  
/*
if (total2> 4000){
  strip.setPixelColor(0, Color(255,0,0)); 
  strip.show();
}
*/  
  
    Serial.print(millis() - start);        // check on performance in milliseconds
    Serial.print("\t");                    // tab character for debug windown spacing

    Serial.println(total1);                  // print sensor output 1
    //Serial.print("\t");
    //Serial.println(total2);                  // print sensor output 2
    //Serial.print("\t");
    //Serial.println(total3);                // print sensor output 3

    delay(10);                             // arbitrary delay to limit data to serial port 
}

// Create a 24 bit color value from R,G,B
RGBPixel Color(byte r, byte g, byte b)
{
  RGBPixel p;
  
  p.red = r;
  p.green = g;
  p.blue = b;
  
  return p;
}
