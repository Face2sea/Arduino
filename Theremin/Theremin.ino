#define BLEVEL 255
#define MINLEVEL 50

void setup()
{
    // initialize serial communication
    Serial.begin(57600);
    pinMode(0, OUTPUT);    //set D0 as output
    pinMode(1, OUTPUT);
    digitalWrite(0, LOW);   //turn it off initially
}

// the loop routine runs over and over again forever:
void loop()
{
    digitalWrite(0, HIGH);
    delay(10);
    int sensorValue = analogRead(A1);
    int r,g,b;
    if (sensorValue > MINLEVEL && sensorValue < BLEVEL + 1) {
        g = (sensorValue - MINLEVEL) / 2;
        b = 0;
    } else if (sensorValue > BLEVEL) {
        g = sensorValue - MINLEVEL;
        if (g > 255) {
            g = 255;
        }
        b = sensorValue - BLEVEL;
        if (b > 255) {
            b = 255;
        }
    } else {
        g = 0;
        b = 0;
    }
    Bean.setLed(0, g, b);
    tone(1, 300 + sensorValue * 10, 20);
}
