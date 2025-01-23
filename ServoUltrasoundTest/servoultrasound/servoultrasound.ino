#include <Servo.h>
#include <HCSR04.h>

Servo myservo;  // create Servo object to control a servo
// twelve Servo objects can be created on most boards

int pos = 0;    // variable to store the servo position
UltraSonicDistanceSensor distanceSensor(6, 5);  // Initialize sensor that uses digital pins 13 and 12.

void setup() {
  Serial.begin(9600);  // We initialize serial connection so that we could print values from sensor.
  myservo.attach(4);  // attaches the servo on pin 9 to the Servo object
}

void loop() {
  for (pos = 0; pos <= 65; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    Serial.print(pos);
    Serial.print(',');
    Serial.println(distanceSensor.measureDistanceCm());
    delay(5);                       // waits 15 ms for the servo to reach the position
  }
  for (pos = 65; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    Serial.print(pos);
    Serial.print(',');
    Serial.println(distanceSensor.measureDistanceCm());
    delay(5);                       // waits 15 ms for the servo to reach the position
  }
}
