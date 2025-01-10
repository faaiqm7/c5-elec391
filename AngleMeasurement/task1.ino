#include "Arduino_BMI270_BMM150.h"

void setup() {
    // put your setup code here, to run once:
    IMU.begin();

    if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    }

}

void loop() {
  
  float x, y, z;

if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x, y, z);

    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.println(z);
}



}