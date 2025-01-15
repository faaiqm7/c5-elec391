#include "Arduino_BMI270_BMM150.h"

void setup() {
    Serial.begin(9600); // Initialize serial communication
    while (!Serial);
    
    if (!IMU.begin()) {
        Serial.println("Failed to initialize IMU!");
        while (1);
    }
}

void loop() {
    float x, y, z;

    if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(x, y, z);

        // Send x, y, z values as comma-separated
        Serial.print(x, 2);
        Serial.print(',');
        Serial.print(y, 2);
        Serial.print(',');
        Serial.println(z, 2);
    }
    delay(100); // Adjust the delay for desired update rate
}
