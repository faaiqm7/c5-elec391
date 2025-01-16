#include "Arduino_BMI270_BMM150.h"

float Q = 0;
void setup() {
    Serial.begin(9600); // Initialize serial communication
    while (!Serial);
    
    if (!IMU.begin()) {
        Serial.println("Failed to initialize IMU!");
        while (1);
    }
}

void loop() {
    float ax, ay, az;

    if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(ay, ax, az); //Axis for X and Y are flipped

        Q = atan(ax/az);

        // Send x, y, z values as comma-separated
        Serial.print(Q);
        Serial.print('\n');
        //Serial.print(',');
        //Serial.print(y, 2);
        //Serial.print(',');
        //Serial.println(z, 2);
    }
}
