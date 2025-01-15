#include "Arduino_BMI270_BMM150.h"

float Q0 = 0;
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
    float x_0, y_0, z_0, x_new, y_new, z_new;

    

    if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(x_0, y_0, z_0);

        // Send x, y, z values as comma-separated
        // Serial.print(x_0, 2);
        /*Serial.print(',');
        Serial.print(y_0, 2);
        Serial.print(',');
        Serial.println(z_0, 2);*/

        delay(10);

        float x;

        x = Q0 + x_0 * 0.01;
        Q0 = x;
        Q = Q + x;

        printf("%f\n", Q);
        
    }
    delay(100); // Adjust the delay for desired update rate
}


