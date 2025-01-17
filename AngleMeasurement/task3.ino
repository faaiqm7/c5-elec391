#include "Arduino_BMI270_BMM150.h"

float QG = 0;
float QFinal = 0;

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
        IMU.readGyroscope(gx_0, gy_0, gz_0);

        // Send x, y, z values as comma-separated
        // Serial.print(x_0, 2);
        /*Serial.print(',');
        Serial.print(y_0, 2);
        Serial.print(',');
        Serial.println(z_0, 2);*/

        QG = gz_0*0.01 + QFinal;

        Serial.print(QG);
        Serial.print('\n');
    }
}


