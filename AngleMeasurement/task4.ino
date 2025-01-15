#include "Arduino_BMI270_BMM150.h"

//Gyroscope Values
float Q0 = 0;
float QG = 0;

//Accelerometer Values
float QA = 0;

//Weighted Values
float QFinal = 0;
float QFinal0 = 0;
float k = 0.6;

float gx_0, gy_0, gz_0, ax_0,ay_0,az_0, x_0, x;

void setup() {
    Serial.begin(9600); // Initialize serial communication
    while (!Serial);
    
    if (!IMU.begin()) {
        Serial.println("Failed to initialize IMU!");
        while (1);
    }
}

void loop() {

    if (IMU.gyroscopeAvailable() && IMU.accelerationAvailable()) {

        IMU.readGyroscope(gx_0, gy_0, gz_0);
        IMU.readAcceleration(ay_0,ax_0,az_0);

        /*Serial.print(ay_0);
        Serial.print('\t');
        Serial.print(ax_0);
        Serial.print('\t');
        Serial.print(az_0);
        Serial.println('\t');*/

        QA = atan(ax_0/az_0) * 180/3.14;

        QFinal = (gz_0*0.01 + QFinal0)*k + QA*(1-k);
        QFinal0 = QFinal;

        Serial.print(QFinal);
        Serial.println('\t');
    }
    //delay(100); // Adjust the delay for desired update rate
}
