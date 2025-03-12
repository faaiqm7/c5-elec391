#include "Arduino_BMI270_BMM150.h"
#include <Wire.h>
#define AS5600_I2C_ADDR  0x36  // AS5600 I2C address

// Gyroscope Values
float Theta_Gyro = 0;
// Accelerometer Values
float Theta_Acc = 0;
// Filtered Values
float Theta_Final = 0;
float Theta_Bad = 0;
float k = 0.6; // Initial weight
float gx_0, gy_0, gz_0, ax_0, ay_0, az_0;

// Kalman filter variables
float Q_angle = 0.001; // Process noise variance for the angle
float Q_bias = 0.003;  // Process noise variance for the gyro bias
float R_measure = 0.03; // Measurement noise variance
float angle = 0;        // The angle calculated by the Kalman filter
float bias = 0;         // The gyro bias calculated by the Kalman filter
float P[2][2] = {{0, 0}, {0, 0}}; // Error covariance matrix

void setup() {
    initializeAll();
}

void loop() {
    Serial.print(50);
    Serial.print(" ");
    if (IMU.gyroscopeAvailable() && IMU.accelerationAvailable()) {
        IMU.readGyroscope(gy_0, gx_0, gz_0);
        IMU.readAcceleration(ay_0, ax_0, az_0);

        float accel_magnitude = sqrt(pow(ax_0, 2) + pow(ay_0, 2) + pow(az_0, 2));

        if(abs(accel_magnitude - 1) > 0.01)
        {
          k = 1;
        }
        else
        {
          0.7;
        }

        Theta_Acc = atan(ax_0 / az_0) * 180 / 3.14159;
        Theta_Gyro = gy_0 * 0.01 + Theta_Final;
        Theta_Final = (Theta_Gyro) * k + Theta_Acc * (1 - k);
        Theta_Bad = (Theta_Gyro) * 0.7 + Theta_Acc * (1 - 0.7);

        float Theta_Kalman = kalmanFilter(Theta_Acc, gy_0);

        Serial.print(Theta_Final, 2);
        Serial.print(" ");
        Serial.print(Theta_Bad, 2);
        Serial.print(" ");
        Serial.print(Theta_Kalman, 2);
    }
    Serial.print(" ");
    Serial.println(-50);
}

void initializeAll() {
    Serial.begin(115200);
    Wire.begin();
    if (!IMU.begin()) {
        Serial.println("Failed to initialize IMU!");
        while (1);
    }
}

float kalmanFilter(float newAngle, float newRate) {
    float dt = 0.01; // Time step
    
    // Predict
    angle += dt * (newRate - bias);
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;
    
    // Update
    float y = newAngle - angle;
    float S = P[0][0] + R_measure;
    float K[2] = {P[0][0] / S, P[1][0] / S};
    
    angle += K[0] * y;
    bias += K[1] * y;
    
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];
    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;
    
    return angle;
}
