#include "Arduino_BMI270_BMM150.h"
#include <Wire.h>
#define AS5600_I2C_ADDR 0x36  // AS5600 I2C address

// Gyroscope Values
float Theta_Gyro = 0;
// Accelerometer Values
float Theta_Acc = 0;
// Filtered Values
float Theta_Final = 0;
float gx_0, gy_0, gz_0, ax_0, ay_0, az_0;

float k2 = 0.6;
int start = 0; //if start == 1 (START!!)

void setup() {
  initializeAll();
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readString();

    start = input.substring(0, input.indexOf(' ')).toDouble();
  }
  readIMUData();
  Serial.print(Theta_Final);
  Serial.print(" ");
  Serial.print(50);
  Serial.print(" ");
  Serial.println(-50);
}

void initializeAll() {
  Serial.begin(115200);
  Wire.begin();
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1)
      ;
  }
}

void readIMUData() {
  if (IMU.gyroscopeAvailable() && IMU.accelerationAvailable()) {
    IMU.readGyroscope(gy_0, gx_0, gz_0);
    IMU.readAcceleration(ay_0, ax_0, az_0);

    Theta_Acc = atan(ax_0 / az_0) * 180 / 3.14159;
    Theta_Gyro = gy_0 * 0.01 + Theta_Final;
    if(start == 1)
    {
      Theta_Final = (Theta_Gyro);
    }
    else
    {
      Theta_Final = (Theta_Gyro)*k2 + Theta_Acc * (1 - k2);
    }
  }
}
