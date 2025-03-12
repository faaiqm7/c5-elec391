#include "Arduino_BMI270_BMM150.h"
#include <Wire.h>
#define AS5600_I2C_ADDR  0x36  // AS5600 I2C address

/*
Name: Task 4 Arduino
Authors: Adarsh Sood, Samarr Parmaar. Faaiq Majeed (Group L2C C5)

 ################### IMPORTANT COMMENTS #####################
- For the Arduino the x-axis and y-axis are flipped/inverted (x-axis is actually y-axis vice-versa)
- From Figure 2 of the assignment PDF:
    - the y-axis on the figure is the Arduino z-axis
    - the x-axis on the figure is the Arduino y-axis
    - the z-axis on the figure is Arduino x-axis
*/

//Gyroscope Values
float Theta_Gyro = 0;
//Accelerometer Values
float Theta_Acc = 0;
//Weighted Values
float Theta_Final = 0;
float Theta_Bad = 0;
float k = 0.6; //0.6 before
float gx_0, gy_0, gz_0, ax_0,ay_0,az_0, x_0, x;

float motorLeftAngle0, motorLeftAngle1, motorLeftVelocity0, motorLeftVelocity1, motorLeftAcceleration;

void setup() {
    initializeAll();
}

void loop() {

   Serial.print(50);
   Serial.print(" ");
    if (IMU.gyroscopeAvailable() && IMU.accelerationAvailable()) {

        IMU.readGyroscope(gy_0, gx_0, gz_0);
        IMU.readAcceleration(ay_0,ax_0,az_0);

        float accel_magnitude = sqrt(pow(ax_0,2) + pow(ay_0,2) + pow(az_0,2));

        if(abs(accel_magnitude - 1) > 0.01)
        {
          k = 1;
        }
        else
        {
          k = 0.7;
        }


        Theta_Acc = atan(ax_0/az_0) * 180/3.14159;
        Theta_Gyro = gy_0*0.01 + Theta_Final;
        Theta_Final = (Theta_Gyro)*k + Theta_Acc*(1-k);
        Theta_Bad = (Theta_Gyro)*(0.7) + Theta_Acc*(1-0.7);

        motorLeftAngle1 = getAS5600Angle();
        if(motorLeftAngle0 >= motorLeftAngle1)
        {
          //forward direction
          motorLeftVelocity1 = (((360.0 - motorLeftAngle0) + motorLeftAngle1)/0.01)*(1/360.0)*60.0;
          if(abs(motorLeftVelocity1) >= 470)
          {
            motorLeftVelocity1 = motorLeftVelocity0;
          }
        }
        else if(motorLeftAngle1 > motorLeftAngle0)
        {
          motorLeftVelocity1 = ((motorLeftAngle1 - motorLeftAngle0)/0.01)*(1/360.0)*60.0;
          if(abs(motorLeftVelocity1) >= 470)
          {
            motorLeftVelocity1 = motorLeftVelocity0;
          }
        }

        motorLeftAcceleration = (((((motorLeftVelocity1 - motorLeftVelocity0)/60))*(2*3.14159*0.04)/0.01))/9.81;  

        motorLeftAngle0 = motorLeftAngle1;
        motorLeftVelocity0 = motorLeftVelocity1;

        Serial.print(Theta_Final, 2);
        Serial.print(" ");
        Serial.print(Theta_Bad, 2);
        Serial.print(" ");
        Serial.print(motorLeftAngle1, 2);
        Serial.print(" ");
        Serial.print(motorLeftAcceleration);

    }
     Serial.print(" ");
     Serial.println(-50);
}

void initializeAll()
{
  Serial.begin(115200);
  Wire.begin();
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
}

float getAS5600Angle() {
    Wire.beginTransmission(AS5600_I2C_ADDR);
    Wire.write(0x0E);  // Address of the high byte of the raw angle
    Wire.endTransmission();
    
    Wire.requestFrom(AS5600_I2C_ADDR, 2);
    if (Wire.available() == 2) {
        uint16_t rawAngle = (Wire.read() << 8) | Wire.read();
        return (rawAngle * 360.0) / 4096.0;  // Convert to degrees
    }
    
    return -1;  // Error reading angle
}
