#include <mbed.h>
#include "Arduino_BMI270_BMM150.h"
#include <math.h> 

#define LEFT_MOTOR_FORWARD_PIN  A0
#define LEFT_MOTOR_BACKWARD_PIN  A1
#define RIGHT_MOTOR_FORWARD_PIN  A2
#define RIGHT_MOTOR_BACKWARD_PIN  A3

using namespace mbed;
using namespace rtos;
using namespace std::chrono_literals;

Thread readIMU;
Thread moveMotors;

//Gyroscope Values
float Theta_Gyro = 0;

//Accelerometer Values
float Theta_Acc = 0;

//Weighted Values
float Theta_Final = 0;
float k = 0.6; //0.6 before

float gx_0, gy_0, gz_0, ax_0,ay_0,az_0, x_0, x;

/***************************************************************************************************
  positiveAngle from (0 Degrees to 90 Degrees) = 0% MAX RPM to 100% MAX RPM IN FORWARD DIRECTION
  negativeAngle from (0 Degrees to -90 Degrees) = 0% MAX RPM to 100% MAX RPM IN BACKWARD DIRECTION
***************************************************************************************************/

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while (!Serial);

  if (!IMU.begin()) {
        Serial.println("Failed to initialize IMU!");
        while (1);
  }

  pinMode(LEFT_MOTOR_FORWARD_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD_PIN, OUTPUT);

  readIMU.start(readIMUFunction);
  moveMotors.start(moveMotorsFunction);

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("Angle: ");
  Serial.print(Theta_Final);
  Serial.print(" Duty Cycle: ");
  Serial.println((Theta_Final/45.0)*100);

}

void readIMUFunction()
{
  while(true)
  {
    if (IMU.gyroscopeAvailable() && IMU.accelerationAvailable()) {

      IMU.readGyroscope(gy_0, gx_0, gz_0);
      IMU.readAcceleration(ay_0,ax_0,az_0);

      Theta_Acc = atan(ax_0/az_0) * 180/3.14159;
      Theta_Gyro = gy_0*0.01 + Theta_Final;

      Theta_Final = (Theta_Gyro)*k + Theta_Acc*(1-k);
    }
  }
}

/*
Theta_Final is between 0 to 45 and 0 to -45 degrees from 0 to max rpm
*/

void moveMotorsFunction()
{
  while(true)
  {
    if(Theta_Final == 0)
    {
      analogWrite(LEFT_MOTOR_FORWARD_PIN, 0);
      analogWrite(LEFT_MOTOR_BACKWARD_PIN, 0);
      analogWrite(RIGHT_MOTOR_FORWARD_PIN, 0);
      analogWrite(RIGHT_MOTOR_BACKWARD_PIN, 0);
    }
    else if(Theta_Final > 0 && Theta_Final <= 45)
    {
      analogWrite(LEFT_MOTOR_FORWARD_PIN, (Theta_Final/45.0)*255.0);
      analogWrite(LEFT_MOTOR_BACKWARD_PIN, 0);
      analogWrite(RIGHT_MOTOR_FORWARD_PIN, (Theta_Final/45.0)*255.0);
      analogWrite(RIGHT_MOTOR_BACKWARD_PIN, 0);
    }
    else if(Theta_Final < 0 && Theta_Final >= -45)
    {
      analogWrite(LEFT_MOTOR_FORWARD_PIN, 0);
      analogWrite(LEFT_MOTOR_BACKWARD_PIN, -(Theta_Final/45.0)*255.0);
      analogWrite(RIGHT_MOTOR_FORWARD_PIN, 0);
      analogWrite(RIGHT_MOTOR_BACKWARD_PIN, -(Theta_Final/45.0)*255.0);
    }
    else
    {
      //If angle is past the +- 45 degrees then just keep the current speed (should be max RPM technically)
    }
  }
}