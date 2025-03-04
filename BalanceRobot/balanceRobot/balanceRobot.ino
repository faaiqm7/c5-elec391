#include <mbed.h>
#include "Arduino_BMI270_BMM150.h"
#include <math.h>

//WHEEL MOTOR PINS
#define RIGHT_MOTOR_FORWARD_PIN  A0
#define RIGHT_MOTOR_BACKWARD_PIN  A1
#define LEFT_MOTOR_FORWARD_PIN  A2
#define LEFT_MOTOR_BACKWARD_PIN  A3

//Gyroscope Values
float Theta_Gyro = 0;
//Accelerometer Values
float Theta_Acc = 0;
//Weighted Values
float Theta_Final = 0;
float k = 0.6;  //0.6 before
float gx_0, gy_0, gz_0, ax_0, ay_0, az_0, x_0, x;

//Wheel Motor Variables
int left_Motor_Speed, right_Motor_Speed, forward_Motor_Speed, back_Motor_Speed;

//PID Variables
float et_old,et_new, kp_et,ki_et,kd_et,et_integral, et_derivative;
float kp = 0.01; //Proportional
float ki = 0; //Integral
float kd = 0; //Derivative
float desired_angle = 0; //We always want the robot to be at a 0 degree pitch (angle about the y-axis)
float dt, t0,t1;
float PID_OUTPUT = 0; //number between 0 - 255
float pi = 3.1415;

int resetIntegral = 0;

int PID_MIN = -30;
int PID_MAX = 30;

int LEFT_FORWARD_OFFSET = 22;
int LEFT_BACKWARD_OFFSET = 22;
int RIGHT_FORWARD_OFFSET = 22;
int RIGHT_BACKWARD_OFFSET = 22;

float DEAD_ZONE_POSITIVE = 0;
float DEAD_ZONE_NEGATIVE = 0;

void setup() {
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

  //readIMUFunction.start(readIMUData);
  //PIDFunction.start(PID);

}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available())
  {
    String input = Serial.readString();

    kp = input.substring(0, input.indexOf(' ')).toFloat();
    input = input.substring(input.indexOf(' ') + 1);
    ki = input.substring(0, input.indexOf(' ')).toFloat();
    input = input.substring(input.indexOf(' ') + 1);
    kd = input.substring(0, input.indexOf(' ')).toFloat();
    input = input.substring(input.indexOf(' ') + 1);
    resetIntegral = input.substring(0, input.indexOf(' ')).toInt();
    input = input.substring(input.indexOf('-'));
    PID_MIN = input.substring(0, input.indexOf(' ')).toInt();
    input = input.substring(input.indexOf(' ') + 1);
    PID_MAX = input.substring(0, input.indexOf(' ')).toInt();
    input = input.substring(input.indexOf(' ') + 1);
    DEAD_ZONE_POSITIVE = input.substring(0, input.indexOf(' ')).toFloat();
    input = input.substring(input.indexOf('-'));
    DEAD_ZONE_NEGATIVE = input.substring(0, input.indexOf(' ')).toFloat();

    Serial.print("Kp: ");
    Serial.print(kp);
    Serial.print(" Ki: ");
    Serial.print(ki);
    Serial.print(" Kd: ");
    Serial.print(kd);
    Serial.print(" P_MIN: ");
    Serial.print(PID_MIN);
  }
  if(resetIntegral == 1)
  {
    //if resetIntegral == 1 RESET else if 0 then do not reset
    et_integral = 0;
    resetIntegral = 0;
  }

  readIMUData();

}

void readIMUData() {
  if (IMU.gyroscopeAvailable() && IMU.accelerationAvailable()) {

    IMU.readGyroscope(gy_0, gx_0, gz_0);
    IMU.readAcceleration(ay_0, ax_0, az_0);

    Theta_Acc = atan(ax_0 / az_0) * 180 / 3.14159;
    Theta_Gyro = gy_0*0.01 + Theta_Final;

    Theta_Final = (Theta_Gyro)*k + Theta_Acc * (1 - k);

    PID();
  }
}

void PID()
{

    t1 = millis();
    dt = (float)(t1 - t0)/1000.0;
    //Kp, Ki, and Kd are choosen for radians not for degrees.
    //et_new is in radians not degrees
    
    et_new = (desired_angle - Theta_Final)*pi/180.0;
    kp_et = kp*et_new;

    et_integral += et_new;
    //et_integral = constrain(et_integral, -1.0*pi/4.0, pi/4.0); //Constrain the integral error sum to not exceed 45 degrees.
    ki_et = ki*(et_integral)*dt;
    
    et_derivative = (et_new - et_old) / ((float)dt);
    kd_et = kd * et_derivative;

    PID_OUTPUT = (kp_et + ki_et - kd_et);
    PID_OUTPUT = constrain(PID_OUTPUT, -255, 255);

    if(Theta_Final > DEAD_ZONE_POSITIVE)
    {

      //NOTE TO SELF: PLAY AROUND WITH the not ground the other PWM pin when going forward or backward, etc.

      //PID_OUT will be negative (mostly)
      controlWheelMotors(abs(PID_OUTPUT), 0, abs(PID_OUTPUT), 0);
    }
    else if(Theta_Final < DEAD_ZONE_NEGATIVE)
    {
      //Theta < 0, PID_OUT will be positive (mostly)
      controlWheelMotors(PID_OUTPUT, 1, PID_OUTPUT, 1);
    }
    else if(Theta_Final < DEAD_ZONE_POSITIVE && Theta_Final >= 0 && et_derivative < 0)
    {
      //Fast Decay Stop when Theta hits close to 0 degrees from the positive side


      analogWrite(LEFT_MOTOR_FORWARD_PIN, 0);
      analogWrite(LEFT_MOTOR_BACKWARD_PIN, LEFT_BACKWARD_OFFSET + PID_MIN);
      analogWrite(RIGHT_MOTOR_FORWARD_PIN, 0);
      analogWrite(RIGHT_MOTOR_BACKWARD_PIN, RIGHT_BACKWARD_OFFSET + PID_MIN);
    }
    else if(Theta_Final > DEAD_ZONE_NEGATIVE && Theta_Final <= 0 && et_derivative > 0)
    {
      //Fast Decay Stop when Theta hits close to 0 degrees from the negative side

      analogWrite(LEFT_MOTOR_FORWARD_PIN, LEFT_FORWARD_OFFSET + PID_MIN);
      analogWrite(LEFT_MOTOR_BACKWARD_PIN, 0);
      analogWrite(RIGHT_MOTOR_FORWARD_PIN, RIGHT_FORWARD_OFFSET + PID_MIN);
      analogWrite(RIGHT_MOTOR_BACKWARD_PIN, 0);
    }

    Serial.print(kp_et, 0);
    Serial.print(" ");
    Serial.print(ki_et,1);
    Serial.print(" ");
    Serial.print(kd_et,2);

    Serial.print(" ");
    Serial.print(PID_OUTPUT, 0);
    Serial.print(" ");
    Serial.print(Theta_Final, 1);
    Serial.println(" ");


    et_old = et_new;
    t0 = t1;
    //ThisThread::sleep_for(5ms);
  
}

/************************************************************************************************************
 LEFT_MOTOR_PWM_SPEED (0,255) :
 LEFT_MOTOR_DIR (0 OR 1) : 0 = Forward Direction of Left Motor and 1 = Backward Direction of Left Motor
 RIGHT_MOTOR_PWM_SPEED (0,255) : 
 RIGHT_MOTOR_DIR (0 OR 1) : 0 = Forward Direction of Right Motor and 1 = Backward Direction of Right Motor
*************************************************************************************************************/

void controlWheelMotors(int LEFT_MOTOR_PWM_SPEED, int LEFT_MOTOR_DIR, int RIGHT_MOTOR_PWM_SPEED, int RIGHT_MOTOR_DIR)
{
  if(LEFT_MOTOR_DIR == 0)
  {
    analogWrite(LEFT_MOTOR_FORWARD_PIN, LEFT_FORWARD_OFFSET + PID_MIN + LEFT_MOTOR_PWM_SPEED);
    analogWrite(LEFT_MOTOR_BACKWARD_PIN, 0);
  }
  else if(LEFT_MOTOR_DIR == 1)
  {
    analogWrite(LEFT_MOTOR_FORWARD_PIN, 0);
    analogWrite(LEFT_MOTOR_BACKWARD_PIN, LEFT_BACKWARD_OFFSET + PID_MIN + LEFT_MOTOR_PWM_SPEED);
  }

  if(RIGHT_MOTOR_DIR == 0)
  {
    analogWrite(RIGHT_MOTOR_FORWARD_PIN, RIGHT_FORWARD_OFFSET + PID_MIN + RIGHT_MOTOR_PWM_SPEED);
    analogWrite(RIGHT_MOTOR_BACKWARD_PIN, 0);
  }
  else if(RIGHT_MOTOR_DIR == 1)
  {
    analogWrite(RIGHT_MOTOR_FORWARD_PIN, 0);
    analogWrite(RIGHT_MOTOR_BACKWARD_PIN, RIGHT_BACKWARD_OFFSET + PID_MIN + RIGHT_MOTOR_PWM_SPEED);
  }
}
