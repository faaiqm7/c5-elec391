#include <mbed.h>
#include "Arduino_BMI270_BMM150.h"
#include <math.h>

//WHEEL MOTOR PINS
#define RIGHT_MOTOR_FORWARD_PIN  A0
#define RIGHT_MOTOR_BACKWARD_PIN  A1
#define LEFT_MOTOR_FORWARD_PIN  A2
#define LEFT_MOTOR_BACKWARD_PIN  A3


using namespace mbed;
using namespace rtos;
using namespace std::chrono_literals;

Thread readIMUFunction;
Thread PIDFunction;

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
float et_old,et_new, kp_et,ki_et,kd_et,et_integral;
float kp = 0.01; //Proportional
float ki = 0; //Integral
float kd = 0; //Derivative
float desired_angle = 0; //We always want the robot to be at a 0 degree pitch (angle about the y-axis)
float dt = 0.005;
int PID_OUTPUT; //number between 0 - 255
float pi = 3.1415;
float beta = 0.6154;

//controlWheel Variables
int maxRPM = 467;
float RPMRequired = 0;
int DCycle = 0;
int DCycleLeftMin = 10;
int DCycleRightMin = 10;
int DCycleAdjustLeft = 0;
int DCycleAdjustRight = 0;

int resetIntegral = 0;

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

  readIMUFunction.start(readIMUData);
  PIDFunction.start(PID);

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

      Serial.print("Kp: ");
      Serial.print(kp);
      Serial.print(" Ki: ");
      Serial.print(ki);
      Serial.print(" Kd: ");
      Serial.println(kd);
    }
    if(resetIntegral == 1)
    {
      //if resetIntegral == 1 RESET else if 0 then do not reset
      et_integral = 0;
      resetIntegral = 0;
    }

}

void PID()
{
  while(true)
  {
  
    //Kp, Ki, and Kd are choosen for radians not for degrees.

    //et_new is in radians not degeres
    
    et_new = (float)(desired_angle - Theta_Final)*pi/180.0;

    //et_new = beta*et_old + (1-beta)*et_new;
    
    kp_et = kp*et_new;

    et_integral += et_new;
    ki_et = ki*(float)(et_integral);
    
    kd_et = kd * ((float)(et_new - et_old) / dt);

    PID_OUTPUT = (kp_et + ki_et + kd_et);

    Serial.print(kp_et);
    Serial.print(" ");
    Serial.print(ki_et,1);
    Serial.print(" ");
    Serial.print(kd_et,2);
    
    PID_OUTPUT = constrain(PID_OUTPUT, -100, 100);
    PID_OUTPUT = PID_OUTPUT*(1.0-abs(Theta_Final)/270.0);

    Serial.print(" ");
    Serial.print(PID_OUTPUT);
    Serial.print(" ");
    Serial.println(Theta_Final);

    if(Theta_Final >= 60 || Theta_Final <= -60)
    {
      //Turn of motors to avoid the robot going all crazy and reset the integral term
      kp = 0;
      ki = 0;
      kd = 0;
      et_integral = 0;
    }
    else if(Theta_Final > 3)
    {
      controlWheelMotors(PID_OUTPUT, 0 , PID_OUTPUT, 0);
    }
    else if(Theta_Final < -3)
    {
      //Theta_Final < 0
      controlWheelMotors(abs(PID_OUTPUT), 1, abs(PID_OUTPUT), 1);
    }
    else
    {
      //Slow Decay Brake
      analogWrite(RIGHT_MOTOR_FORWARD_PIN, 1);
      analogWrite(RIGHT_MOTOR_BACKWARD_PIN, 1);
      analogWrite(LEFT_MOTOR_FORWARD_PIN, 1);
      analogWrite(LEFT_MOTOR_BACKWARD_PIN, 1);

    }

    et_old = et_new;
    //ThisThread::sleep_for(5ms);
  }
  
}

void readIMUData() {
  while (true) {
    if (IMU.gyroscopeAvailable() && IMU.accelerationAvailable()) {

      IMU.readGyroscope(gy_0, gx_0, gz_0);
      IMU.readAcceleration(ay_0, ax_0, az_0);

      Theta_Acc = atan(ax_0 / az_0) * 180 / 3.14159;
      Theta_Gyro = gy_0 * 0.01 + Theta_Final;

      Theta_Final = (Theta_Gyro)*k + Theta_Acc * (1 - k);

    }
  }
}

/************************************************************************************************************
 LEFT_MOTOR_PWM_SPEED (0,100) : % of max RPM into Left Motor
 LEFT_MOTOR_DIR (0 OR 1) : 0 = Forward Direction of Left Motor and 1 = Backward Direction of Left Motor
 RIGHT_MOTOR_PWM_SPEED (0,100) : % of max RPM into Right Motor
 RIGHT_MOTOR_DIR (0 OR 1) : 0 = Forward Direction of Right Motor and 1 = Backward Direction of Right Motor
*************************************************************************************************************/

void controlWheelMotors(int LEFT_MOTOR_PWM_SPEED, int LEFT_MOTOR_DIR, int RIGHT_MOTOR_PWM_SPEED, int RIGHT_MOTOR_DIR)
{
  if(LEFT_MOTOR_DIR == 0)
  {
    analogWrite(LEFT_MOTOR_FORWARD_PIN, ((calcMotorSpeed(LEFT_MOTOR_PWM_SPEED, 0))/100.0)*255.0);
    analogWrite(LEFT_MOTOR_BACKWARD_PIN, 0);
  }
  else if(LEFT_MOTOR_DIR == 1)
  {
    analogWrite(LEFT_MOTOR_FORWARD_PIN, 0);
    analogWrite(LEFT_MOTOR_BACKWARD_PIN, ((calcMotorSpeed(LEFT_MOTOR_PWM_SPEED, 0))/100.0)*255.0);
  }

  if(RIGHT_MOTOR_DIR == 0)
  {
    analogWrite(RIGHT_MOTOR_FORWARD_PIN, ((calcMotorSpeed(RIGHT_MOTOR_PWM_SPEED, 1))/100.0)*255.0);
    analogWrite(RIGHT_MOTOR_BACKWARD_PIN, 0);
  }
  else if(RIGHT_MOTOR_DIR == 1)
  {
    analogWrite(RIGHT_MOTOR_FORWARD_PIN, 0);
    analogWrite(RIGHT_MOTOR_BACKWARD_PIN, ((calcMotorSpeed(RIGHT_MOTOR_PWM_SPEED, 1))/100.0)*255.0);
  }
}


//Returns Duty Cycle needed for % of maxRPM & if motor == 0 (LEFT MOTOR) OR 1 (RIGHT MOTOR) dir = 0 (FORWARD) & dir = 1 (BACKWARD)
int calcMotorSpeed(float percentMaxRPM, int motor)
{
  RPMRequired = (percentMaxRPM/100.0)*maxRPM;
  DCycle = 5.34 - 0.0167*RPMRequired + 1.47*pow(10,-3)*pow(RPMRequired,2) - 6.82*pow(10,-6)*pow(RPMRequired,3) + 1.01*pow(10,-8)*pow(RPMRequired,4);
  if(DCycle > 100)
  {
    DCycle = 100;
  }
  if(motor == 0)
  {
    return min(DCycle + DCycleLeftMin, 70);
  }
  else
  {
    return min(DCycle + DCycleRightMin,70);
  }
}

/*
NOTES FOR TOMMOROW:
1) Adjust the DCycleMin for LEFT and RIGHT motor
2) Adjust DCycleAdjust such that they start moving the same speeds at the same time
3) Then tune Kp, Ki, and Kd
*/

