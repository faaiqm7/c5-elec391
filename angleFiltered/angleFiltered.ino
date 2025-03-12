#include "Arduino_BMI270_BMM150.h"
#include <Wire.h>
#include "mbed.h"
#define AS5600_I2C_ADDR 0x36  // AS5600 I2C address

//WHEEL MOTOR PINS
#define RIGHT_MOTOR_FORWARD_PIN  A0
#define RIGHT_MOTOR_BACKWARD_PIN  A1
#define LEFT_MOTOR_FORWARD_PIN  A2
#define LEFT_MOTOR_BACKWARD_PIN  A3

// Define PWM objects for the analog pins
mbed::PwmOut pwmA0(digitalPinToPinName(A2));
mbed::PwmOut pwmA1(digitalPinToPinName(A3));
mbed::PwmOut pwmA2(digitalPinToPinName(A0));
mbed::PwmOut pwmA3(digitalPinToPinName(A1));

// Gyroscope Values
float Theta_Gyro = 0;
// Accelerometer Values
float Theta_Acc = 0;
// Filtered Values
float Theta_Final = 0;
float gx_0, gy_0, gz_0, ax_0, ay_0, az_0;

float k2 = 0.6;
int start = 0; //if start == 1 (START!!)

//PID VARIABLES
//PID Variables
float et_old,et_new, kp_et,ki_et,kd_et,et_integral, et_derivative;
float kp = 0; //Proportional
float ki = 0; //Integral
float kd = 0; //Derivative
float desired_angle = 0; //We always want the robot to be at a 0 degree pitch (angle about the y-axis)
float PID_OUTPUT = 0; //number between 0 - 100
float pi = 3.1415;
int resetIntegral = 0;
float t0, t1, dt;

void setup() {
  initializeAll();
}

void loop() {

  if(Serial.available())
  {
    String input = Serial.readString();

    start = input.substring(0, input.indexOf(' ')).toDouble();
    input = input.substring(input.indexOf(' ') + 1);
    kp = input.substring(0, input.indexOf(' ')).toDouble();
    input = input.substring(input.indexOf(' ') + 1);
    ki = input.substring(0, input.indexOf(' ')).toDouble();
    input = input.substring(input.indexOf(' ') + 1);
    kd = input.substring(0, input.indexOf(' ')).toDouble();
    input = input.substring(input.indexOf(' ') + 1);
    resetIntegral = input.substring(0, input.indexOf(' ')).toInt();

  }
  if(resetIntegral == 1)
  {
    //if resetIntegral == 1 RESET else if 0 then do not reset
    et_integral = 0;
    resetIntegral = 0;
  }

  readIMUData();

}

void initializeAll() {
  Serial.begin(115200);
  Wire.begin();
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1)
      ;
  }
  float pwm_frequency = 20000.0;  // 20 kHz
  float pwm_period = 1.0 / pwm_frequency;
  pwmA0.period(pwm_period);
  pwmA1.period(pwm_period);
  pwmA2.period(pwm_period);
  pwmA3.period(pwm_period);
}

void readIMUData() {
  if (IMU.gyroscopeAvailable() && IMU.accelerationAvailable()) {
    IMU.readGyroscope(gy_0, gx_0, gz_0);
    IMU.readAcceleration(ay_0, ax_0, az_0);
    
    t1 = micros();
    dt = (t1 - t0) / 1000000.0;
    t0 = t1;

    Theta_Acc = atan(ax_0 / az_0) * 180 / 3.14159;
    Theta_Gyro = gy_0 * 0.01 + Theta_Final;
    if(start == 1)
    {
      Theta_Final = (Theta_Gyro);
      PID();
    }
    else
    {
      Theta_Final = (Theta_Gyro)*k2 + Theta_Acc * (1 - k2);
      SerialPrintFunctions();
    }
  }
}

void SerialPrintFunctions()
{
  Serial.println(Theta_Final);
}

void PID()
{
    //Kp, Ki, and Kd are choosen for radians not for degrees.
    //et_new is in radians not degrees
    
    et_new = (desired_angle - Theta_Final);
    //et_new = (desired_angle - Theta_Final); //degrees
    kp_et = kp*et_new;

    et_integral += et_new*(float)dt;
    ki_et = ki*et_integral;
    
    et_derivative = (et_new - et_old) / ((float)dt);
    kd_et = kd * et_derivative;

    //PID_OUTPUT = (kp_et + ki_et + kd_et)/(MAX_KP + MAX_KI + MAX_KD); // normalizing it to be between 0 and 1
    PID_OUTPUT = (kp_et + ki_et + kd_et);
    PID_OUTPUT = constrain(PID_OUTPUT, -100.0, 100.0);
    PID_OUTPUT /= 100.0;

    Serial.print(Theta_Final);
    Serial.print(" ");
    Serial.print("kp_et: ");
    Serial.print(kp_et);
    Serial.print(" ki_et: ");
    Serial.print(ki_et);
    Serial.print(" kd_et: ");
    Serial.print(kd_et);
    Serial.print(" ");
    Serial.println(PID_OUTPUT);

    if(PID_OUTPUT >= 0.00)
    {
      //PID_OUTPUT /= 1000.0;
      //PID_OUT will be negative (mostly)
      controlWheelMotors(abs(PID_OUTPUT), 0, abs(PID_OUTPUT), 0);
    }
    else if(PID_OUTPUT <= 0.00)
    {
      //PID_OUTPUT /= 1000.0;
      //Theta < 0, PID_OUT will be positive (mostly)
      controlWheelMotors(abs(PID_OUTPUT), 1, abs(PID_OUTPUT), 1);
    }

    et_old = et_new;

}

/************************************************************************************************************
 LEFT_MOTOR_PWM_SPEED (0,255) :
 LEFT_MOTOR_DIR (0 OR 1) : 0 = Forward Direction of Left Motor and 1 = Backward Direction of Left Motor
 RIGHT_MOTOR_PWM_SPEED (0,255) : 
 RIGHT_MOTOR_DIR (0 OR 1) : 0 = Forward Direction of Right Motor and 1 = Backward Direction of Right Motor
*************************************************************************************************************/

void controlWheelMotors(float LEFT_MOTOR_PWM_SPEED, float LEFT_MOTOR_DIR, float RIGHT_MOTOR_PWM_SPEED, float RIGHT_MOTOR_DIR)
{
  if(LEFT_MOTOR_DIR == 0)
  {
    pwmA2.write(LEFT_MOTOR_PWM_SPEED);
    pwmA3.write(0);
  }
  else if(LEFT_MOTOR_DIR == 1)
  {
    pwmA2.write(0);
    pwmA3.write(LEFT_MOTOR_PWM_SPEED);
  }

  if(RIGHT_MOTOR_DIR == 0)
  {
    pwmA0.write(RIGHT_MOTOR_PWM_SPEED);
    pwmA1.write(0);
  }
  else if(RIGHT_MOTOR_DIR == 1)
  {
    pwmA0.write(0);
    pwmA1.write(RIGHT_MOTOR_PWM_SPEED);
  }
}
