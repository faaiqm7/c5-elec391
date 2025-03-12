#include "mbed.h"
#include "Arduino_BMI270_BMM150.h"
#include <ArduinoBLE.h>
#include <PID_v1.h>

//WHEEL MOTOR PINS
#define RIGHT_MOTOR_FORWARD_PIN  A0
#define RIGHT_MOTOR_BACKWARD_PIN  A1
#define LEFT_MOTOR_FORWARD_PIN  A2
#define LEFT_MOTOR_BACKWARD_PIN  A3

// Define PWM objects for the analog pins
mbed::PwmOut pwmA0(digitalPinToPinName(A0));
mbed::PwmOut pwmA1(digitalPinToPinName(A1));
mbed::PwmOut pwmA2(digitalPinToPinName(A2));
mbed::PwmOut pwmA3(digitalPinToPinName(A3));

// Gyroscope Values
float Theta_Gyro = 0;  // Gyroscope angle estimate
// Accelerometer Values
float Theta_Acc = 0;    // Accelerometer angle estimate
// Weighted Values (Final Tilt Angle)
double Theta_Final = 0;  
float Theta_Old = 0;
double Theta_Raw = 0;
float newAngle = 0;
double y; // Kalman filter measurement residual
double k = 0.9;

//Wheel Motor Variables
float left_Motor_Speed, right_Motor_Speed, forward_Motor_Speed, back_Motor_Speed;

// IMU variables
float gx_0, gy_0, gz_0, ax_0, ay_0, az_0, gx_drift, gy_drift, gz_drift, gyroPitch, accelPitch;
float gyroUncertainty = 0.07;  // Gyro uncertainty (in radians per second)
float accelUncertainty = 0.03; // Accelerometer uncertainty (in radians)
float t0, t1, t2, t3, dt;

//PID Variables
float et_old,et_new, kp_et,ki_et,kd_et,et_integral, et_derivative;
float kp = 0; //Proportional
float ki = 0; //Integral
float kd = 0; //Derivative
float desired_angle = 0; //We always want the robot to be at a 0 degree pitch (angle about the y-axis)
float PID_OUTPUT = 0; //number between 0 - 100
float pi = 3.1415;
int resetIntegral = 0;

float LEFT_FORWARD_OFFSET = 0.790;
float LEFT_BACKWARD_OFFSET = 0.640;
float RIGHT_FORWARD_OFFSET = 0.790;
float RIGHT_BACKWARD_OFFSET = 0.645;

void setup() {
  // Initialize the IMU
  Serial.begin(115200);
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);  // Stop here if initialization fails
  }

  float pwm_frequency = 20000.0;  // 20 kHz
  float pwm_period = 1.0 / pwm_frequency;
  pwmA0.period(pwm_period);
  pwmA1.period(pwm_period);
  pwmA2.period(pwm_period);
  pwmA3.period(pwm_period);

  pwmA0.write(0);
  pwmA1.write(0);
  pwmA2.write(0);
  pwmA3.write(0);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available())
  {
    String input = Serial.readString();

    kp = input.substring(0, input.indexOf(' ')).toDouble();
    input = input.substring(input.indexOf(' ') + 1);
    ki = input.substring(0, input.indexOf(' ')).toDouble();
    input = input.substring(input.indexOf(' ') + 1);
    kd = input.substring(0, input.indexOf(' ')).toDouble();
    input = input.substring(input.indexOf(' ') + 1);
    LEFT_FORWARD_OFFSET = input.substring(0, input.indexOf(' ')).toFloat();
    input = input.substring(input.indexOf(' ') + 1);
    LEFT_BACKWARD_OFFSET = input.substring(0, input.indexOf(' ')).toFloat();
    input = input.substring(input.indexOf(' ') + 1);
    RIGHT_FORWARD_OFFSET = input.substring(0, input.indexOf(' ')).toFloat();
    input = input.substring(input.indexOf(' ') + 1);
    RIGHT_BACKWARD_OFFSET = input.substring(0, input.indexOf(' ')).toFloat();


  }
  readIMUData();
  /*Serial.print("Kp: ");
  Serial.print(kp);
  Serial.print(" Ki: ");
  Serial.print(ki);
  Serial.print(" Kd: ");
  Serial.print(kd);
  Serial.print(" Out:");*/

}

void readIMUData() {
  if (IMU.gyroscopeAvailable() && IMU.accelerationAvailable()) {
    // Read gyroscope and accelerometer data
    IMU.readGyroscope(gy_0, gx_0, gz_0);
    IMU.readAcceleration(ay_0, ax_0, az_0);

    float accel_magnitude = sqrt(pow(ax_0, 2) + pow(ay_0, 2) + pow(az_0, 2));

    if(abs(accel_magnitude - 1) > 0.01)
    {
      k = 1;
    }
    else
    {
      k = 0.95;
    }

    // Calculate time step (dt)
    t1 = micros();
    dt = (t1 - t0) / 1000000.0;
    t0 = t1;

    // Compute the accelerometer pitch angle in degrees
    Theta_Acc = atan(ax_0 / az_0) * 180 / 3.14159;

    // Compute the gyroscope pitch angle
    Theta_Gyro += gx_0 * dt;

    Theta_Final = (Theta_Gyro)*k + Theta_Acc * (1 - k);
    Serial.print(Theta_Final);

    PID();

  }

  if(PID_OUTPUT >= 0)
  {
    controlWheelMotors(PID_OUTPUT, 1, PID_OUTPUT, 1);
  }
  else
  {
    controlWheelMotors(abs(PID_OUTPUT), 0, abs(PID_OUTPUT), 0);
  }
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

    Serial.print(" kp_et: ");
    Serial.print(kp_et);
    Serial.print(" ki_et: ");
    Serial.print(ki_et);
    Serial.print(" kd_et: ");
    Serial.print(kd_et);
    Serial.print(" ");
    Serial.println(PID_OUTPUT);

    if(PID_OUTPUT <= 0.00)
    {
      //PID_OUTPUT /= 1000.0;
      //PID_OUT will be negative (mostly)
      controlWheelMotors(abs(PID_OUTPUT), 0, abs(PID_OUTPUT), 0);
    }
    else if(PID_OUTPUT >= 0.00)
    {
      //PID_OUTPUT /= 1000.0;
      //Theta < 0, PID_OUT will be positive (mostly)
      controlWheelMotors(PID_OUTPUT, 1, PID_OUTPUT, 1);
    }

    et_old = et_new;
}


void controlWheelMotors(float LEFT_MOTOR_PWM_SPEED, float LEFT_MOTOR_DIR, float RIGHT_MOTOR_PWM_SPEED, float RIGHT_MOTOR_DIR)
{
  if(LEFT_MOTOR_DIR == 0)
  {
    pwmA2.write(LEFT_MOTOR_PWM_SPEED + LEFT_FORWARD_OFFSET);
    pwmA3.write(0);
  }
  else if(LEFT_MOTOR_DIR == 1)
  {
    pwmA2.write(0);
    pwmA3.write(LEFT_MOTOR_PWM_SPEED + LEFT_BACKWARD_OFFSET);
  }

  if(RIGHT_MOTOR_DIR == 0)
  {
    pwmA0.write(RIGHT_MOTOR_PWM_SPEED + RIGHT_FORWARD_OFFSET);
    pwmA1.write(0);
  }
  else if(RIGHT_MOTOR_DIR == 1)
  {
    pwmA0.write(0);
    pwmA1.write(RIGHT_MOTOR_PWM_SPEED + RIGHT_BACKWARD_OFFSET);
  }
}