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

// Calibration variables
int calibrateIMUTime = 3000;  // 3 seconds for calibration
int calibrationIterations = 0;

// Kalman filter variables
float P[2][2] = {{1, 0}, {0, 1}};  // Error covariance matrix
float K[2];  // Kalman gain
float S;  // Innovation (or residual) covariance
float P00_temp, P01_temp, P10_temp, P11_temp;  // Temporary variables for calculations
float Q = 0.001; // Process noise covariance (gyro uncertainty)
float R = 0.03;   

double PID_OUTPUT, kp, ki, kd;
float PID_MOTOR_OUTPUT;

float LEFT_FORWARD_OFFSET = 0.790;
float LEFT_BACKWARD_OFFSET = 0.640;
float RIGHT_FORWARD_OFFSET = 0.790;
float RIGHT_BACKWARD_OFFSET = 0.645;

PID myPID(&Theta_Final, &PID_OUTPUT, 0, 5, 0, 1.5, DIRECT);

void setup() {
  // Initialize the IMU
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);  // Stop here if initialization fails
  }
  calibrateIMU();  // Calibrate IMU sensors

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
  
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-1000, 1000);
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

    myPID.SetTunings(kp, ki, kd);

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

void calibrateIMU() {
  // Calibrate the IMU by averaging the gyroscope values
  for (int i = 0; i < calibrateIMUTime; i++) {
    if (IMU.gyroscopeAvailable()) {
      IMU.readGyroscope(gx_0, gy_0, gz_0);
      gx_drift += gx_0;
      gy_drift += gy_0;
      gz_drift += gz_0;
      calibrationIterations++;
      delay(1);
    }
  }
  
  // Average the gyroscope values for drift correction
  gx_drift = gx_drift / calibrationIterations;
  gy_drift = gy_drift / calibrationIterations;
  gz_drift = gz_drift / calibrationIterations;
}

void readIMUData() {
  if (IMU.gyroscopeAvailable() && IMU.accelerationAvailable()) {
    // Read gyroscope and accelerometer data
    IMU.readGyroscope(gy_0, gx_0, gz_0);
    IMU.readAcceleration(ay_0, ax_0, az_0);

    // Correct for gyroscope drift
    gx_0 -= gx_drift;
    gy_0 -= gy_drift;
    gz_0 -= gz_drift;

    // Calculate time step (dt)
    t1 = micros();
    dt = (t1 - t0) / 1000000.0;
    t0 = t1;

    // Compute the accelerometer pitch angle in degrees
    Theta_Acc = atan(ax_0 / az_0) * 180 / 3.14159;

    // Compute the gyroscope pitch angle
    Theta_Gyro += gx_0 * dt;

    Theta_Final = (Theta_Gyro)*k + Theta_Acc * (1 - k);
    Serial.println(Theta_Final);

    //kalmanFilter();
    myPID.Compute();
    PID_MOTOR_OUTPUT = (float)PID_OUTPUT/1000.0;
  }

  if(PID_MOTOR_OUTPUT >= 0)
  {
    controlWheelMotors(PID_MOTOR_OUTPUT, 1, PID_MOTOR_OUTPUT, 1);
  }
  else
  {
    controlWheelMotors(abs(PID_MOTOR_OUTPUT), 0, abs(PID_MOTOR_OUTPUT), 0);
  }
}

void kalmanFilter()
{
  // Kalman Filter Update (combining the accelerometer and gyroscope angles)
    // Prediction Step
    P00_temp = P[0][0] + dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q);
    P01_temp = P[0][1] - dt * P[1][1];
    P10_temp = P[1][0] - dt * P[1][1];
    P11_temp = P[1][1] + Q;

    // Measurement Update
    S = P00_temp + R;  // Innovation (or residual) covariance
    K[0] = P00_temp / S;  // Kalman gain for the angle
    K[1] = P10_temp / S;  // Kalman gain for the rate

    // Compute the new angle estimate
    y = Theta_Acc - Theta_Gyro;  // Residual (difference between the accelerometer and gyroscope)
    Theta_Gyro += K[0] * y;
    
    // Update error covariance matrix
    P[0][0] = P00_temp - K[0] * P00_temp;
    P[0][1] = P01_temp - K[0] * P01_temp;
    P[1][0] = P10_temp - K[1] * P00_temp;
    P[1][1] = P11_temp - K[1] * P01_temp;

    // The final angle estimate after the Kalman filter update
    Theta_Final = Theta_Gyro;

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

