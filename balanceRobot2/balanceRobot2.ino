#include "mbed.h"
#include "Arduino_BMI270_BMM150.h"
#include <ArduinoBLE.h>


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
float Theta_Final = 0;  
float Theta_Old = 0;
double Theta_Raw = 0;
float newAngle = 0;
double y; // Kalman filter measurement residual
double k = 0.6;

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
float R = 0.03;   // Measurement noise covariance (accelerometer uncertainty)

//PID Variables
float et_old,et_new, kp_et,ki_et,kd_et,et_integral, et_derivative;
float kp = 0; //Proportional
float ki = 0; //Integral
float kd = 0; //Derivative
float desired_angle = 0; //We always want the robot to be at a 0 degree pitch (angle about the y-axis)
float PID_OUTPUT = 0; //number between 0 - 255
float pi = 3.1415;
int resetIntegral = 0;

float LEFT_FORWARD_OFFSET = 0.790;
float LEFT_BACKWARD_OFFSET = 0.640;
float RIGHT_FORWARD_OFFSET = 0.790;
float RIGHT_BACKWARD_OFFSET = 0.645;

float DEAD_ZONE_POSITIVE = 0;
float DEAD_ZONE_NEGATIVE = 0;

float MAX_TILT = 30; //degrees
float MAX_KP = 0;
float MAX_KI = 0;
float MAX_KD = 0;

//Bluetooth Declarations
#define BUFFER_SIZE 64

BLEService laptopMasterService("00000000-5EC4-4083-81CD-A10B8D5CF6EC");

BLEStringCharacteristic laptopMasterCharacteristic("00000001-5EC4-4083-81CD-A10B8D5CF6EC", BLERead | BLEWrite, BUFFER_SIZE);
BLEStringCharacteristic laptopMasterReceiveCharacteristic("00000002-5EC4-4083-81CD-A10B8D5CF6EC", BLERead | BLEWrite, BUFFER_SIZE);
BLEStringCharacteristic laptopMasterPIDOUTPUTCharacteristic("00000003-5EC4-4083-81CD-A10B8D5CF6EC", BLERead | BLEWrite, BUFFER_SIZE);
BLEStringCharacteristic laptopMasterKpOutputCharacteristic("00000004-5EC4-4083-81CD-A10B8D5CF6EC", BLERead | BLEWrite, BUFFER_SIZE);
BLEStringCharacteristic laptopMasterKiOutputCharacteristic("00000005-5EC4-4083-81CD-A10B8D5CF6EC", BLERead | BLEWrite, BUFFER_SIZE);
BLEStringCharacteristic laptopMasterKdOutputCharacteristic("00000006-5EC4-4083-81CD-A10B8D5CF6EC", BLERead | BLEWrite, BUFFER_SIZE);

BLEDevice laptopMaster;
int Laptop2RobotLength = 0;
int receiveLength = 0;


void setup() {
  // Initialize the IMU
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);  // Stop here if initialization fails
  }
  calibrateIMU();  // Calibrate IMU sensors

  if (!BLE.begin()) {
    Serial.println("* Starting Bluetooth® Low Energy module failed!");
    while (1)
      ;
  }

  BLE.setLocalName("C5-BLE");
  BLE.setDeviceName("C5-BLE");
  BLE.setAdvertisedService(laptopMasterService);

  laptopMasterService.addCharacteristic(laptopMasterCharacteristic);
  laptopMasterService.addCharacteristic(laptopMasterReceiveCharacteristic);
  laptopMasterService.addCharacteristic(laptopMasterPIDOUTPUTCharacteristic);
  laptopMasterService.addCharacteristic(laptopMasterKpOutputCharacteristic);
  laptopMasterService.addCharacteristic(laptopMasterKiOutputCharacteristic);
  laptopMasterService.addCharacteristic(laptopMasterKdOutputCharacteristic);
  BLE.addService(laptopMasterService);

  BLE.advertise();

  while (true) {
    laptopMaster = BLE.central();
    if (laptopMaster) {
      Serial.println("Connected to MASTER LAPTOP");
      break;
    }
  }

  /*pinMode(LEFT_MOTOR_FORWARD_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD_PIN, OUTPUT);*/

  float pwm_frequency = 20000.0;  // 20 kHz
  float pwm_period = 1.0 / pwm_frequency;
  pwmA0.period(pwm_period);
  pwmA1.period(pwm_period);
  pwmA2.period(pwm_period);
  pwmA3.period(pwm_period);

}

void loop() {
  if(resetIntegral == 1)
  {
    //if resetIntegral == 1 RESET else if 0 then do not reset
    et_integral = 0;
    resetIntegral = 0;
  }
  readIMUData();
  receiveBLE();
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
    IMU.readGyroscope(gx_0, gy_0, gz_0);
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

    //Theta_Raw = (Theta_Gyro)*k + Theta_Acc * (1 - k);

    kalmanFilter();
    laptopMasterCharacteristic.writeValue(String(Theta_Final, 2));

    PID();
  }
  else
  {
    kalmanFilter();
    laptopMasterCharacteristic.writeValue(String(Theta_Final, 2));
    PID();
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
    if(abs(Theta_Final - Theta_Old) < 0.05)
    {
      Theta_Final = Theta_Old;
    }
    else
    {
      Theta_Old = Theta_Final;
    }

}

void receiveBLE()
{
  if(laptopMaster.connected())
  {
    if (laptopMasterReceiveCharacteristic.written()) {

      receiveLength = laptopMasterReceiveCharacteristic.valueLength();
      byte receiveBuffer[receiveLength];
      if (laptopMasterReceiveCharacteristic.readValue(receiveBuffer, receiveLength)) {

        String receiveString = String((char*)receiveBuffer);

        //Serial.println(receiveString);

        kp = receiveString.substring(3, receiveString.indexOf(' ')).toFloat();
        receiveString = receiveString.substring(receiveString.indexOf(' ') + 1);
        ki = receiveString.substring(3, receiveString.indexOf(' ')).toFloat();
        receiveString = receiveString.substring(receiveString.indexOf(' ') + 1);
        kd = receiveString.substring(3, receiveString.indexOf(' ')).toFloat();
        receiveString = receiveString.substring(receiveString.indexOf(' ') + 1);
        resetIntegral = receiveString.substring(3, receiveString.indexOf(' ')).toInt();
        receiveString = receiveString.substring(receiveString.indexOf(' ') + 1);
        RIGHT_FORWARD_OFFSET = receiveString.substring(4, receiveString.indexOf(' ')).toFloat();
        receiveString = receiveString.substring(receiveString.indexOf(' ') + 1);
        RIGHT_BACKWARD_OFFSET = receiveString.substring(4, receiveString.indexOf(' ')).toFloat();
        receiveString = receiveString.substring(receiveString.indexOf(' ') + 1);
        LEFT_FORWARD_OFFSET = receiveString.substring(4, receiveString.indexOf(' ')).toFloat();
        receiveString = receiveString.substring(receiveString.indexOf(' ') + 1);
        LEFT_BACKWARD_OFFSET = receiveString.substring(4, receiveString.indexOf(' ')).toFloat();

        Serial.println(RIGHT_BACKWARD_OFFSET);
        RIGHT_BACKWARD_OFFSET = LEFT_BACKWARD_OFFSET;

        MAX_KP = kp*(MAX_TILT);
        MAX_KI = ki*pow((MAX_TILT),2)/2.0;
        MAX_KD = kd*(1.0/0.01); //1 degree in 10 ms is the assumed max change in error we expect.
      }
    }
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
    ki_et = constrain(ki_et, -1.0*MAX_KI, MAX_KI);
    
    et_derivative = (et_new - et_old) / ((float)dt);
    kd_et = kd * et_derivative;

    //PID_OUTPUT = (kp_et + ki_et + kd_et)/(MAX_KP + MAX_KI + MAX_KD); // normalizing it to be between 0 and 1
    PID_OUTPUT = (kp_et + ki_et + kd_et);
    PID_OUTPUT = constrain(PID_OUTPUT, -100.0, 100.0);
    PID_OUTPUT /= 100.0;

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
    laptopMasterPIDOUTPUTCharacteristic.writeValue(String(PID_OUTPUT, 4));
    laptopMasterKpOutputCharacteristic.writeValue(String(kp_et/100.0, 1));
    laptopMasterKiOutputCharacteristic.writeValue(String(ki_et/100.0, 1));
    laptopMasterKdOutputCharacteristic.writeValue(String(kd_et/100.0, 1));

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
    pwmA2.write(LEFT_MOTOR_PWM_SPEED + LEFT_FORWARD_OFFSET);
    pwmA3.write(0.55);
  }
  else if(LEFT_MOTOR_DIR == 1)
  {
    pwmA2.write(0.55);
    pwmA3.write(LEFT_MOTOR_PWM_SPEED + LEFT_BACKWARD_OFFSET);
  }

  if(RIGHT_MOTOR_DIR == 0)
  {
    pwmA0.write(RIGHT_MOTOR_PWM_SPEED + RIGHT_FORWARD_OFFSET);
    pwmA1.write(0.55);
  }
  else if(RIGHT_MOTOR_DIR == 1)
  {
    pwmA0.write(0.55);
    pwmA1.write(RIGHT_MOTOR_PWM_SPEED + RIGHT_BACKWARD_OFFSET);
  }
}

