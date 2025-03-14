#include "Arduino_BMI270_BMM150.h"
#include <ArduinoBLE.h>
#include "mbed.h"
#include <SimpleKalmanFilter.h>


SimpleKalmanFilter kf(1, 1, 0.01);

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

//PID Variables
float et_old,et_new, kp_et,ki_et,kd_et,et_integral, et_derivative;
float kp = 0; //Proportional
float ki = 0; //Integral
float kd = 0; //Derivative
float desired_angle = 0; //We always want the robot to be at a 0 degree pitch (angle about the y-axis)
float PID_OUTPUT = 0; //number between 0 - 100
float pi = 3.1415;
int resetIntegral = 0;
float t0, t1, dt, t2, t3;

float LEFT_FORWARD_OFFSET;
float LEFT_BACKWARD_OFFSET;
float RIGHT_FORWARD_OFFSET;
float RIGHT_BACKWARD_OFFSET;

//Bluetooth Declarations
#define BUFFER_SIZE 64

BLEService laptopMasterService("00000000-5EC4-4083-81CD-A10B8D5CF6EC");

BLEStringCharacteristic laptopMasterCharacteristic("00000001-5EC4-4083-81CD-A10B8D5CF6EC", BLERead | BLEWrite, BUFFER_SIZE);
BLEStringCharacteristic laptopMasterReceiveCharacteristic("00000002-5EC4-4083-81CD-A10B8D5CF6EC", BLERead | BLEWrite, BUFFER_SIZE);
// BLEStringCharacteristic laptopMasterPIDOUTPUTCharacteristic("00000003-5EC4-4083-81CD-A10B8D5CF6EC", BLERead | BLEWrite, BUFFER_SIZE);
// BLEStringCharacteristic laptopMasterKpOutputCharacteristic("00000004-5EC4-4083-81CD-A10B8D5CF6EC", BLERead | BLEWrite, BUFFER_SIZE);
// BLEStringCharacteristic laptopMasterKiOutputCharacteristic("00000005-5EC4-4083-81CD-A10B8D5CF6EC", BLERead | BLEWrite, BUFFER_SIZE);
// BLEStringCharacteristic laptopMasterKdOutputCharacteristic("00000006-5EC4-4083-81CD-A10B8D5CF6EC", BLERead | BLEWrite, BUFFER_SIZE);

BLEDevice laptopMaster;
int Laptop2RobotLength = 0;
int receiveLength = 0;

//IMU Variables
float gxbias,gybias,gzbias;
float accel_magnitude;
float threshold = 0.04;

// Kalman filter variables
float P[2][2] = {{1, 0}, {0, 1}};  // Error covariance matrix
float K[2];  // Kalman gain
float S;  // Innovation (or residual) covariance
float P00_temp, P01_temp, P10_temp, P11_temp;  // Temporary variables for calculations
float Q = 0.1; // Process noise covariance (gyro uncertainty)
float R = 0.5;   
double y;


void setup() {
  initializeAll();
}

void loop() {
  t2 = micros();

  if(resetIntegral == 1)
  {
    //if resetIntegral == 1 RESET else if 0 then do not reset
    et_integral = 0;
    resetIntegral = 0;
  }

  readIMUData();
  receiveBLE();
  
  t3 = micros();
  if(start == 1)
  {
    //Serial.println(t3-t2);
  }
}

void initializeAll() {
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1)
      ;
  }

  calibrateIMU();

  float pwm_frequency = 20000.0;  // 20 kHz
  float pwm_period = 1.0 / pwm_frequency;
  pwmA0.period(pwm_period);
  pwmA1.period(pwm_period);
  pwmA2.period(pwm_period);
  pwmA3.period(pwm_period);

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
  BLE.addService(laptopMasterService);

  BLE.advertise();

  while (true) {
    laptopMaster = BLE.central();
    if (laptopMaster) {
      Serial.println("Connected to MASTER LAPTOP");
      break;
    }
  }
}

void calibrateIMU()
{ 
  int iterations = 0;
  while(iterations < 1000)
  {
    if (IMU.gyroscopeAvailable() && IMU.accelerationAvailable()) {
      IMU.readGyroscope(gy_0, gx_0, gz_0);
      IMU.readAcceleration(ay_0, ax_0, az_0);

      gxbias += gx_0;
      gybias += gy_0;
      gzbias += gz_0;
      iterations++;
    }
  }

  gxbias /= 1000;
  gybias /= 1000;
  gzbias /= 1000;
}


void readIMUData() {
  if (IMU.gyroscopeAvailable() && IMU.accelerationAvailable()) {
    IMU.readGyroscope(gy_0, gx_0, gz_0);
    IMU.readAcceleration(ay_0, ax_0, az_0);

    gx_0 -= gxbias;
    gy_0 -= gybias;
    gz_0 -= gzbias;
    
    t1 = micros();
    dt = (t1 - t0) / 1000000.0;
    t0 = t1;

    Theta_Acc = atan(ax_0 / (az_0/1.01) )* 180 / 3.14159;
    Theta_Gyro = gy_0 *dt + Theta_Final;

    if(start == 1)
    {
      Theta_Final = (Theta_Gyro);
      PID();
    }
    else
    {
      // Theta_Final = (Theta_Gyro)*k2 + Theta_Acc * (1 - k2);
      Theta_Final = kf.updateEstimate(Theta_Acc);
  
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

    /*Serial.print(Theta_Final);
    Serial.print(" ");
    Serial.print("kp_et: ");
    Serial.print(kp_et);
    Serial.print(" ki_et: ");
    Serial.print(ki_et);
    Serial.print(" kd_et: ");
    Serial.print(kd_et);
    Serial.print(" ");
    Serial.println(PID_OUTPUT);*/

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
    pwmA2.write(1.0);
    pwmA3.write(1.0 - (LEFT_MOTOR_PWM_SPEED + LEFT_FORWARD_OFFSET));
  }
  else if(LEFT_MOTOR_DIR == 1)
  {
    pwmA2.write(1.0 - (LEFT_MOTOR_PWM_SPEED + LEFT_BACKWARD_OFFSET));
    pwmA3.write(1.0);
  }

  if(RIGHT_MOTOR_DIR == 0)
  {
    pwmA0.write(1.0);
    pwmA1.write(1.0 - (RIGHT_MOTOR_PWM_SPEED + RIGHT_FORWARD_OFFSET));
  }
  else if(RIGHT_MOTOR_DIR == 1)
  {
    pwmA0.write(1.0 - (RIGHT_MOTOR_PWM_SPEED + RIGHT_BACKWARD_OFFSET));
    pwmA1.write(1.0);
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

        kp = receiveString.substring(3, receiveString.indexOf(' ')).toFloat();
        receiveString = receiveString.substring(receiveString.indexOf(' ') + 1);
        ki = receiveString.substring(3, receiveString.indexOf(' ')).toFloat();
        receiveString = receiveString.substring(receiveString.indexOf(' ') + 1);
        kd = receiveString.substring(3, receiveString.indexOf(' ')).toFloat();
        receiveString = receiveString.substring(receiveString.indexOf(' ') + 1);
        resetIntegral = receiveString.substring(3, receiveString.indexOf(' ')).toInt();
        receiveString = receiveString.substring(receiveString.indexOf(' ') + 1);
        RIGHT_FORWARD_OFFSET = receiveString.substring(0, receiveString.indexOf(' ')).toFloat();
        receiveString = receiveString.substring(receiveString.indexOf(' ') + 1);
        RIGHT_BACKWARD_OFFSET = receiveString.substring(0, receiveString.indexOf(' ')).toFloat();
        receiveString = receiveString.substring(receiveString.indexOf(' ') + 1);
        LEFT_FORWARD_OFFSET = receiveString.substring(0, receiveString.indexOf(' ')).toFloat();
        receiveString = receiveString.substring(receiveString.indexOf(' ') + 1);
        LEFT_BACKWARD_OFFSET = receiveString.substring(0, receiveString.indexOf(' ')).toFloat();

        start = 1;

        /*Serial.print("Kp: ");
        Serial.print(kp);
        Serial.print(" Ki: ");
        Serial.print(ki);
        Serial.print(" Kd: ");
        Serial.print(kd);
        Serial.print(" LMF: ");
        Serial.print(LEFT_FORWARD_OFFSET);
        Serial.print(" LMB: ");
        Serial.print(LEFT_BACKWARD_OFFSET);
        Serial.print(" RMF: ");
        Serial.print(RIGHT_FORWARD_OFFSET);
        Serial.print(" RMB: ");
        Serial.println(RIGHT_BACKWARD_OFFSET);*/

        /*MAX_KP = kp*(MAX_TILT);
        MAX_KI = ki*pow((MAX_TILT),2)/2.0;
        MAX_KD = kd*(1.0/0.01); //1 degree in 10 ms is the assumed max change in error we expect.*/
      }
    }
  }
  laptopMasterCharacteristic.writeValue(String(Theta_Final, 2) + " " + String(PID_OUTPUT, 2) + " " + String(kp_et/100.0, 2) + " " + String(ki_et/100.0, 2) + " " + String(kd_et/100.0, 2));
  // laptopMasterPIDOUTPUTCharacteristic.writeValue(String(PID_OUTPUT, 4));
  // laptopMasterKpOutputCharacteristic.writeValue(String(kp_et/100.0, 4));
  // laptopMasterKiOutputCharacteristic.writeValue(String(ki_et/100.0, 4));
  // laptopMasterKdOutputCharacteristic.writeValue(String(kd_et/100.0, 4));
}
