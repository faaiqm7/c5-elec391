#include <ArduinoBLE.h>
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
float dt, t0,t1,t2,t3;
float PID_OUTPUT = 0; //number between 0 - 255
float pi = 3.1415;

int resetIntegral = 0;

int PID_MIN = 0;
int PID_MAX = 0;

int LEFT_FORWARD_OFFSET = 26;
int LEFT_BACKWARD_OFFSET = 26;
int RIGHT_FORWARD_OFFSET = 26;
int RIGHT_BACKWARD_OFFSET = 26;

float DEAD_ZONE_POSITIVE = 0;
float DEAD_ZONE_NEGATIVE = 0;

float MAX_TILT = 40; //degrees
float MAX_KP = 0;
float MAX_KI = 0;
float MAX_KD = 0;

//Bluetooth Declarations
#define BUFFER_SIZE 20

BLEService laptopMasterService("00000000-5EC4-4083-81CD-A10B8D5CF6EC");

BLEStringCharacteristic laptopMasterCharacteristic("00000001-5EC4-4083-81CD-A10B8D5CF6EC", BLERead | BLEWrite, BUFFER_SIZE);
BLEStringCharacteristic laptopMasterReceiveCharacteristic("00000002-5EC4-4083-81CD-A10B8D5CF6EC", BLERead | BLEWrite, BUFFER_SIZE);
BLEStringCharacteristic laptopMasterPIDOUTPUTCharacteristic("00000003-5EC4-4083-81CD-A10B8D5CF6EC", BLERead | BLEWrite, BUFFER_SIZE);

BLEDevice laptopMaster;
int Laptop2RobotLength = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

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
  BLE.addService(laptopMasterService);

  BLE.advertise();

  while (true) {
    laptopMaster = BLE.central();
    if (laptopMaster) {
      Serial.println("Connected to MASTER LAPTOP");
      break;
    }
  }


  pinMode(LEFT_MOTOR_FORWARD_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD_PIN, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  if(resetIntegral == 1)
  {
    //if resetIntegral == 1 RESET else if 0 then do not reset
    et_integral = 0;
    resetIntegral = 0;
  }

  readIMUData();
  receiveBLE();

}

void receiveBLE()
{
  if(laptopMaster.connected())
  {
    if (laptopMasterReceiveCharacteristic.written()) {

      int receiveLength = laptopMasterReceiveCharacteristic.valueLength();
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

        MAX_KP = kp*(MAX_TILT*pi/180);
        MAX_KI = ki*pow((MAX_TILT*pi/180),2)/2;
        MAX_KD = kd*(1.0/0.01)*(pi/180);

        /*Serial.print("Kp: ");
        Serial.print(kp);
        Serial.print(" Ki: ");
        Serial.print(ki);
        Serial.print(" Kd: ");
        Serial.print(kd);
        Serial.print(" RI: ");
        Serial.println(resetIntegral);*/

      }

      //Receiving data from Laptop Master
      //String Laptop2RobotReceived = laptopMasterReceiveCharacteristic.readValue();
    }
  // Read from IMU and Send Data to Laptop Master (convert float to byte array)
  laptopMasterCharacteristic.writeValue(String(Theta_Final, 3));
  }
}

void readIMUData() {
  if (IMU.gyroscopeAvailable() && IMU.accelerationAvailable()) {

    t1 = millis();
    IMU.readGyroscope(gy_0, gx_0, gz_0);
    IMU.readAcceleration(ay_0, ax_0, az_0);

    Theta_Acc = atan(ax_0 / az_0) * 180 / 3.14159;
    Theta_Gyro = gy_0*0.01 + Theta_Final;

    Theta_Final = (Theta_Gyro)*k + Theta_Acc * (1 - k);
    

    PID();
    t0 = t1;
    laptopMasterPIDOUTPUTCharacteristic.writeValue(String(PID_OUTPUT, 4));
  }
}

void PID()
{

    dt = (float)(t1 - t0)/1000.0;
    //Kp, Ki, and Kd are choosen for radians not for degrees.
    //et_new is in radians not degrees
    
    et_new = (desired_angle - Theta_Final)*pi/180.0;
    kp_et = kp*et_new;

    et_integral += et_new*(float)dt;
    ki_et = ki*et_integral;
    
    et_derivative = (et_new - et_old) / ((float)dt);
    kd_et = kd * et_derivative;

    PID_OUTPUT = (kp_et + ki_et + kd_et)/(MAX_KP + MAX_KI + MAX_KD);

    //PID_OUTPUT = constrain(PID_OUTPUT, -100, 100);

    if(PID_OUTPUT <= 0.00)
    {
      //PID_OUT will be negative (mostly)
      controlWheelMotors(abs(PID_OUTPUT*(255 - 38.25)) + 10 , 0, (abs(PID_OUTPUT*(255 - 38.25)) + 10), 0);
    }
    else if(PID_OUTPUT >= 0.00)
    {
      //Theta < 0, PID_OUT will be positive (mostly)
      controlWheelMotors((PID_OUTPUT*(255 - 38.25) + 10), 1, (PID_OUTPUT*(255 - 38.25) + 10), 1);
    }

    /*Serial.print(kp_et, 0);
    Serial.print(" ");
    Serial.print(ki_et,1);
    Serial.print(" ");
    Serial.print(kd_et,2);

    Serial.print(" ");
    Serial.print(PID_OUTPUT, 5);
    Serial.print(" ");
    Serial.print(Theta_Final, 1);
    Serial.println(" ");*/


    et_old = et_new;
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
    analogWrite(LEFT_MOTOR_FORWARD_PIN, LEFT_FORWARD_OFFSET + LEFT_MOTOR_PWM_SPEED);
    analogWrite(LEFT_MOTOR_BACKWARD_PIN, LEFT_BACKWARD_OFFSET);
  }
  else if(LEFT_MOTOR_DIR == 1)
  {
    analogWrite(LEFT_MOTOR_FORWARD_PIN, LEFT_FORWARD_OFFSET);
    analogWrite(LEFT_MOTOR_BACKWARD_PIN, LEFT_BACKWARD_OFFSET + LEFT_MOTOR_PWM_SPEED);
  }

  if(RIGHT_MOTOR_DIR == 0)
  {
    analogWrite(RIGHT_MOTOR_FORWARD_PIN, RIGHT_FORWARD_OFFSET + RIGHT_MOTOR_PWM_SPEED);
    analogWrite(RIGHT_MOTOR_BACKWARD_PIN, RIGHT_BACKWARD_OFFSET);
  }
  else if(RIGHT_MOTOR_DIR == 1)
  {
    analogWrite(RIGHT_MOTOR_FORWARD_PIN, RIGHT_FORWARD_OFFSET);
    analogWrite(RIGHT_MOTOR_BACKWARD_PIN, RIGHT_BACKWARD_OFFSET + RIGHT_MOTOR_PWM_SPEED);
  }
}
