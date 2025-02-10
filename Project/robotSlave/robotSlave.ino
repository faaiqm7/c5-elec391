#include <mbed.h>
#include <ArduinoBLE.h>
#include "Arduino_BMI270_BMM150.h"
#include <math.h>
#include <HCSR04.h>

const float pi = 3.14159265358979323846;

//WHEEL MOTOR PINS
#define LEFT_MOTOR_FORWARD_PIN  A0
#define LEFT_MOTOR_BACKWARD_PIN  A1
#define RIGHT_MOTOR_FORWARD_PIN  A2
#define RIGHT_MOTOR_BACKWARD_PIN  A3

//DISTANCE SENSOR STEPPER MOTOR PINS
#define STEPPER_PIN_1 2
#define STEPPER_PIN_2 3
#define STEPPER_PIN_3 4
#define STEPPER_PIN_4 5
#define STEPPER_IR_SENSOR_PIN 6

using namespace mbed;
using namespace rtos;
using namespace std::chrono_literals;

Thread readIMUFunction;
Thread BLEReceiveSendFunction;
Thread PIDFunction;
Thread distanceSensorFunction;
Thread rotateStepperFunction;


#define BUFFER_SIZE 20

BLEService laptopMasterService("00000000-5EC4-4083-81CD-A10B8D5CF6EC");

BLEStringCharacteristic laptopMasterCharacteristicAngle("00000001-5EC4-4083-81CD-A10B8D5CF6EC", BLERead | BLEWrite, BUFFER_SIZE);
BLEStringCharacteristic laptopMasterReceiveCharacteristic("00000002-5EC4-4083-81CD-A10B8D5CF6EC", BLERead | BLEWrite, BUFFER_SIZE);
BLEStringCharacteristic laptopMasterCharacteristicCoordinates("00000003-5EC4-4083-81CD-A10B8D5CF6EC", BLERead | BLEWrite, BUFFER_SIZE);
BLEStringCharacteristic laptopMasterCharacteristicDistanceSensor("00000004-5EC4-4083-81CD-A10B8D5CF6EC", BLERead | BLEWrite, BUFFER_SIZE);

BLEDevice laptopMaster;

int Laptop2RobotLength = 0;

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

//Stepper Motor Variables
float stepper_angle = 0; //Clock-wise = Positive angle gain, Counter-clockwise = Negative angle gain
float angleStepRatio = 5.625/64; //5.625 degrees/64 steps approximately  == 0.088 degrees/step
float gearStepperRatio = 1;
int stepperHomingExtraSteps = 36;
bool direction = false; //False == CW Motion, True = CCW Motion
int step_number = 0;

//Mapping Variables
float x_coord = 0;
float y_coord = 0;

//Ultrasonic Sensor Variables
UltraSonicDistanceSensor distanceSensor(6, 5);  // Initialize sensor that uses digital pins 5 and 6.
float distanceSensed = 0; //in meters

//Motor Wheel Properties
float wheel_radius = 0.08/2.0; //including rubber tire (0.08 is diameter here)
float wheel_circumference = 2*pi*wheel_radius;
float metersPerDegree = wheel_circumference/360.0;

//PID Variables
float et, kp_et,ki_et,kd_et;
float kp = 1; //Proportional
float ki = 1; //Integral
float kd = 1; //Derivative
float desired_angle = 0; //We always want the robot to be at a 0 degree pitch (angle about the y-axis)
float time_0,dt;


void setup() {
  Serial.begin(9600);
  while (!Serial);

  initializeALL();
}

void loop() {
}

void initializeALL() {

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1)
      ;
  }

  if (!BLE.begin()) {
    Serial.println("* Starting Bluetooth® Low Energy module failed!");
    while (1)
      ;
  }

  BLE.setLocalName("C5-BLE");
  BLE.setDeviceName("C5-BLE");
  BLE.setAdvertisedService(laptopMasterService);

  laptopMasterService.addCharacteristic(laptopMasterCharacteristicAngle);
  laptopMasterService.addCharacteristic(laptopMasterReceiveCharacteristic);
  laptopMasterService.addCharacteristic(laptopMasterCharacteristicCoordinates);
  laptopMasterService.addCharacteristic(laptopMasterCharacteristicDistanceSensor);
  BLE.addService(laptopMasterService);

  BLE.advertise();
  connectToMasterLaptop();

  pinMode(STEPPER_PIN_1, OUTPUT);
  pinMode(STEPPER_PIN_2, OUTPUT);
  pinMode(STEPPER_PIN_3, OUTPUT);
  pinMode(STEPPER_PIN_4, OUTPUT);
  pinMode(STEPPER_IR_SENSOR_PIN, INPUT);
  stepperHoming(); //Initialize Angle 0;

  readIMUFunction.start(readIMUData);
  rotateStepperFunction.start(rotateStepper);
  distanceSensorFunction.start(readDistanceSensor);
  BLEReceiveSendFunction.start(Robot2MasterSendReceive);
  PIDFunction.start(PID);
  time_0 = millis(); //We initialize it here because we want dt to be based of right when PID starts.
}

void connectToMasterLaptop() {
  //Continously scan until the master laptop is connected to the arduino robot.
  while (true) {
    laptopMaster = BLE.central();
    if (laptopMaster) {
      Serial.println("Connected to MASTER LAPTOP");
      break;
    }
  }
}

void Robot2MasterSendReceive() {
  while (laptopMaster.connected()) {
    if (laptopMasterReceiveCharacteristic.written()) {

      int receiveLength = laptopMasterReceiveCharacteristic.valueLength();
      byte receiveBuffer[receiveLength];
      if (laptopMasterReceiveCharacteristic.readValue(receiveBuffer, receiveLength)) {

        String receiveString = String((char*)receiveBuffer);

        left_Motor_Speed = receiveString.substring(2, receiveString.indexOf(' ')).toInt();
        receiveString = receiveString.substring(receiveString.indexOf(' ') + 3);
        right_Motor_Speed = receiveString.substring(0, receiveString.indexOf(' ')).toInt();
        receiveString = receiveString.substring(receiveString.indexOf(' ') + 3);
        forward_Motor_Speed = receiveString.substring(0, receiveString.indexOf(' ')).toInt();
        receiveString = receiveString.substring(receiveString.indexOf(' ') + 3);
        back_Motor_Speed = receiveString.substring(0, receiveString.indexOf(' ')).toInt();

        Serial.print("L: ");
        Serial.print(left_Motor_Speed);
        Serial.print(" ");
        Serial.print("R: ");
        Serial.print(right_Motor_Speed);
        Serial.print(" ");
        Serial.print("F: ");
        Serial.print(forward_Motor_Speed);
        Serial.print(" ");
        Serial.print("B: ");
        Serial.println(back_Motor_Speed);
      }

    }

    // Read from IMU and Send Data to Laptop Master (convert float to byte array)
    laptopMasterCharacteristicAngle.writeValue(String(Theta_Final, 3));
    laptopMasterCharacteristicCoordinates.writeValue(String(x_coord, 2) + " " + String(y_coord, 2));
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

void rotateStepper()
{
  while(true)
  {
    OneStep(direction);
  }
}

void readDistanceSensor()
{
  while(true)
  {
    distanceSensed = distanceSensor.measureDistanceCm()/100.0;
  }
}

void PID()
{
  dt = (millis() - time_0)/1000.0; //converted to seconds
  



  time_0 = millis();
}

void stepperHoming()
{
  while(digitalRead(STEPPER_IR_SENSOR_PIN) == LOW)
  {
    OneStep(direction);
    delay(2);
  }
  while(stepperHomingExtraSteps > 0)
  {
    OneStep(direction);
    delay(2);
    stepperHomingExtraSteps--;
  }
  stepper_angle = 0;
}

void OneStep(bool dir){
  if(dir){
    stepper_angle -= angleStepRatio;
    switch(step_number){
      case 0:
        digitalWrite(STEPPER_PIN_1, HIGH);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
      case 1:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, HIGH);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
      case 2:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, HIGH);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
      case 3:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, HIGH);
        break;
    } 
  }
  else{
    stepper_angle += angleStepRatio;
    switch(step_number){
      case 0:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, HIGH);
        break;
      case 1:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, HIGH);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
      case 2:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, HIGH);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
      case 3:
        digitalWrite(STEPPER_PIN_1, HIGH);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, LOW); 
        break; 
    } 
  }
  step_number++;
  if(step_number > 3){
    step_number = 0;
  }

  if(stepper_angle > 360.00)
  {
    stepper_angle = stepper_angle - 360.00;
  }
  else if(stepper_angle < 0)
  {
    stepper_angle = 360.00 + stepper_angle;
  }
}