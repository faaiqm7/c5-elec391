#include <mbed.h>
#include <ArduinoBLE.h>
#include "Arduino_BMI270_BMM150.h"
#include <math.h>

using namespace mbed;
using namespace rtos;
using namespace std::chrono_literals;

Thread readIMUFunction;
Thread BLEReceiveSendFunction;


#define BUFFER_SIZE 20

BLEService laptopMasterService("00000000-5EC4-4083-81CD-A10B8D5CF6EC");

BLEStringCharacteristic laptopMasterCharacteristic("00000001-5EC4-4083-81CD-A10B8D5CF6EC", BLERead | BLEWrite, BUFFER_SIZE);
BLEStringCharacteristic laptopMasterReceiveCharacteristic("00000002-5EC4-4083-81CD-A10B8D5CF6EC", BLERead | BLEWrite, BUFFER_SIZE);

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

//Motor Variables
int left_Motor_Speed, right_Motor_Speed, forward_Motor_Speed, back_Motor_Speed;

void setup() {
  Serial.begin(9600);
  while (!Serial)
    ;

  initializeALL();
}

void loop() {
}

void initializeALL() {
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

  connectToMasterLaptop();

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1)
      ;
  }

  readIMUFunction.start(readIMUData);
  BLEReceiveSendFunction.start(Robot2MasterSendReceive);
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

      //Receiving data from Laptop Master
      //String Laptop2RobotReceived = laptopMasterReceiveCharacteristic.readValue();
    }

    // Read from IMU and Send Data to Laptop Master (convert float to byte array)
    laptopMasterCharacteristic.writeValue(String(Theta_Final, 3));
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