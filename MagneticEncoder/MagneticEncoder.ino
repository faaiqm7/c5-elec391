#include <Wire.h>

#define AS5600_I2C_ADDR  0x36  // AS5600 I2C address

void setup() {
    Serial.begin(115200);
    Wire.begin();
}

void loop() {
    float angle = getAS5600Angle();
    Serial.print("Angle: ");
    Serial.print(angle);
    Serial.println("°");
    delay(100);
}

float getAS5600Angle() {
    Wire.beginTransmission(AS5600_I2C_ADDR);
    Wire.write(0x0E);  // Address of the high byte of the raw angle
    Wire.endTransmission();
    
    Wire.requestFrom(AS5600_I2C_ADDR, 2);
    if (Wire.available() == 2) {
        uint16_t rawAngle = (Wire.read() << 8) | Wire.read();
        return (rawAngle * 360.0) / 4096.0;  // Convert to degrees
    }
    
    return -1;  // Error reading angle
}
