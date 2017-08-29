#include "mobrobSensor.hpp"

void I2C::initializeI2C() {
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
    Serial.println("I2C Initialization complete!");
}

void readSensorData() {
    I2C i2c;
    int sensorData[5][8] = {0};
    for (size_t i = 0; i < 2; i++) {
        for (size_t j = 0; j < 8; j++) {
            sensorData[i][j] = i2c.getI2CSensorData(i, j);
        }
    }
    for (size_t i = 0; i < 8; i++) {
        sensorData[4][i] = i2c.getI2CSensorData(4, i);
        sensorData[3][i] = i2c.getI2CSensorData(3, i);
        sensorData[2][i] = i2c.getI2CSensorData(2, i);

    }
    convertSensorDataToJson(sensorData);
}

int I2C::getI2CSensorData(int add, int ch) {
    Wire.beginTransmission(address[add]);     // slave addr
    Wire.write(channel[ch]);
    int status = Wire.endTransmission();
    if (status == 0)
    {
    Wire.requestFrom(address[add], 2, I2C_STOP);
    while (Wire.available()) {
      int byte1 = Wire.readByte();
      int byte2 = Wire.readByte();
      int number = byte2 | byte1 << 8;
      number = number / 16;
      return number;
    }
    }
    return -1;
}

void convertSensorDataToJson(int sensorData[][8]) {
    StaticJsonBuffer<jsonStringLen> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();
    root["data"] = "sensor";
    JsonArray& sensorFrontLeft = root.createNestedArray("FL");
    JsonArray& sensorFrontRight = root.createNestedArray("FR");
    JsonArray& sensorRight = root.createNestedArray("R");
    JsonArray& sensorLeft = root.createNestedArray("L");
    JsonArray& sensorBack = root.createNestedArray("B");
    //for (size_t i = 0; i < 2; i++) {
        for (int j = 0; j < 8; j++) {
            sensorFrontLeft.add(sensorData[0][j]);
            sensorFrontRight.add(sensorData[1][j]);
        }
    //}
    for (size_t i = 2; i < 5; i++) {
    }
    for (int j = 0; j < 8; j++) {
        sensorLeft.add(sensorData[4][j]);
        sensorBack.add(sensorData[3][j]);
        sensorRight.add(sensorData[2][j]);
    }
    writeSensorDataToUSB(root);
}

void writeSensorDataToUSB(JsonObject& root) {
    root.printTo(Serial);
}
