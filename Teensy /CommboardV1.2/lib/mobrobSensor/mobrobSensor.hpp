#ifndef mobrobSensor_HPP
#define mobrobSensor_HPP

#include <cstdint>
#include <ArduinoJson.h>
#include <i2c_t3.h>
#include <Arduino.h>

#define jsonStringLen 1024

//AD-Channels-Addresses
#define AD0 0x08 // 0001000
#define AD1 0x09 // 0001001
#define AD2 0x1B // 0001010
#define AD3 0x0B // 0001011
#define AD4 0x18 // 0011000
#define AD5 0x19 // 0011001
#define AD6 0x1A // 0011010
#define AD7 0x0A // 0011011
#define AD8 0x28 // 0101000

//AD-Converter Channels
#define CH0       0x88 // 10001000
#define CH1       0xC8 // 11001000
#define CH2       0x98 // 10011000
#define CH3       0xD8 // 11011000
#define CH4       0xA8 // 10101000
#define CH5       0xE8 // 11101000
#define CH6       0xB8 // 10111000
#define CH7       0xF8 // 11111000
#define CHGLOBAL  0xD6 // 11010110

void readSensorData();
void convertSensorDataToJson(int sensorData[][8]);
void writeSensorDataToUSB(JsonObject& root);

class I2C {
    public:
        void initializeI2C();
        int getI2CSensorData(int add, int ch);
    private:
        uint8_t address[5] = {AD0, AD1, AD2, AD3, AD4};
        uint8_t channel[8] = {CH0, CH1, CH2, CH3, CH4, CH5, CH6, CH7};
};

#endif
