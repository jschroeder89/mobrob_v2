#include <DynamixelController.hpp>

void setup() {
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
    Serial.begin(9600);
    init();
    I2C i2c;
    i2c.initializeI2C();
    digitalWrite(13, LOW);
}

void loop() {
    requestHandler();
    //delay(200);
}
