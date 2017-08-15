#include <DynamixelController.hpp>

void setup() {
    I2C i2c;
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
    Serial.begin(9600);
    init();
    i2c.initI2C();
    digitalWrite(13, LOW);
}

void loop() {
    requestHandler();
    Serial.println("YEP");
    delay(500);
}
